#include "odometry.h"

#include <math.h>

#include "config.h"
#include "constants.h"
#include "delay.h"
#include "encoders.h"
#include "lsm6dsr.h"

/*
 * Odometría con EKF de 6 estados (Fase 3).
 *
 * Estado X = [ x, y, theta, v, omega, b_g ]
 *   x, y     : posición del centro del robot (mm), +x avance, +y izquierda
 *   theta    : rumbo (rad), CCW-positivo
 *   v        : velocidad lineal (mm/s)
 *   omega    : velocidad angular real de guiñada (rad/s), CCW-positiva
 *   b_g      : bias del giroscopio (rad/s)
 *
 * Predicción (cada ciclo, dt medido):
 *   x += v cos(theta) dt ; y += v sin(theta) dt ; theta += omega dt
 *   v, omega, b_g: paseo aleatorio
 *
 * Medidas:
 *   - Giroscopio: z = omega + b_g (se actualiza solo con muestra nueva).
 *   - Encoders:   z = [v, omega] (a partir de deltas crudos de ticks).
 *   - ZUPT:       z = [0, 0] cuando el robot está parado (observa el bias).
 *
 * La fusión con los sensores de línea (offset lateral absoluto) se hará en F3.3.
 */

#define EKF_N 6

enum ekf_state {
  S_X = 0,
  S_Y,
  S_TH,
  S_V,
  S_W,
  S_BG,
};

/* ---- Ruido de proceso Q (diagonal) ---- */
#define EKF_Q_X 0.01f
#define EKF_Q_Y 0.01f
#define EKF_Q_TH 1e-8f
#define EKF_Q_V 25.0f
#define EKF_Q_W 1e-4f
#define EKF_Q_BG 1e-10f

/* ---- Ruido de medida R (diagonal) ---- */
#define EKF_R_GYRO 4e-5f    /* (rad/s)^2 */
#define EKF_R_ENC_V 1225.0f /* (mm/s)^2 (cuantización ~35 mm/s) */
#define EKF_R_ENC_W 1.0f    /* (rad/s)^2 (cuantización ~1 rad/s) */
#define EKF_R_ZUPT_V 25.0f  /* (mm/s)^2 */
#define EKF_R_ZUPT_W 1e-4f  /* (rad/s)^2 */

/* ---- Covarianza inicial P0 (diagonal) ---- */
#define EKF_P0_X 1.0f
#define EKF_P0_Y 1.0f
#define EKF_P0_TH 1e-4f
#define EKF_P0_V 100.0f
#define EKF_P0_W 0.01f
#define EKF_P0_BG 1e-4f

/* ---- Gating de innovaciones (límites físicos anti-glitch/divergencia) ---- */
#define EKF_GYRO_INNOV_MAX 10.0f   /* rad/s */
#define EKF_ENC_V_INNOV_MAX 5000.0f /* mm/s */
#define EKF_ENC_W_INNOV_MAX 30.0f   /* rad/s */
#define EKF_ZUPT_V_INNOV_MAX 500.0f /* mm/s */
#define EKF_ZUPT_W_INNOV_MAX 5.0f   /* rad/s */

/* ---- Clamps de estado y suelo de varianza ---- */
#define EKF_MAX_V 8000.0f   /* mm/s */
#define EKF_MAX_W 40.0f     /* rad/s */
#define EKF_MAX_BG 1.0f     /* rad/s */
#define EKF_MIN_VAR 1e-9f

/* ---- Umbrales ---- */
#define ODOMETRY_ZUPT_MIN_CYCLES 200U
#define ODOMETRY_SLIP_WINDOW_CYCLES 100U
#define ODOMETRY_SLIP_THETA_THRESHOLD 0.2f
#define ODOMETRY_DT_MAX_S 0.005f

static float X[EKF_N];
static float P[EKF_N][EKF_N];

static volatile float last_dt;
static volatile float encoder_omega; /* medida de guiñada de los encoders */
static volatile float encoder_v;     /* medida de velocidad lineal de los encoders */
static volatile bool slip;

static uint16_t stationary_cycles;
static float slip_gyro_accum;
static float slip_encoder_accum;
static uint16_t slip_window_cycles;
static uint32_t last_cycles;
static bool dt_initialized;

/* ------------------------------------------------------------------------- */
/* Álgebra de matrices 6x6 / m x 6                                           */
/* ------------------------------------------------------------------------- */

static void mat_mul(float out[EKF_N][EKF_N], const float a[EKF_N][EKF_N], const float b[EKF_N][EKF_N]) {
  for (int i = 0; i < EKF_N; i++) {
    for (int j = 0; j < EKF_N; j++) {
      float s = 0.0f;
      for (int k = 0; k < EKF_N; k++) {
        s += a[i][k] * b[k][j];
      }
      out[i][j] = s;
    }
  }
}

static void mat_mul_transp(float out[EKF_N][EKF_N], const float a[EKF_N][EKF_N], const float b[EKF_N][EKF_N]) {
  /* out = a * b^T */
  for (int i = 0; i < EKF_N; i++) {
    for (int j = 0; j < EKF_N; j++) {
      float s = 0.0f;
      for (int k = 0; k < EKF_N; k++) {
        s += a[i][k] * b[j][k];
      }
      out[i][j] = s;
    }
  }
}

static void invert_small(const float s[3][3], float inv[3][3], int m) {
  if (m == 1) {
    inv[0][0] = 1.0f / s[0][0];
  } else if (m == 2) {
    float det = s[0][0] * s[1][1] - s[0][1] * s[1][0];
    float id = 1.0f / det;
    inv[0][0] = s[1][1] * id;
    inv[0][1] = -s[0][1] * id;
    inv[1][0] = -s[1][0] * id;
    inv[1][1] = s[0][0] * id;
  } else {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        inv[i][j] = (i == j) ? 1.0f : 0.0f;
      }
    }
  }
}

/* ------------------------------------------------------------------------- */
/* EKF                                                                        */
/* ------------------------------------------------------------------------- */

static float odometry_compute_dt(void) {
  uint32_t now = read_cycle_counter();
  if (!dt_initialized) {
    last_cycles = now;
    dt_initialized = true;
    return 1.0f / (float)UPDATE_FREQUENCY_HZ;
  }

  uint32_t delta_cycles = now - last_cycles;
  last_cycles = now;

  float dt = (float)delta_cycles / (float)SYSCLK_FREQUENCY_HZ;
  if (dt <= 0.0f || dt > ODOMETRY_DT_MAX_S) {
    dt = 1.0f / (float)UPDATE_FREQUENCY_HZ;
  }
  return dt;
}

static void ekf_predict(float dt) {
  float th = X[S_TH];
  float v = X[S_V];
  float w = X[S_W];
  float c = cosf(th);
  float s = sinf(th);

  /* Predicción del estado. */
  X[S_X] += v * c * dt;
  X[S_Y] += v * s * dt;
  X[S_TH] += w * dt;

  /* Jacobiano F. */
  static float F[EKF_N][EKF_N];
  for (int i = 0; i < EKF_N; i++) {
    for (int j = 0; j < EKF_N; j++) {
      F[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }
  F[S_X][S_TH] = -v * s * dt;
  F[S_X][S_V] = c * dt;
  F[S_Y][S_TH] = v * c * dt;
  F[S_Y][S_V] = s * dt;
  F[S_TH][S_W] = dt;

  /* P = F P F^T + Q */
  static float FP[EKF_N][EKF_N];
  static float FPFt[EKF_N][EKF_N];
  mat_mul(FP, F, P);
  mat_mul_transp(FPFt, FP, F);
  for (int i = 0; i < EKF_N; i++) {
    for (int j = 0; j < EKF_N; j++) {
      P[i][j] = FPFt[i][j];
    }
  }
  P[S_X][S_X] += EKF_Q_X;
  P[S_Y][S_Y] += EKF_Q_Y;
  P[S_TH][S_TH] += EKF_Q_TH;
  P[S_V][S_V] += EKF_Q_V;
  P[S_W][S_W] += EKF_Q_W;
  P[S_BG][S_BG] += EKF_Q_BG;
}

static bool ekf_update(const float H[3][EKF_N], const float *z, const float *r, const float *innov_max, int m, bool update_bias) {
  static float PHt[EKF_N][3];
  static float S[3][3];
  static float Sinv[3][3];
  static float K[EKF_N][3];
  float y[3];

  /* PHt = P H^T */
  for (int i = 0; i < EKF_N; i++) {
    for (int k = 0; k < m; k++) {
      float sum = 0.0f;
      for (int j = 0; j < EKF_N; j++) {
        sum += P[i][j] * H[k][j];
      }
      PHt[i][k] = sum;
    }
  }

  /* S = H P H^T + R */
  for (int a = 0; a < m; a++) {
    for (int b = 0; b < m; b++) {
      float sum = 0.0f;
      for (int i = 0; i < EKF_N; i++) {
        sum += H[a][i] * PHt[i][b];
      }
      S[a][b] = sum + ((a == b) ? r[a] : 0.0f);
    }
  }

  /* Innovación y = z - H X, con gating de outliers. */
  for (int a = 0; a < m; a++) {
    float sum = 0.0f;
    for (int i = 0; i < EKF_N; i++) {
      sum += H[a][i] * X[i];
    }
    y[a] = z[a] - sum;
    if (fabsf(y[a]) > innov_max[a]) {
      return false; /* medida descartada (glitch/divergencia) */
    }
  }

  invert_small(S, Sinv, m);

  /* K = PHt S^-1 */
  for (int i = 0; i < EKF_N; i++) {
    for (int a = 0; a < m; a++) {
      float sum = 0.0f;
      for (int b = 0; b < m; b++) {
        sum += PHt[i][b] * Sinv[b][a];
      }
      K[i][a] = sum;
    }
  }

  /*
   * El bias solo se estima con medidas fiables (ZUPT). Si update_bias es false,
   * se anula su fila de ganancia: el bias queda congelado durante la marcha.
   */
  if (!update_bias) {
    for (int a = 0; a < m; a++) {
      K[S_BG][a] = 0.0f;
    }
  }

  /* X += K y (y ya calculada arriba). */
  for (int i = 0; i < EKF_N; i++) {
    float sum = 0.0f;
    for (int a = 0; a < m; a++) {
      sum += K[i][a] * y[a];
    }
    X[i] += sum;
  }

  /* P = (I - K H) P */
  static float KH[EKF_N][EKF_N];
  static float newP[EKF_N][EKF_N];
  for (int i = 0; i < EKF_N; i++) {
    for (int j = 0; j < EKF_N; j++) {
      float sum = 0.0f;
      for (int a = 0; a < m; a++) {
        sum += K[i][a] * H[a][j];
      }
      KH[i][j] = sum;
    }
  }
  for (int i = 0; i < EKF_N; i++) {
    for (int j = 0; j < EKF_N; j++) {
      float sum = 0.0f;
      for (int k = 0; k < EKF_N; k++) {
        float ikh = ((i == k) ? 1.0f : 0.0f) - KH[i][k];
        sum += ikh * P[k][j];
      }
      newP[i][j] = sum;
    }
  }
  for (int i = 0; i < EKF_N; i++) {
    for (int j = 0; j < EKF_N; j++) {
      P[i][j] = newP[i][j];
    }
  }

  /* Simetrización para evitar deriva numérica en punto flotante. */
  for (int i = 0; i < EKF_N; i++) {
    for (int j = i + 1; j < EKF_N; j++) {
      float avg = 0.5f * (P[i][j] + P[j][i]);
      P[i][j] = avg;
      P[j][i] = avg;
    }
  }

  return true;
}

/* Clamps de estado y suelo de varianza (anti-divergencia). */
static void ekf_clamp(void) {
  if (X[S_V] > EKF_MAX_V) {
    X[S_V] = EKF_MAX_V;
  } else if (X[S_V] < -EKF_MAX_V) {
    X[S_V] = -EKF_MAX_V;
  }

  if (X[S_W] > EKF_MAX_W) {
    X[S_W] = EKF_MAX_W;
  } else if (X[S_W] < -EKF_MAX_W) {
    X[S_W] = -EKF_MAX_W;
  }

  if (X[S_BG] > EKF_MAX_BG) {
    X[S_BG] = EKF_MAX_BG;
  } else if (X[S_BG] < -EKF_MAX_BG) {
    X[S_BG] = -EKF_MAX_BG;
  }

  for (int i = 0; i < EKF_N; i++) {
    if (P[i][i] < EKF_MIN_VAR) {
      P[i][i] = EKF_MIN_VAR;
    }
  }
}

static void ekf_reset_covariance(void) {
  for (int i = 0; i < EKF_N; i++) {
    for (int j = 0; j < EKF_N; j++) {
      P[i][j] = 0.0f;
    }
  }
  P[S_X][S_X] = EKF_P0_X;
  P[S_Y][S_Y] = EKF_P0_Y;
  P[S_TH][S_TH] = EKF_P0_TH;
  P[S_V][S_V] = EKF_P0_V;
  P[S_W][S_W] = EKF_P0_W;
  P[S_BG][S_BG] = EKF_P0_BG;
}

/* ------------------------------------------------------------------------- */
/* API pública                                                                */
/* ------------------------------------------------------------------------- */

void odometry_init(void) {
  for (int i = 0; i < EKF_N; i++) {
    X[i] = 0.0f;
  }
  ekf_reset_covariance();
  last_dt = 0.0f;
  encoder_omega = 0.0f;
  slip = false;
  stationary_cycles = 0;
  slip_gyro_accum = 0.0f;
  slip_encoder_accum = 0.0f;
  slip_window_cycles = 0;
  dt_initialized = false;
}

void odometry_reset(void) {
  /* El bias es una propiedad del sensor: se conserva valor Y varianza para que
   * el reset sea coherente (no "olvidar" la confianza que teníamos en él). */
  float bias = X[S_BG];
  float bias_var = P[S_BG][S_BG];
  for (int i = 0; i < EKF_N; i++) {
    X[i] = 0.0f;
  }
  ekf_reset_covariance();
  X[S_BG] = bias;
  P[S_BG][S_BG] = bias_var;
  encoder_omega = 0.0f;
  slip = false;
  stationary_cycles = 0;
  slip_gyro_accum = 0.0f;
  slip_encoder_accum = 0.0f;
  slip_window_cycles = 0;
  dt_initialized = false;
}

void odometry_update(void) {
  float dt = odometry_compute_dt();
  last_dt = dt;

  /* Medidas de los encoders a partir de deltas crudos de ticks. */
  float dist_per_tick_mm = get_micrometers_per_tick() / (float)MICROMETERS_PER_MILLIMETER;
  float dl = (float)get_encoder_left_diff_ticks() * dist_per_tick_mm;
  float dr = (float)get_encoder_right_diff_ticks() * dist_per_tick_mm;
  float v_enc = 0.5f * (dl + dr) / dt;
  float w_enc = (dr - dl) / get_wheels_separation() / dt;
  encoder_omega = w_enc;
  encoder_v = v_enc;

  bool stationary = (get_encoder_left_diff_ticks() == 0) && (get_encoder_right_diff_ticks() == 0);
  if (stationary) {
    if (stationary_cycles < ODOMETRY_ZUPT_MIN_CYCLES + 1U) {
      stationary_cycles++;
    }
  } else {
    stationary_cycles = 0;
  }
  bool zero_velocity = stationary_cycles > ODOMETRY_ZUPT_MIN_CYCLES;

  /* Predicción. */
  ekf_predict(dt);

  /* Medida de velocity/velocidad angular de encoders. */
  const float H_enc[3][EKF_N] = {
      {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
      {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
      {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
  };
  float z_enc[2] = {v_enc, w_enc};
  float r_enc[2] = {EKF_R_ENC_V, EKF_R_ENC_W};
  float lim_enc[2] = {EKF_ENC_V_INNOV_MAX, EKF_ENC_W_INNOV_MAX};
  ekf_update(H_enc, z_enc, r_enc, lim_enc, 2, false);

  /* Medida del giroscopio (solo con muestra nueva): z = omega + b_g. */
  if (lsm6dsr_gyro_sample_ready()) {
    const float H_gyro[3][EKF_N] = {
        {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f},
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    };
    float z_gyro[1] = {lsm6dsr_get_gyro_z_radps()};
    float r_gyro[1] = {EKF_R_GYRO};
    float lim_gyro[1] = {EKF_GYRO_INNOV_MAX};
    ekf_update(H_gyro, z_gyro, r_gyro, lim_gyro, 1, false);
  }

  /* Pseudo-medida ZUPT: v = 0, omega = 0 (observa el bias en reposo). */
  if (zero_velocity) {
    const float H_zupt[3][EKF_N] = {
        {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
    };
    float z_zupt[2] = {0.0f, 0.0f};
    float r_zupt[2] = {EKF_R_ZUPT_V, EKF_R_ZUPT_W};
    float lim_zupt[2] = {EKF_ZUPT_V_INNOV_MAX, EKF_ZUPT_W_INNOV_MAX};
    ekf_update(H_zupt, z_zupt, r_zupt, lim_zupt, 2, true);
  }

  /* Clamps anti-divergencia. */
  ekf_clamp();

  /* Detección de derrape: ángulo integrado (gyro vs encoders) en una ventana. */
  slip_gyro_accum += lsm6dsr_get_gyro_z_radps() * dt;
  slip_encoder_accum += w_enc * dt;
  if (++slip_window_cycles >= ODOMETRY_SLIP_WINDOW_CYCLES) {
    slip = fabsf(slip_gyro_accum - slip_encoder_accum) > ODOMETRY_SLIP_THETA_THRESHOLD;
    slip_gyro_accum = 0.0f;
    slip_encoder_accum = 0.0f;
    slip_window_cycles = 0;
  }
}

float odometry_get_x(void) {
  return X[S_X];
}

float odometry_get_y(void) {
  return X[S_Y];
}

float odometry_get_theta_rad(void) {
  return X[S_TH];
}

float odometry_get_theta_deg(void) {
  return X[S_TH] * 180.0f / (float)PI;
}

float odometry_get_linear_speed(void) {
  return X[S_V];
}

float odometry_get_angular_speed(void) {
  return X[S_W];
}

float odometry_get_encoder_omega(void) {
  return encoder_omega;
}

float odometry_get_encoder_v(void) {
  return encoder_v;
}

float odometry_get_gyro_bias(void) {
  return X[S_BG];
}

bool odometry_get_slip(void) {
  return slip;
}

float odometry_get_dt(void) {
  return last_dt;
}

float odometry_get_var_x(void) {
  return P[S_X][S_X];
}

float odometry_get_var_y(void) {
  return P[S_Y][S_Y];
}

float odometry_get_var_theta(void) {
  return P[S_TH][S_TH];
}
