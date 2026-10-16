#include <kinematics.h>

/*
 * Conjunto de KPI por estrategia. Los KPI_LINEAR/KPI_ANGULAR son comunes; los de
 * línea (KPI_LINE_SENSORS) están escalados desde la referencia de SPEED_TEST
 * (kp=0.005, kd=0.03 a 2000 mm/s) por la regla ∝ 1/v (ver .features/pid_tuning).
 */
#define KPI_SET(line_kp, line_ki, line_kd)                                  \
  {                                                                         \
    [KPI_LINEAR] = {.kp = 0.003, .ki = 0.0002, .kd = 0.00},                 \
    [KPI_ANGULAR] = {.kp = 0.0, .ki = 0.0, .kd = 0.0},                      \
    [KPI_LINE_SENSORS] = {.kp = line_kp, .ki = line_ki, .kd = line_kd},     \
  }

static struct kpi_params kpi_test[] = KPI_SET(0.005, 0.0, 0.030);      /* v=2000 (referencia) */
static struct kpi_params kpi_normal[] = KPI_SET(0.0040, 0.0, 0.024);   /* v=2500 */
static struct kpi_params kpi_medium[] = KPI_SET(0.003333, 0.0, 0.020); /* v=3000 */
static struct kpi_params kpi_fast[] = KPI_SET(0.00367, 0.0, 0.01971); /* v=3500 */
static struct kpi_params kpi_super[] = KPI_SET(0.002667, 0.0, 0.0160); /* v=3750 */
static struct kpi_params kpi_haki[] = KPI_SET(0.0025, 0.0, 0.0150);    /* v=4000 */

static struct kinematics kinematics_settings[] = {
    [SPEED_TEST] = {
        .linear_speed = 2000,
        .linear_accel = {
            .break_accel = 5000,
            .accel_hard = 5000,
            .speed_hard = 0,
            .accel_soft = 0,
        },
        .fan_speed = 15,
        .kpi = kpi_test,
        .mpu = {
            .full_scale_dps = MPU_FULL_SCALE_2000DPS,
            .low_pass_filter_alpha = 0.27f,
        },
    },
    [SPEED_NORMAL] = {
        .linear_speed = 2500,
        .linear_accel = {
            .break_accel = 12000,
            .accel_hard = 12000,
            .speed_hard = 0,
            .accel_soft = 0,
        },
        .fan_speed = 20,
        .kpi = kpi_normal,
        .mpu = {
            .full_scale_dps = MPU_FULL_SCALE_2000DPS,
            .low_pass_filter_alpha = 0.27f,
        },
    },
    [SPEED_MEDIUM] = {
        .linear_speed = 3000,
        .linear_accel = {
            .break_accel = 15000,
            .accel_hard = 15000,
            .speed_hard = 0,
            .accel_soft = 0,
        },
        .fan_speed = 35,
        .kpi = kpi_medium,
        .mpu = {
            .full_scale_dps = MPU_FULL_SCALE_2000DPS,
            .low_pass_filter_alpha = 0.27f,
        },
    },
    [SPEED_FAST] = {
        .linear_speed = 3500,
        .linear_accel = {
            .break_accel = 20000,
            .accel_hard = 20000,
            .speed_hard = 3500,
            .accel_soft = 15000,
        },
        .fan_speed = 45,
        .kpi = kpi_fast,
        .mpu = {
            .full_scale_dps = MPU_FULL_SCALE_2000DPS,
            .low_pass_filter_alpha = 0.27f,
        },
    },
    [SPEED_SUPER] = {
        .linear_speed = 3750,
        .linear_accel = {
            .break_accel = 25000,
            .accel_hard = 25000,
            .speed_hard = 4000,
            .accel_soft = 20000,
        },
        .fan_speed = 50,
        .kpi = kpi_super,
        .mpu = {
            .full_scale_dps = MPU_FULL_SCALE_2000DPS,
            .low_pass_filter_alpha = 0.27f,
        },
    },
    [SPEED_HAKI] = {
        .linear_speed = 4000,
        .linear_accel = {
            .break_accel = 30000,
            .accel_hard = 30000,
            .speed_hard = 4500,
            .accel_soft = 25000,
        },
        .fan_speed = 60,
        .kpi = kpi_haki,
        .mpu = {
            .full_scale_dps = MPU_FULL_SCALE_2000DPS,
            .low_pass_filter_alpha = 0.27f,
        },
    }};

static struct kinematics kinematics;

void setup_kinematics(void) {
  kinematics = kinematics_settings[SPEED_TEST];
}

void configure_kinematics(enum speed_strategy speed) {
  kinematics = kinematics_settings[speed];
  lsm6dsr_reload_config();
}

struct kinematics get_kinematics(void) {
  return kinematics;
}