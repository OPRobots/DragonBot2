#include "lsm6dsr.h"

#define MPU_READ 0x80
#define MPU_WHOAMI 0x0F

#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define CTRL6_C 0x15
#define CTRL7_G 0x16
#define CTRL8_XL 0x17
#define CTRL9_XL 0x18

#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27

#define STATUS_REG 0x1E
#define STATUS_GDA 0x02

#define GYRO_SENSITIVITY_2000DPS 70.0f
#define GYRO_SENSITIVITY_4000DPS 140.0f
#define MPU_DPS_TO_RADPS (PI / 180)

static stmdev_ctx_t dev_ctx;

static uint8_t full_scale_dps[] = {MPU_FULL_SCALE_1000DPS, MPU_FULL_SCALE_2000DPS, MPU_FULL_SCALE_4000DPS};
static uint8_t sensitivity_dps[] = {MPU_SENSITIVITY_1000DPS, MPU_SENSITIVITY_2000DPS, MPU_SENSITIVITY_4000DPS};

static uint8_t current_full_scale_dps;

static volatile float gyro_z_raw;
static volatile bool gyro_sample_ready;
static uint8_t gyro_status_miss;
static bool mpu_updating = false;
static float offset_z[MPU_FULL_SCALE_COUNT];

static uint8_t lsm6dsr_read_register(uint8_t address) {
  gpio_clear(GPIOA, GPIO15);
  spi_xfer(SPI3, (MPU_READ | address));
  uint8_t reading = spi_xfer(SPI3, 0x00);
  gpio_set(GPIOA, GPIO15);
  return reading;
}

static void lsm6dsr_write_register(uint8_t address, uint8_t value) {
  gpio_clear(GPIOA, GPIO15);
  spi_xfer(SPI3, address);
  spi_xfer(SPI3, value);
  gpio_set(GPIOA, GPIO15);
}

/* #region Librería */

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  handle = handle;
  len = len;
  lsm6dsr_write_register(reg, *bufp);
  return 0;
}
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  handle = handle;
  len = len;
  *bufp = lsm6dsr_read_register(reg);
  return 0;
}
static void platform_delay(uint32_t ms) {
  delay(ms);
}

/* #endregion */

static int16_t lsm6dsr_read_axis16(uint8_t reg_l) {
  gpio_clear(GPIOA, GPIO15);
  spi_xfer(SPI3, (MPU_READ | reg_l));
  uint8_t low = spi_xfer(SPI3, 0x00);
  uint8_t high = spi_xfer(SPI3, 0x00);
  gpio_set(GPIOA, GPIO15);
  return (int16_t)((high << 8) | low);
}

static int16_t lsm6dsr_read_gyro_z_raw(void) {
  return lsm6dsr_read_axis16(OUTZ_L_G);
}

static int get_sensitivity_dps(void) {
  switch (current_full_scale_dps) {
    case MPU_FULL_SCALE_1000DPS:
      return sensitivity_dps[0];
    case MPU_FULL_SCALE_2000DPS:
      return sensitivity_dps[1];
    case MPU_FULL_SCALE_4000DPS:
      return sensitivity_dps[2];
  }
  return 0;
}

void lsm6dsr_init(void) {

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;

  /* Restore default configuration */
  lsm6dsr_reset_set(&dev_ctx, PROPERTY_ENABLE);
  uint8_t rst;
  do {
    lsm6dsr_reset_get(&dev_ctx, &rst);
  } while (rst);

  // /* Disable I3C interface */
  // lsm6dsr_i3c_disable_set(&dev_ctx, LSM6DSR_I3C_DISABLE);
  // /* Enable Block Data Update */
  // lsm6dsr_block_data_update_set(&dev_ctx, PROPERTY_DISABLE);

  // /* Set Output Data Rate */
  // lsm6dsr_xl_data_rate_set(&dev_ctx, LSM6DSR_XL_ODR_12Hz5);
  // lsm6dsr_gy_data_rate_set(&dev_ctx, LSM6DSR_GY_ODR_1666Hz);
  // /* Set full scale */
  // lsm6dsr_xl_full_scale_set(&dev_ctx, LSM6DSR_2g);
  // lsm6dsr_gy_full_scale_set(&dev_ctx, LSM6DSR_4000dps);
  // /* Configure filtering chain(No aux interface)
  //  * Accelerometer - LPF1 + LPF2 path
  //  */
  // lsm6dsr_xl_hp_path_on_out_set(&dev_ctx, LSM6DSR_LP_ODR_DIV_100);
  // lsm6dsr_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

  /* Disable I3C */
  lsm6dsr_i3c_disable_set(&dev_ctx, LSM6DSR_I3C_DISABLE);

  /* IMPORTANT */
  lsm6dsr_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Accelerometer (helper clock only) */
  lsm6dsr_xl_data_rate_set(&dev_ctx, LSM6DSR_XL_ODR_104Hz);
  lsm6dsr_xl_full_scale_set(&dev_ctx, LSM6DSR_2g);

  /* Gyro */
  lsm6dsr_gy_power_mode_set(&dev_ctx, LSM6DSR_GY_HIGH_PERFORMANCE);
  lsm6dsr_gy_data_rate_set(&dev_ctx, LSM6DSR_GY_ODR_833Hz);
  lsm6dsr_gy_full_scale_set(&dev_ctx, get_kinematics().mpu.full_scale_dps);
  current_full_scale_dps = get_kinematics().mpu.full_scale_dps;

  /*
   * Base de medida CRUDA para el EKF (F3.0, D8): sin LPF1 hardware ni filtro
   * software. A ODR 833 Hz el contenido (<416 Hz) queda por debajo de Nyquist
   * de la lectura a 1 kHz, así que no hay aliasing al decimar. El filtrado lo
   * hará el estimador (α-β / EKF).
   */
  lsm6dsr_gy_filter_lp1_set(&dev_ctx, PROPERTY_DISABLE);

  delay(1000);
}

void lsm6dsr_reload_config(void) {
  if (current_full_scale_dps != get_kinematics().mpu.full_scale_dps) {
    bool was_paused = sram_log_is_paused();
    sram_log_set_paused(true);
    lsm6dsr_gy_full_scale_set(&dev_ctx, get_kinematics().mpu.full_scale_dps);
    current_full_scale_dps = get_kinematics().mpu.full_scale_dps;
    delay(50);
    reset_control_all();
    sram_log_set_paused(was_paused);
  }
}

uint8_t lsm6dsr_who_am_i(void) {
  return lsm6dsr_read_register(MPU_WHOAMI);
}

void lsm6dsr_gyro_z_calibration(void) {
  bool was_paused = sram_log_is_paused();
  sram_log_set_paused(true);

  mpu_updating = false;
  int16_t eeprom_data[MPU_FULL_SCALE_COUNT];

  for (uint8_t i = 0; i < MPU_FULL_SCALE_COUNT; i++) {
    lsm6dsr_gy_full_scale_set(&dev_ctx, full_scale_dps[i]);
    delay(50);
    int32_t sum_z = 0;
    offset_z[i] = 0;
    for (int s = 0; s < 200; s++) {
      sum_z += lsm6dsr_read_gyro_z_raw();
      delay(5);
      set_info_leds();
    }
    clear_info_leds();

    offset_z[i] = sum_z / 200.0f;
    printf("Offset Z for full-scale %d dps: %.4f\n", full_scale_dps[i], offset_z[i]);
    eeprom_data[i] = offset_z[i] * 1000;
  }

  eeprom_set_data(DATA_INDEX_GYRO_Z, eeprom_data, MPU_DATA_LENGTH);

  lsm6dsr_gy_full_scale_set(&dev_ctx, current_full_scale_dps);
  delay(100);
  mpu_updating = true;

  sram_log_set_paused(was_paused);
}

void lsm6dsr_load_eeprom(void) {
  int16_t *eeprom_data = eeprom_get_data();
  for (uint8_t i = 0; i < MPU_FULL_SCALE_COUNT; i++) {
    offset_z[i] = eeprom_data[DATA_INDEX_GYRO_Z + i] / 1000.0f;
    printf("Offset Z for full-scale %d dps: %.4f\n", full_scale_dps[i], offset_z[i]);
  }
  mpu_updating = true;
}

void lsm6dsr_update(void) {
  if (!mpu_updating) {
    return;
  }

  uint8_t status = lsm6dsr_read_register(STATUS_REG);
  bool gyro_new = (status & STATUS_GDA) != 0;

  /*
   * Si el flag de data-ready no llega (fallo de configuración o de lectura),
   * se fuerza una lectura periódica para no dejar el rumbo congelado.
   */
  if (gyro_new) {
    gyro_status_miss = 0;
  } else if (gyro_status_miss < 0xFF) {
    gyro_status_miss++;
  }

  if (gyro_new || gyro_status_miss >= 5) {
    gyro_status_miss = 0;
    /* Medida cruda (offset calibrado restado), sin filtrado software (F3.0). */
    gyro_z_raw = (float)lsm6dsr_read_gyro_z_raw() - get_offset_z();
    gyro_sample_ready = true;
  }
}

float lsm6dsr_get_gyro_z_raw(void) {
  return gyro_z_raw;
}

float lsm6dsr_get_gyro_z_radps(void) {
  return (gyro_z_raw * get_sensitivity_dps() / 1000 * MPU_DPS_TO_RADPS);
}

float lsm6dsr_get_gyro_z_dps(void) {
  return (gyro_z_raw * get_sensitivity_dps()) / 1000;
}

/**
 * @brief Consume el aviso de muestra nueva de gyro.
 *
 * Devuelve true una sola vez por cada muestra leída del sensor. Permite al
 * estimador no reutilizar la misma medida en varias actualizaciones.
 */
bool lsm6dsr_gyro_sample_ready(void) {
  if (gyro_sample_ready) {
    gyro_sample_ready = false;
    return true;
  }
  return false;
}

float get_offset_z(void) {
  switch (current_full_scale_dps) {
    case MPU_FULL_SCALE_1000DPS:
      return offset_z[0];
    case MPU_FULL_SCALE_2000DPS:
      return offset_z[1];
    case MPU_FULL_SCALE_4000DPS:
      return offset_z[2];
  }
  return 0;
}

uint8_t get_current_full_scale_dps(void) {
  return current_full_scale_dps;
}