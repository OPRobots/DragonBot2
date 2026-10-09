#include <kinematics.h>

static struct kpi_params kpi_test[] = {
    [KPI_LINEAR] = {
        .kp = 0.003,
        .ki = 0.0002,
        .kd = 0.00,
    },
    [KPI_ANGULAR] = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0,
    },
    [KPI_LINE_SENSORS] = {
        .kp = 0.0025,
        .ki = 0.0,
        .kd = 0.03,
    },
};

static struct kpi_params kpi_run[] = {
    [KPI_LINEAR] = {
        .kp = 0.003,
        .ki = 0.0002,
        .kd = 0.00,
    },
    [KPI_ANGULAR] = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0,
    },
    [KPI_LINE_SENSORS] = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0,
    },
};

static struct kinematics kinematics_settings[] = {
    [SPEED_TEST] = {
        .linear_speed = 800,
        .linear_accel = {
            .break_accel = 5000,
            .accel_hard = 5000,
            .speed_hard = 0,
            .accel_soft = 0,
        },
        .fan_speed = 30,
        .kpi = kpi_test,
        .mpu = {
            .full_scale_dps = MPU_FULL_SCALE_2000DPS,
            .low_pass_filter_alpha = 0.27f,
        },
    },
    [SPEED_NORMAL] = {
        .linear_speed = 3000,
        .linear_accel = {
            .break_accel = 12000,
            .accel_hard = 12000,
            .speed_hard = 0,
            .accel_soft = 0,
        },
        .fan_speed = 60,
        .kpi = kpi_run,
        .mpu = {
            .full_scale_dps = MPU_FULL_SCALE_2000DPS,
            .low_pass_filter_alpha = 0.27f,
        },
    },
    [SPEED_MEDIUM] = {
        .linear_speed = 5000,
        .linear_accel = {
            .break_accel = 15000,
            .accel_hard = 15000,
            .speed_hard = 0,
            .accel_soft = 0,
        },
        .fan_speed = 65,
        .kpi = kpi_run,
        .mpu = {
            .full_scale_dps = MPU_FULL_SCALE_2000DPS,
            .low_pass_filter_alpha = 0.27f,
        },
    },
    [SPEED_FAST] = {
        .linear_speed = 5500,
        .linear_accel = {
            .break_accel = 20000,
            .accel_hard = 20000,
            .speed_hard = 3500,
            .accel_soft = 15000,
        },
        .fan_speed = 75,
        .kpi = kpi_run,
        .mpu = {
            .full_scale_dps = MPU_FULL_SCALE_2000DPS,
            .low_pass_filter_alpha = 0.27f,
        },
    },
    [SPEED_SUPER] = {
        .linear_speed = 6000,
        .linear_accel = {
            .break_accel = 25000,
            .accel_hard = 25000,
            .speed_hard = 4000,
            .accel_soft = 20000,
        },
        .fan_speed = 85,
        .kpi = kpi_run,
        .mpu = {
            .full_scale_dps = MPU_FULL_SCALE_2000DPS,
            .low_pass_filter_alpha = 0.27f,
        },
    },
    [SPEED_HAKI] = {
        .linear_speed = 6500,
        .linear_accel = {
            .break_accel = 30000,
            .accel_hard = 30000,
            .speed_hard = 4500,
            .accel_soft = 25000,
        },
        .fan_speed = 90,
        .kpi = kpi_run,
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