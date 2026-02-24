#include <kinematics.h>

static struct kinematics kinematics_settings[] = {
    [SPEED_TEST] = {
        .linear_speed = 650,
        .linear_accel = {
            .break_accel = 5000,
            .accel_hard = 5000,
            .speed_hard = 0,
            .accel_soft = 0,
        },
        .fan_speed = 30,
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
    }};

static struct kinematics kinematics;

void configure_kinematics(enum speed_strategy speed) {
  kinematics = kinematics_settings[speed];
}

struct kinematics get_kinematics(void) {
  return kinematics;
}