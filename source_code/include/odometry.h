#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include <stdbool.h>

/*
 * Odometría de DragonBot2.
 *
 * Convención (única en todo el módulo):
 *   +x    = avance del robot
 *   +y    = izquierda del robot
 *   theta = rumbo en radianes, positivo = giro a la izquierda (CCW),
 *           theta = +integral(omega_z dt)
 *
 * Fuentes:
 *   - Encoders de cuadratura (deltas de ticks crudos)  -> desplazamiento.
 *   - Giroscopio LSM6DSR (rata Z, CCW-positiva)        -> rumbo.
 *
 * Este módulo es la única fuente de verdad de posición y rumbo. La velocidad
 * lineal (mm/s) procede de los encoders; la angular (rad/s) del giroscopio,
 * con la velocidad angular diferencial de los encoders expuesta aparte.
 */

/** @brief Inicializa el estado de la odometría (equivale a un reset). */
void odometry_init(void);

/** @brief Pone a cero posición, rumbo y estado interno. */
void odometry_reset(void);

/** @brief Actualiza la odometría. Debe llamarse a la frecuencia de control (1 kHz). */
void odometry_update(void);

/** @brief Posición X en milímetros (avance). */
float odometry_get_x(void);

/** @brief Posición Y en milímetros (izquierda). */
float odometry_get_y(void);

/** @brief Rumbo en radianes, CCW-positivo. */
float odometry_get_theta_rad(void);

/** @brief Rumbo en grados, CCW-positivo. */
float odometry_get_theta_deg(void);

/** @brief Velocidad lineal en mm/s (media de encoders). */
float odometry_get_linear_speed(void);

/** @brief Velocidad angular en rad/s, CCW-positiva (giroscopio). */
float odometry_get_angular_speed(void);

/** @brief Velocidad angular en rad/s, CCW-positiva, derivada de los encoders. */
float odometry_get_encoder_omega(void);

/** @brief Velocidad lineal cruda de los encoders (medida), en mm/s. */
float odometry_get_encoder_v(void);

/** @brief Estimación online del bias del giroscopio en rad/s (Fase 2). */
float odometry_get_gyro_bias(void);

/** @brief Indicador de derrape detectado (Fase 2). */
bool odometry_get_slip(void);

/** @brief Último intervalo de actualización medido, en segundos. */
float odometry_get_dt(void);

/** @brief Varianza estimada de la posición X (mm^2). */
float odometry_get_var_x(void);
/** @brief Varianza estimada de la posición Y (mm^2). */
float odometry_get_var_y(void);
/** @brief Varianza estimada del rumbo (rad^2). */
float odometry_get_var_theta(void);

#endif /* __ODOMETRY_H */
