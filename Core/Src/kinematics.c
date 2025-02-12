#include "kinematics.h"
#include "motor_control.h"
#include "encoder.h"
#include "PID.h"
#include <math.h>


double setpoint_left_rpm  = 0.0;
double setpoint_right_rpm = 0.0;

double outputLeft  = 0.0;
double outputRight = 0.0;

double inputLeft   = 0.0;
double inputRight  = 0.0;

PID_TypeDef pidLeft, pidRight;


void Kinematics_Init(void) {
	  PID2(&pidLeft, &inputLeft, &outputLeft, &setpoint_left_rpm, 18.81, 626.0, 0.0, _PID_CD_DIRECT);
	  PID2(&pidRight, &inputRight, &outputRight, &setpoint_right_rpm, 21.18, 863.0, 0.0, _PID_CD_DIRECT);

	  PID_SetOutputLimits(&pidLeft, -PWM_MAX, PWM_MAX);
	  PID_SetOutputLimits(&pidRight, -PWM_MAX, PWM_MAX);

	  PID_SetSampleTime(&pidLeft, 10);
	  PID_SetSampleTime(&pidRight, 10);

	  PID_SetMode(&pidLeft, _PID_MODE_AUTOMATIC);
	  PID_SetMode(&pidRight, _PID_MODE_AUTOMATIC);
}

/**
 * @brief Converte velocidade linear para RPM.
 */
float LinearToRPM(float v) {
    return (v * 60.0) / (2 * M_PI * WHEEL_RADIUS);
}

/**
 * @brief Define as velocidades do robô com base em velocidades lineares (m/s).
 *        Chama `Set_Motor_Speeds()` do `motor_control.c` para aplicar nos motores.
 */
void Kinematics_SetSpeeds(float vL, float vR) {
	float target_rpm_left = LinearToRPM(vL);
    float target_rpm_right = LinearToRPM(vR);

    setpoint_left_rpm  = target_rpm_left;
    setpoint_right_rpm = target_rpm_right;

    inputLeft  = left_encoder.rpm;
    inputRight = right_encoder.rpm;


    PID_Compute(&pidLeft);
    PID_Compute(&pidRight);

    if (outputLeft > PWM_MAX) {
        outputLeft = PWM_MAX;
    } else if (outputLeft < -PWM_MAX) {
        outputLeft = -PWM_MAX;
    }

    if (outputRight > PWM_MAX) {
        outputRight = PWM_MAX;
    } else if (outputRight < -PWM_MAX) {
        outputRight = -PWM_MAX;
    }


    if (fabs(outputLeft) < OUTPUT_TOLERANCE) outputLeft = 0;
    if (fabs(outputRight) < OUTPUT_TOLERANCE) outputRight = 0;


    float pwm_left  = fabs(outputLeft);
    float pwm_right = fabs(outputRight);

    pwm_left  = fmax(pwm_left, PWM_MIN);
    pwm_left  = fmin(pwm_left, PWM_MAX);
    pwm_right = fmax(pwm_right, PWM_MIN);
    pwm_right = fmin(pwm_right, PWM_MAX);


    uint8_t dir_left  = (outputLeft >= 0) ? 0 : 1;
    uint8_t dir_right = (outputRight >= 0) ? 0 : 1;

	Encoder_Update();
    Motor_Control(pwm_left, dir_left, pwm_right, dir_right);
}
