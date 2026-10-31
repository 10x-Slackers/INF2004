/* Initial calibration targets; validate on the actual chassis. */
#ifndef S5_LIMITS_H
#define S5_LIMITS_H
#define S5_PRI_AVOID             20
#define S5_PRI_SCAN              21
#define S5_TRIG_PIN              0
#define S5_ECHO_PIN              1
#define S5_SERVO_PIN             12
#define S5_MIN_MM                20
#define S5_MAX_MM                4000
#define S5_ECHO_TIMEOUT_US       25000U
#define S5_MEASURE_INTERVAL_US   60000U
#define S5_FORWARD_TOLERANCE_DEG 5
#define S5_FORWARD_MAX_AGE_MS    200U
#define S5_PROFILE_MAX_AGE_MS    6000U
#define S5_COARSE_COUNT          5
#define S5_COARSE_ANGLES         {30, 60, 90, 120, 150}
#define S5_FINE_STEP_DEG         5
#define S5_FINE_HALF_SPAN_DEG    15
#define S5_FINE_COUNT            7
#define S5_MAX_SAMPLES           (S5_COARSE_COUNT + S5_FINE_COUNT)
#define S5_FILTER_COUNT          3
#define S5_SERVO_MIN_DEG         30
#define S5_SERVO_CENTRE_DEG      90
#define S5_SERVO_MAX_DEG         150
#define S5_SERVO_MIN_US          1100
#define S5_SERVO_CENTRE_US       1500
#define S5_SERVO_MAX_US          1900
#define S5_SERVO_SETTLE_MS       200
#define S5_SCAN_MAX_MS           8000U
#define S5_WAIT_STOP_MS          2000U
#define S5_MOTION_MAX_MS         10000U
#define S5_RECOVERY_MAX_MS       3000U
#define S5_MAX_SCAN_RETRIES      2
#define S5_MAX_RECOVERY_RETRIES  2
#define S5_OBSTACLE_MM           350
#define S5_EMERGENCY_MM          100
#define S5_BYPASS_CLEARANCE_MM   500
/* No chassis dimensions are documented. Gate autonomous recommendations. */
#define S5_BYPASS_CALIBRATED 0
#endif
