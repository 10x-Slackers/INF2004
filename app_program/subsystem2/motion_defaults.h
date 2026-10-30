/* Shared boot calibration; review the setup and confirm explicitly each boot.
 * Console edits are RAM-only.
 */
#ifndef MOTION_DEFAULTS_H
#define MOTION_DEFAULTS_H

#ifndef MOTION_DEFAULT_CIRCUMFERENCE_MM
#define MOTION_DEFAULT_CIRCUMFERENCE_MM 204u
#endif
#ifndef MOTION_DEFAULT_TRACK_MM
#define MOTION_DEFAULT_TRACK_MM 113u
#endif
#ifndef MOTION_DEFAULT_COUNTS_PER_REV
#define MOTION_DEFAULT_COUNTS_PER_REV 0u
#endif
/* Per-wheel zero selects shared CPR; counts include all quadrature edges. */
#ifndef MOTION_DEFAULT_LEFT_COUNTS_PER_REV
#define MOTION_DEFAULT_LEFT_COUNTS_PER_REV 2466u
#endif
#ifndef MOTION_DEFAULT_RIGHT_COUNTS_PER_REV
#define MOTION_DEFAULT_RIGHT_COUNTS_PER_REV 2524u
#endif
#ifndef MOTION_DEFAULT_KP_Q8
#define MOTION_DEFAULT_KP_Q8 256u
#endif
#ifndef MOTION_DEFAULT_KI_Q8
#define MOTION_DEFAULT_KI_Q8 1u
#endif
#ifndef MOTION_DEFAULT_KD_Q8
#define MOTION_DEFAULT_KD_Q8 0u
#endif
#ifndef MOTION_DEFAULT_SPEED_MM_S
#define MOTION_DEFAULT_SPEED_MM_S 100u
#endif
#ifndef MOTION_DEFAULT_DUTY_LIMIT
#define MOTION_DEFAULT_DUTY_LIMIT 100u
#endif
#ifndef MOTION_DEFAULT_TIMEOUT_MS
#define MOTION_DEFAULT_TIMEOUT_MS 1500u
#endif
#ifndef MOTION_DEFAULT_STALL_MS
#define MOTION_DEFAULT_STALL_MS 500u
#endif
#ifndef MOTION_DEFAULT_MOTOR_LEFT_REVERSE
#define MOTION_DEFAULT_MOTOR_LEFT_REVERSE 1u
#endif
#ifndef MOTION_DEFAULT_MOTOR_RIGHT_REVERSE
#define MOTION_DEFAULT_MOTOR_RIGHT_REVERSE 0u
#endif
#ifndef MOTION_DEFAULT_ENCODER_LEFT_REVERSE
#define MOTION_DEFAULT_ENCODER_LEFT_REVERSE 1u
#endif
#ifndef MOTION_DEFAULT_ENCODER_RIGHT_REVERSE
#define MOTION_DEFAULT_ENCODER_RIGHT_REVERSE 0u
#endif
#ifndef MOTION_DEFAULT_CONFIRMED
#define MOTION_DEFAULT_CONFIRMED 0u
#endif

/* Reject values before assignment can truncate an unsigned bit-field. */
#if (MOTION_DEFAULT_CIRCUMFERENCE_MM < 0) || \
    (MOTION_DEFAULT_CIRCUMFERENCE_MM > 1000u)
#error Invalid motion default CIRCUMFERENCE_MM
#endif
#if (MOTION_DEFAULT_TRACK_MM < 0) || (MOTION_DEFAULT_TRACK_MM > 1000u)
#error Invalid motion default TRACK_MM
#endif
#if (MOTION_DEFAULT_COUNTS_PER_REV < 0) || \
    (MOTION_DEFAULT_COUNTS_PER_REV > 65535u)
#error Invalid motion default COUNTS_PER_REV
#endif
#if (MOTION_DEFAULT_LEFT_COUNTS_PER_REV < 0) || \
    (MOTION_DEFAULT_LEFT_COUNTS_PER_REV > 65535u)
#error Invalid motion default LEFT_COUNTS_PER_REV
#endif
#if (MOTION_DEFAULT_RIGHT_COUNTS_PER_REV < 0) || \
    (MOTION_DEFAULT_RIGHT_COUNTS_PER_REV > 65535u)
#error Invalid motion default RIGHT_COUNTS_PER_REV
#endif
#if (MOTION_DEFAULT_KP_Q8 < 0) || (MOTION_DEFAULT_KP_Q8 > 4095u)
#error Invalid motion default KP_Q8
#endif
#if (MOTION_DEFAULT_KI_Q8 < 0) || (MOTION_DEFAULT_KI_Q8 > 4095u)
#error Invalid motion default KI_Q8
#endif
#if (MOTION_DEFAULT_KD_Q8 < 0) || (MOTION_DEFAULT_KD_Q8 > 4095u)
#error Invalid motion default KD_Q8
#endif
#if (MOTION_DEFAULT_SPEED_MM_S < 0) || (MOTION_DEFAULT_SPEED_MM_S > 1000u)
#error Invalid motion default SPEED_MM_S
#endif
#if (MOTION_DEFAULT_DUTY_LIMIT < 0) || (MOTION_DEFAULT_DUTY_LIMIT > 1000u)
#error Invalid motion default DUTY_LIMIT
#endif
#if (MOTION_DEFAULT_TIMEOUT_MS < 0) || (MOTION_DEFAULT_TIMEOUT_MS > 60000u)
#error Invalid motion default TIMEOUT_MS
#endif
#if (MOTION_DEFAULT_STALL_MS < 0) || (MOTION_DEFAULT_STALL_MS > 10000u)
#error Invalid motion default STALL_MS
#endif
#if (MOTION_DEFAULT_MOTOR_LEFT_REVERSE < 0) || \
    (MOTION_DEFAULT_MOTOR_LEFT_REVERSE > 1u)
#error Invalid motion default MOTOR_LEFT_REVERSE
#endif
#if (MOTION_DEFAULT_MOTOR_RIGHT_REVERSE < 0) || \
    (MOTION_DEFAULT_MOTOR_RIGHT_REVERSE > 1u)
#error Invalid motion default MOTOR_RIGHT_REVERSE
#endif
#if (MOTION_DEFAULT_ENCODER_LEFT_REVERSE < 0) || \
    (MOTION_DEFAULT_ENCODER_LEFT_REVERSE > 1u)
#error Invalid motion default ENCODER_LEFT_REVERSE
#endif
#if (MOTION_DEFAULT_ENCODER_RIGHT_REVERSE < 0) || \
    (MOTION_DEFAULT_ENCODER_RIGHT_REVERSE > 1u)
#error Invalid motion default ENCODER_RIGHT_REVERSE
#endif
#if (MOTION_DEFAULT_CONFIRMED < 0) || (MOTION_DEFAULT_CONFIRMED > 1u)
#error Invalid motion default CONFIRMED
#endif
#if (MOTION_DEFAULT_SPEED_MM_S < 10u) || (MOTION_DEFAULT_DUTY_LIMIT < 1u) || \
    (MOTION_DEFAULT_TIMEOUT_MS < 100u) || (MOTION_DEFAULT_STALL_MS < 100u)
#error Invalid motion operating defaults
#endif
#if MOTION_DEFAULT_CONFIRMED && \
    ((MOTION_DEFAULT_KP_Q8 < 1u) || \
     (MOTION_DEFAULT_CIRCUMFERENCE_MM < 50u) || \
     (MOTION_DEFAULT_TRACK_MM < 50u) || \
     ((MOTION_DEFAULT_LEFT_COUNTS_PER_REV == 0u) ? \
      (MOTION_DEFAULT_COUNTS_PER_REV < 4u) : \
      (MOTION_DEFAULT_LEFT_COUNTS_PER_REV < 4u)) || \
     ((MOTION_DEFAULT_RIGHT_COUNTS_PER_REV == 0u) ? \
      (MOTION_DEFAULT_COUNTS_PER_REV < 4u) : \
      (MOTION_DEFAULT_RIGHT_COUNTS_PER_REV < 4u)))
#error Confirmed motion defaults require measured valid geometry and CPR
#endif

#endif /* MOTION_DEFAULTS_H */
/* End of file. */
