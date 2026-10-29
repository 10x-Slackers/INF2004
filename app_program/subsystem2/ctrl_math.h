/* Small deterministic motion primitives, shared by firmware and host tests. */
#ifndef CTRL_MATH_H
#define CTRL_MATH_H

#include <stdint.h>

typedef struct
{
    unsigned int integral : 17; /* Signed value + 65535. */
    unsigned int previous_error : 13; /* Signed value + 4096. */
} ctrl_math_pid_t;

int32_t ctrl_math_clamp(int64_t value, int32_t limit);
/* elapsed_us > 0; caller bounds targets to +/-1250 and speed to +/-2000. */
int32_t ctrl_math_pid(ctrl_math_pid_t * p_pid, int32_t target,
                        int32_t measured, uint32_t gain_p, uint32_t gain_i,
                        uint32_t gain_d, uint32_t limit,
                        uint32_t elapsed_us);
void ctrl_math_pid_reset(ctrl_math_pid_t * p_pid);

#endif /* CTRL_MATH_H */
/* End of file. */
