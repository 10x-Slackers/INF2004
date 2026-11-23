/** @file tkernel.h
 *
 * @brief Host stand-in for the µT-Kernel API: only the types, constants
 * and calls task_imu.c uses. The calls are defined in test_imu.c.
 */

#ifndef TKERNEL_H
#define TKERNEL_H

#include <stdint.h>

typedef int32_t ID;
typedef int32_t ER;
typedef int32_t INT;
typedef int32_t TMO;
typedef uint32_t UW;
typedef uint32_t RELTIM;
typedef int32_t PRI;
typedef uint8_t UB;
typedef int32_t BOOL;
typedef void (*FP)(void);

#define E_OK     0
#define E_TMOUT  (-50)
#define E_IO     (-57)
#define TMO_FEVR (-1)
#define TRUE     1
#define FALSE    0

ER tk_loc_mtx(ID mtxid, TMO tmout);
ER tk_unl_mtx(ID mtxid);
ER tk_slp_tsk(TMO tmout);
ID tk_get_tid(void);

#endif /* TKERNEL_H */
