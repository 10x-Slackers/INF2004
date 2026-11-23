/** @file tkernel.h
 *
 * @brief Host stand-in for the µT-Kernel API: only the types, constants
 * and calls line_following.c and barcode_decoder.c use. The calls are
 * defined in test_faults.c.
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
typedef void (*FP)(void);

typedef struct
{
    uint32_t mtxatr;
} T_CMTX;

typedef struct
{
    int32_t hi;
    uint32_t lo;
} SYSTIM;

#define E_OK       0
#define TMO_FEVR   (-1)
#define TA_INHERIT 2u

ID tk_cre_mtx(const T_CMTX * p_mutex);
ER tk_loc_mtx(ID mtxid, TMO tmout);
ER tk_unl_mtx(ID mtxid);
ER tk_slp_tsk(TMO tmout);
ER tk_get_otm(SYSTIM * p_time);

#endif /* TKERNEL_H */
