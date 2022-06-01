/*
 * demanded_torque_calculator.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "demanded_torque_calculator".
 *
 * Model version              : 4.2
 * Simulink Coder version : 9.4 (R2020b) 29-Jul-2020
 * C source code generated on : Wed Mar 10 22:14:50 2021
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#ifndef RTW_HEADER_demanded_torque_calculator_h_
#define RTW_HEADER_demanded_torque_calculator_h_
#include <string.h>
#include <stddef.h>
#ifndef demanded_torque_calculator_COMMON_INCLUDES_
#define demanded_torque_calculator_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                         /* demanded_torque_calculator_COMMON_INCLUDES_ */

#include "demanded_torque_calculator_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T regen_percentage;             /* '<Root>/regen_percentage' */
  real_T throttle;                     /* '<Root>/throttle' */
  real_T max_torque;                   /* '<Root>/max_torque' */
} ExtU_demanded_torque_calculat_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T demanded_torque;              /* '<Root>/demanded_torque' */
} ExtY_demanded_torque_calculat_T;

/* Real-time Model Data Structure */
struct tag_RTM_demanded_torque_calcu_T {
  const char_T *errorStatus;
};

/* External inputs (root inport signals with default storage) */
extern ExtU_demanded_torque_calculat_T demanded_torque_calculator_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_demanded_torque_calculat_T demanded_torque_calculator_Y;

/* Model entry point functions */
extern void demanded_torque_calculator_initialize(void);
extern void demanded_torque_calculator_step(void);
extern void demanded_torque_calculator_terminate(void);

/* Real-time Model object */
extern RT_MODEL_demanded_torque_calc_T *const demanded_torque_calculator_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S2>/Data Type Duplicate' : Unused code path elimination
 * Block '<S2>/Data Type Propagation' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'demanded_torque_calculator'
 * '<S1>'   : 'demanded_torque_calculator/demanded_torque_calculator'
 * '<S2>'   : 'demanded_torque_calculator/demanded_torque_calculator/Saturation Dynamic'
 * '<S3>'   : 'demanded_torque_calculator/demanded_torque_calculator/forward_torque_throttle_map'
 */
#endif                            /* RTW_HEADER_demanded_torque_calculator_h_ */
