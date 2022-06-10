/*
 * demanded_torque_calculator.c
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

#include "demanded_torque_calculator.h"
#include "demanded_torque_calculator_private.h"

/* External inputs (root inport signals with default storage) */
ExtU_demanded_torque_calculat_T demanded_torque_calculator_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_demanded_torque_calculat_T demanded_torque_calculator_Y;

/* Real-time model */
static RT_MODEL_demanded_torque_calc_T demanded_torque_calculator_M_;
RT_MODEL_demanded_torque_calc_T *const demanded_torque_calculator_M =
  &demanded_torque_calculator_M_;

/* Model step function */
void demanded_torque_calculator_step(void)
{
  real_T rtb_torque;

  /* MATLAB Function: '<S1>/forward_torque_throttle_map' incorporates:
   *  Inport: '<Root>/max_torque'
   *  Inport: '<Root>/regen_percentage'
   *  Inport: '<Root>/throttle'
   */
  if ((demanded_torque_calculator_U.throttle <= 5) &&
      (demanded_torque_calculator_U.throttle >= 0)) {
    rtb_torque = 0;
  } else if (demanded_torque_calculator_U.regen_percentage == 0) {
    if (demanded_torque_calculator_U.throttle - 5 < 0) {
      rtb_torque = 0;
    } else {
      rtb_torque = (demanded_torque_calculator_U.throttle - 5) / 95 *
        demanded_torque_calculator_U.max_torque;
    }
  } else if (demanded_torque_calculator_U.throttle - 10 < 0) {
    rtb_torque = 0;
  } else {
    rtb_torque = (demanded_torque_calculator_U.throttle - 10) / 90 *
      demanded_torque_calculator_U.max_torque;
  }

  if (rtb_torque < 0) {
    rtb_torque = 0;
  } else {
    if (rtb_torque > 240.0) {
      rtb_torque = 240.0;
    }
  }

  /* End of MATLAB Function: '<S1>/forward_torque_throttle_map' */

  /* Switch: '<S2>/Switch2' incorporates:
   *  Inport: '<Root>/max_torque'
   *  RelationalOperator: '<S2>/LowerRelop1'
   */
  if (rtb_torque > demanded_torque_calculator_U.max_torque) {
    /* Outport: '<Root>/demanded_torque' */
    demanded_torque_calculator_Y.demanded_torque =
      demanded_torque_calculator_U.max_torque;
  } else {
    /* Outport: '<Root>/demanded_torque' incorporates:
     *  Switch: '<S2>/Switch'
     */
    demanded_torque_calculator_Y.demanded_torque = rtb_torque;
  }

  /* End of Switch: '<S2>/Switch2' */
}

/* Model initialize function */
void demanded_torque_calculator_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(demanded_torque_calculator_M, (NULL));

  /* external inputs */
  (void)memset(&demanded_torque_calculator_U, 0, sizeof
               (ExtU_demanded_torque_calculat_T));

  /* external outputs */
  demanded_torque_calculator_Y.demanded_torque = 0.0;
}

/* Model terminate function */
void demanded_torque_calculator_terminate(void)
{
  /* (no terminate code required) */
}
