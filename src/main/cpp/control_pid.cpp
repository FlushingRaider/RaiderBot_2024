/*
  control_pid.cpp

  Created on: Jan 01, 2020
  Author: 5561

  Moved to 2024 code January 17, 2024
 */

#include "control_pid.hpp"

/******************************************************************************
 * Function:     Control_PID
 *
 * Description:  Proportional Integral Derivative control.
 ******************************************************************************/
double Control_PID(double  L_DesiredSpeed,
                   double  L_CurrentSpeed,
                   double *LeLU_Cmd_ErrorPrev,
                   double *L_IntegralPrev,
                   double  L_ProportionalGx,
                   double  L_IntegralGx,
                   double  L_DerivativeGx,
                   double  L_ProportionalUpperLimit,
                   double  L_ProportionalLowerLimit,
                   double  L_IntegralUpperLimit,
                   double  L_IntegralLowerLimit,
                   double  L_DerivativeUpperLimit,
                   double  L_DerivativeLowerLimit,
                   double  LeSD_Deg_OutputUpperLimit,
                   double  LeSD_Deg_OutputLowerLimit)
  {
  double LeLU_Cmd_Error        = 0.0;
  double L_Proportional = 0.0;
  double L_Integral     = 0.0;
  double L_Derivative   = 0.0;
  double LeSD_Deg_OutputCmnd   = 0.0;

  LeLU_Cmd_Error = L_DesiredSpeed - L_CurrentSpeed;

  L_Proportional = LeLU_Cmd_Error * L_ProportionalGx;

  L_Integral = *L_IntegralPrev + (LeLU_Cmd_Error * L_IntegralGx);

  L_Derivative = L_DerivativeGx * (*LeLU_Cmd_ErrorPrev / 0.01);

  *LeLU_Cmd_ErrorPrev = LeLU_Cmd_Error;

  if (L_Proportional > L_ProportionalUpperLimit)
    {
    L_Proportional = L_ProportionalUpperLimit;
    }
  else if (L_Proportional < L_ProportionalLowerLimit)
    {
    L_Proportional = L_ProportionalLowerLimit;
    }

  if (L_Integral > L_IntegralUpperLimit)
    {
    L_Integral = L_IntegralUpperLimit;
    }
  else if (L_Integral < L_IntegralLowerLimit)
    {
    L_Integral = L_IntegralLowerLimit;
    }

  if (L_Derivative > L_DerivativeUpperLimit)
    {
    L_Derivative = L_DerivativeUpperLimit;
    }
  else if (L_Derivative < L_DerivativeLowerLimit)
    {
    L_Derivative = L_DerivativeLowerLimit;
    }

  /* Ok, lets record the integral to use next loop: */
  *L_IntegralPrev = L_Integral;

  /* Lets add all three controllers. */
  LeSD_Deg_OutputCmnd = L_Proportional + L_Integral + L_Derivative;

  /* This is kind of redundant, but lets limit the output to the min and max
   * allowed for the controller: */
  if (LeSD_Deg_OutputCmnd > LeSD_Deg_OutputUpperLimit)
    {
    LeSD_Deg_OutputCmnd = LeSD_Deg_OutputUpperLimit;
    }
  else if (LeSD_Deg_OutputCmnd < LeSD_Deg_OutputLowerLimit)
    {
    LeSD_Deg_OutputCmnd = LeSD_Deg_OutputLowerLimit;
    }

  return LeSD_Deg_OutputCmnd;
}