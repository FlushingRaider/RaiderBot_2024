/*
  control_pid.hpp

  Created on: Feb 25, 2019

  Author: Biggs

  Moved to 2024 code: January 17, 2024
*/

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
                   double  LeSD_Deg_OutputLowerLimit);