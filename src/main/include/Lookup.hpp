/*
  LookUp.hpp

   Created on: Feb 14, 2018
       Author: biggs
   
   Imported to 2024: January 18, 2024
 */
#ifndef SRC_ROBORIO2018_LOOKUP_HPP_
#define SRC_ROBORIO2018_LOOKUP_HPP_

#include <string>

extern double ScaleJoystickAxis(double LeLU_Cmd_JoystickAxis);

extern double ScaleJoystickRotateAxis(double LeLU_Cmd_JoystickAxis);

extern double RampTo(double  L_Final,
                     double  L_Current,
                     double  L_Slope);

extern double RampTo_2Ramp(double LeLU_Cmd_Final,
                           double LeLU_Cmd_Current,
                           double LeLU_Cmd_SlopeFast,
                           double LeLU_Cmd_SlopeSlow,
                           double LeLU_K_Db);

extern double DesiredRotateSpeed(double LeLU_Cmd_Error);

extern double ScaleAccelAxis(double LeLU_Cmd_JoystickAxis);

extern double DesiredAutoRotateSpeed(double LeLU_Cmd_Error);

bool DesiredAutonLocation2(double  LeLU_s_AutonTime,
                           T_ADAS_ActiveFeature LeADAS_e_ActiveFeature,
                           frc::DriverStation::Alliance LeLC_e_AllianceColor,
                           double *LeLU_Cmd_L_X_Location,
                           double *LeLU_Cmd_L_Y_Location,
                           double *LeLU_Cmd_Deg_Angle,
                           double *LeLU_Cmd_TimeRemaining);
#endif /* SRC_ROBORIO2018_LOOKUP_HPP_ */
