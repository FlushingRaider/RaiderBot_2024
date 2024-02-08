/*
  ADAS.hpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems)

  Changes:
  2022-02-25 -> Beta
 */

extern T_ADAS_ActiveFeature VeADAS_e_ActiveFeature;

extern double VeADAS_Pct_SD_FwdRev;
extern double VeADAS_Pct_SD_Strafe;
extern double VeADAS_Pct_SD_Rotate;
extern double VeADAS_Deg_SD_DesiredPose;

extern bool VeADAS_b_SD_RobotOriented;
extern bool VeADAS_b_X_Mode;

extern double VeADAS_in_OffsetRequestX;
extern double VeADAS_in_OffsetRequestY;

extern double VeADAS_in_GlobalRequestX;
extern double VeADAS_in_GlobalRequestY;

void ADAS_Main_Reset(void);
void ADAS_Main_Init(void);
void ADAS_DetermineMode(void);


T_ADAS_ActiveFeature ADAS_ControlMain(double *LeADAS_Pct_FwdRev,
                                      double *LeADAS_Pct_Strafe,
                                      double *LeADAS_Pct_Rotate,
                                      double *LeADAS_Deg_DesiredPose,
                                      bool *LeADAS_b_SD_RobotOriented,
                                      bool *LeADAS_b_X_Mode,
                                      bool LeADAS_b_Driver1_JoystickActive,
                                      bool L_Driver_SwerveGoalAutoCenter,
                                      double L_Deg_GyroAngleDeg,
                                      double LeADAS_l_X_FieldPos,
                                      double LeADAS_l_Y_FieldPos,
                                      T_RobotState LeADAS_e_RobotState,
                                      T_ADAS_ActiveFeature LeADAS_e_ActiveFeature,
                                      bool L_OdomCentered,
                                      std::optional<frc::DriverStation::Alliance> LeLC_e_AllianceColor,
                                      double L_OdomOffsetX,
                                      double L_OdomOffsetY,
                                      double L_OdomGlobalRequestX,
                                      double L_OdomGlobalRequestY,
                                      double L_OdomOffsetRequestX,
                                      double L_OdomOffsetRequestY); 