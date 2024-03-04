/*
  ADAS_DM.hpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS Drive Management

  Changes:
  2022-02-25 -> Beta
 */

extern double V_ADAS_DM_InitGyroAngle;
extern double VeADAS_t_DM_AutoMountDbTime;
extern double VeADAS_t_DM_StateTimer;

void ADAS_DM_ConfigsCal(void);
void ADAS_DM_ConfigsInit(void);
void ADAS_DM_Reset(void);

bool ADAS_DM_PathFollower(double *LeADAS_Pct_FwdRev,
                          double *LeADAS_Pct_Strafe,
                          double *LeADAS_Pct_Rotate,
                          double *LeADAS_Deg_DesiredPose,
                          bool *LeADAS_b_SD_RobotOriented,
                          double LeADAS_l_X_FieldPos,
                          double LeADAS_l_Y_FieldPos,
                          double LeADAS_Deg_GyroAngle,
                          T_ADAS_ActiveFeature LeADAS_e_ActiveFeature,
                          std::optional<frc::DriverStation::Alliance> LeLC_e_AllianceColor);

bool ADAS_DM_MoveWithGlobalCoords(double *LeADAS_Pct_FwdRev,
                          double *LeADAS_Pct_Strafe,
                          double *LeADAS_Pct_Rotate,
                          double L_CurrentOdomX,
                          double L_CurrentOdomY,
                          double L_CurrentYaw,
                          double L_RequestedCoordX,
                          double L_RequestedCoordY,
                          double L_RequestedYaw);
