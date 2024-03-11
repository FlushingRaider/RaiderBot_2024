/*
  DriveControl.hpp

   Created on: Feb 25, 2020

   Author: 5561

   Updates:
   2022-02-15: Cleaned up file

   Coverted to 2024 code: January 17, 2024
 */

extern double VaDRC_Pct_WheelAngleCmnd[E_RobotCornerSz];
extern double VaDRC_RPM_WheelSpeedCmnd[E_RobotCornerSz];
extern bool   VeDRC_b_DriveWheelsInPID;

void SwerveDriveMotorConfigsInit(rev::SparkMaxPIDController m_frontLeftDrivePID,
                                 rev::SparkMaxPIDController m_frontRightDrivePID,
                                 rev::SparkMaxPIDController m_rearLeftDrivePID,
                                 rev::SparkMaxPIDController m_rearRightDrivePID);


void SwerveDriveMotorConfigsCal(rev::SparkMaxPIDController m_frontLeftDrivePID,
                                rev::SparkMaxPIDController m_frontRightDrivePID,
                                rev::SparkMaxPIDController m_rearLeftDrivePID,
                                rev::SparkMaxPIDController m_rearRightDrivePID);

void DriveControlInit(void);

void DriveControlMain(double              L_JoyStick1Axis1Y,   // swerve control forward/back
                      double              L_JoyStick1Axis1X,   // swerve control strafe
                      double              L_JoyStick1Axis2X,   // rotate the robot joystick
                      double              L_JoyStick1Axis3,    // extra speed trigger
                      bool                LeDRC_b_RotateTo0,   // auto rotate to 0 degrees
                      bool                LeDRC_b_RotateTo180, // auto rotate to 180 degrees
                      bool                LeDRC_b_ZeroGyro,    // zero gyro
                      bool                LeDRC_b_X_ModeReq,
                      bool                LeDRC_b_X_ModeReqTeleop,
                      T_RobotState L_ROBO_e_RobotState,
                      T_ADAS_ActiveFeature LeDRC_e_ADAS_ActiveFeature,
                      double               L_ADAS_Pct_SD_FwdRev,
                      double               L_ADAS_Pct_SD_Strafe,
                      double               L_ADAS_Pct_SD_Rotate,
                      double               LeADAS_Deg_SD_DesiredPose,
                      bool                 L_ADAS_SD_RobotOriented,
                      double              L_Deg_GyroAngle,
                      double              L_Rad_GyroAngle,
                      double             *L_Deg_WheelAngleFwd,
                      double             *L_Deg_WheelAngleRev,
                      double             *Le_RPM_SD_WheelSpeedCmnd,
                      double             *L_k_SD_WheelAngleCmnd,
                      double              L_V_PDPVoltage);
