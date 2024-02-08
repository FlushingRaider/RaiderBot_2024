/*
  ADAS_DM.cpp

  Created on: Mar 02, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems) Drive Manuvering (DM)
  Contains the logic and code used for the drive manuvering control.
  As of 03-02, this will just blindly launch the ball and drive forward
  for a set amount of time.

  Changes:
  2022-03-02 -> Beta, not tested....
 */

#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "control_pid.hpp"
#include "Const.hpp"
#include "Lookup.hpp"

double VeADAS_t_DM_Debounce = 0;
bool VeADAS_b_DM_StateInit = false;
double V_ADAS_DM_InitGyroAngle = 0;
double VeADAS_t_DM_StateTimer = 0;
double V_ADAS_DM_X_ErrorPrev = 0;
double V_ADAS_DM_Y_ErrorPrev = 0;
double VeADAS_Deg_DM_AngleErrorPrev = 0;
double V_ADAS_DM_X_Integral = 0;
double V_ADAS_DM_Y_Integral = 0;
double VeADAS_Deg_DM_AngleIntegral = 0;
double VeADAS_l_DM_X_StartPosition = 0;
double VeADAS_l_DM_Y_StartPosition = 0;
double VeADAS_Deg_DM_StartAng = 0;
double VeADAS_Deg_DM_TargetStartAng = 0;
double VeADAS_l_DM_X_TargetStartPosition = 0;
double VeADAS_l_DM_Y_TargetStartPosition = 0;
double V_ADAS_DM_TargetAngle;
double V_ADAS_DM_GyroPrevious;
bool V_ADAS_DM_GyroFlipNeg;
bool V_ADAS_DM_GyroFlipPos;

bool VeADAS_b_DM_AutoBalanceInit = false;
bool VeADAS_b_DM_AutoBalancePositive = false;
bool VeADAS_b_DM_AutoBalanceFastSearch = false;
double VeADAS_t_DM_AutoBalanceDbTm = 0.0;
double VeADAS_t_DM_AutoBalanceHoldTm = 0.0;
double VeADAS_Deg_DM_AutoBalanceErrorPrev = 0;
double VeADAS_Deg_DM_AutoBalanceIntegral = 0;

double VeADAS_t_DM_AutoMountDbTime = 0;

// For calibration:
double VaADAS_k_AutonXY_PID_Gx[E_PID_CalSz];
double VaADAS_k_AutonRotatePID_Gx[E_PID_CalSz];

double VeADAS_t_DM_DebounceTime = 0;

double V_TargetAngle;
double V_GyroPrevious;
bool V_GyroFlipNeg;
bool V_GyroFlipPos;
double V_OffsettedGyro;


//TODO - cleanup function for 2024
/******************************************************************************
 * Function:     ADAS_DM_PathFollower
 *
 * Description:  Follow the preplaned path.  We take in the current field
 *               position, then lookup the desired position based on the
 *               current time.  Based on this "error" between the current
 *               position and the desired position, we command either the
 *               robot to move foward/backward or to strafe to try and
 *               reach the desired position
 ******************************************************************************/
bool ADAS_DM_PathFollower(double *LeADAS_Pct_FwdRev,
                          double *LeADAS_Pct_Strafe,
                          double *LeADAS_Pct_Rotate,
                          double *LeADAS_Deg_DesiredPose,
                          bool *LeADAS_b_SD_RobotOriented,
                          double LeADAS_l_X_FieldPos,
                          double LeADAS_l_Y_FieldPos,
                          double LeADAS_Deg_GyroAngle,
                          T_ADAS_ActiveFeature LeADAS_e_ActiveFeature,
                          std::optional<frc::DriverStation::Alliance> LeLC_e_AllianceColor)
{
    bool LeADAS_b_DM_StateComplete = false;
    double LeADAS_l_TargetPositionX = 0.0;
    double LeADAS_l_TargetPositionY = 0.0;
    double LeADAS_Deg_TargetAngle = 0.0;
    double LeADAS_l_RelativePosX = 0.0;
    double LeADAS_l_RelativePosY = 0.0;
    double LeADAS_Deg_RelativeAng = 0.0;
    double LeADAS_l_X_Error = 0.0;
    double LeADAS_l_Y_Error = 0.0;
    double LeADAS_Deg_RotateError = 0.0;
    double LeADAS_Deg_RotateErrorRaw = 0.0;
    double LeADAS_t_TimeReaining = 0.0;
    bool LeADAS_b_TimeEndReached = false;
    double LeADAS_k_SlowSearch = 1.0;

    /* Set the things we are not using to off: */
    *LeADAS_b_SD_RobotOriented = false;

    /* Look up the desired target location point: */
    LeADAS_b_TimeEndReached = DesiredAutonLocation2(VeADAS_t_DM_StateTimer,
                                                    LeADAS_e_ActiveFeature,
                                                    LeLC_e_AllianceColor,
                                                    &LeADAS_l_TargetPositionX,
                                                    &LeADAS_l_TargetPositionY,
                                                    &LeADAS_Deg_TargetAngle,
                                                    &LeADAS_t_TimeReaining);

    /* Capture some of the things we need to save for this state control: */
    if (VeADAS_b_DM_StateInit == false)
    {
        VeADAS_l_DM_X_StartPosition = LeADAS_l_X_FieldPos;
        VeADAS_l_DM_Y_StartPosition = LeADAS_l_Y_FieldPos;
        VeADAS_l_DM_X_TargetStartPosition = LeADAS_l_TargetPositionX;
        VeADAS_l_DM_Y_TargetStartPosition = LeADAS_l_TargetPositionY;
        VeADAS_Deg_DM_StartAng = LeADAS_Deg_GyroAngle;
        VeADAS_Deg_DM_TargetStartAng = LeADAS_Deg_TargetAngle;
        VeADAS_b_DM_StateInit = true;
    }

    /* We need to offset the position by the start position since the odometry will
       start at zero, but the lookup table will not */
    LeADAS_l_TargetPositionX -= VeADAS_l_DM_X_TargetStartPosition;
    LeADAS_l_TargetPositionY -= VeADAS_l_DM_Y_TargetStartPosition;
    LeADAS_Deg_TargetAngle -= VeADAS_Deg_DM_TargetStartAng;

    LeADAS_l_RelativePosX = LeADAS_l_X_FieldPos - VeADAS_l_DM_X_StartPosition;
    LeADAS_l_RelativePosY = LeADAS_l_Y_FieldPos - VeADAS_l_DM_Y_StartPosition;
    LeADAS_Deg_RelativeAng = LeADAS_Deg_GyroAngle - VeADAS_Deg_DM_StartAng;

    LeADAS_l_X_Error = fabs(LeADAS_l_TargetPositionX - LeADAS_l_RelativePosX);
    LeADAS_l_Y_Error = fabs(LeADAS_l_TargetPositionY - LeADAS_l_RelativePosY);
    // LeADAS_Deg_RotateError = fabs(LeADAS_Deg_TargetAngle - LeADAS_Deg_RelativeAng);
    LeADAS_Deg_RotateErrorRaw = LeADAS_Deg_TargetAngle - LeADAS_Deg_RelativeAng;

    VeADAS_t_DM_StateTimer += C_ExeTime;

    if (LeADAS_Deg_RotateErrorRaw < -180)
    {
        LeADAS_Deg_TargetAngle += 360;
        LeADAS_Deg_RotateError = fabs(LeADAS_Deg_RotateErrorRaw + 360);
    }
    else if (LeADAS_Deg_RotateErrorRaw > 180)
    {
        LeADAS_Deg_TargetAngle -= 360;
        LeADAS_Deg_RotateError = fabs(LeADAS_Deg_RotateErrorRaw - 360);
    }
    else
    {
        LeADAS_Deg_RotateError = fabs(LeADAS_Deg_RotateErrorRaw);
    }

    // if (LeADAS_Deg_RotateErrorRaw < -180)
    //   {
    //   // LeADAS_Deg_TargetAngle += 360;
    //   LeADAS_Deg_RotateError = fabs(LeADAS_Deg_RotateErrorRaw + 360);
    //   }
    // else if (LeADAS_Deg_RotateErrorRaw > 180)
    //   {
    //   // LeADAS_Deg_TargetAngle -= 360;
    //   LeADAS_Deg_RotateError = fabs(LeADAS_Deg_RotateErrorRaw - 360);
    //   }
    // else
    //   {
    //   LeADAS_Deg_RotateError = fabs(LeADAS_Deg_RotateErrorRaw);
    //   }

    if ((LeADAS_Deg_TargetAngle < -90) && (LeADAS_Deg_RelativeAng > 90))
    {
        LeADAS_Deg_TargetAngle += 360;
    }
    else if ((LeADAS_Deg_TargetAngle > 90) && (LeADAS_Deg_RelativeAng < -90))
    {
        LeADAS_Deg_TargetAngle -= 360;
    }

    /* Exit criteria: */
    if (LeADAS_Deg_RotateError <= K_ADAS_DM_RotateDeadbandAngle &&
        LeADAS_l_X_Error <= K_ADAS_DM_XY_Deadband &&
        LeADAS_l_Y_Error <= K_ADAS_DM_XY_Deadband &&
        VeADAS_t_DM_Debounce < KeADAS_t_DM_PathFollowDebounceTime &&
        LeADAS_b_TimeEndReached == true)
    {
        VeADAS_t_DM_Debounce += C_ExeTime;
    }
    else if (VeADAS_t_DM_Debounce >= KeADAS_t_DM_PathFollowDebounceTime)
    {
        /* Reset the time, proceed to next state. */
        LeADAS_b_DM_StateComplete = true;
        VeADAS_t_DM_Debounce = 0;
    }
    else if (LeADAS_Deg_RotateError > K_ADAS_DM_RotateDeadbandAngle ||
             LeADAS_l_X_Error > K_ADAS_DM_XY_Deadband ||
             LeADAS_l_Y_Error > K_ADAS_DM_XY_Deadband)
    {
        /* Reset the timer, we have gone out of bounds */
        VeADAS_t_DM_Debounce = 0;
    }

    if (LeADAS_b_DM_StateComplete == false)
    {
        *LeADAS_Pct_Strafe = -Control_PID(LeADAS_l_TargetPositionX,
                                          LeADAS_l_RelativePosX,
                                          &V_ADAS_DM_X_ErrorPrev,
                                          &V_ADAS_DM_X_Integral,
                                          KaADAS_k_AutonXY_PID_Gx[E_P_Gx],
                                          KaADAS_k_AutonXY_PID_Gx[E_I_Gx],
                                          KaADAS_k_AutonXY_PID_Gx[E_D_Gx],
                                          KaADAS_k_AutonXY_PID_Gx[E_P_Ul],
                                          KaADAS_k_AutonXY_PID_Gx[E_P_Ll],
                                          KaADAS_k_AutonXY_PID_Gx[E_I_Ul],
                                          KaADAS_k_AutonXY_PID_Gx[E_I_Ll],
                                          KaADAS_k_AutonXY_PID_Gx[E_D_Ul],
                                          KaADAS_k_AutonXY_PID_Gx[E_D_Ll],
                                          KaADAS_k_AutonXY_PID_Gx[E_Max_Ul],
                                          KaADAS_k_AutonXY_PID_Gx[E_Max_Ll]);

        *LeADAS_Pct_FwdRev = -Control_PID(LeADAS_l_TargetPositionY,
                                          LeADAS_l_RelativePosY,
                                          &V_ADAS_DM_Y_ErrorPrev,
                                          &V_ADAS_DM_Y_Integral,
                                          KaADAS_k_AutonXY_PID_Gx[E_P_Gx],
                                          KaADAS_k_AutonXY_PID_Gx[E_I_Gx],
                                          KaADAS_k_AutonXY_PID_Gx[E_D_Gx],
                                          KaADAS_k_AutonXY_PID_Gx[E_P_Ul],
                                          KaADAS_k_AutonXY_PID_Gx[E_P_Ll],
                                          KaADAS_k_AutonXY_PID_Gx[E_I_Ul],
                                          KaADAS_k_AutonXY_PID_Gx[E_I_Ll],
                                          KaADAS_k_AutonXY_PID_Gx[E_D_Ul],
                                          KaADAS_k_AutonXY_PID_Gx[E_D_Ll],
                                          KaADAS_k_AutonXY_PID_Gx[E_Max_Ul],
                                          KaADAS_k_AutonXY_PID_Gx[E_Max_Ll]);

        *LeADAS_Pct_Rotate = Control_PID(LeADAS_Deg_TargetAngle,
                                         LeADAS_Deg_RelativeAng,
                                         &VeADAS_Deg_DM_AngleErrorPrev,
                                         &VeADAS_Deg_DM_AngleIntegral,
                                         VaADAS_k_AutonRotatePID_Gx[E_P_Gx],
                                         VaADAS_k_AutonRotatePID_Gx[E_I_Gx],
                                         VaADAS_k_AutonRotatePID_Gx[E_D_Gx],
                                         KaADAS_k_AutonRotatePID_Gx[E_P_Ul],
                                         KaADAS_k_AutonRotatePID_Gx[E_P_Ll],
                                         KaADAS_k_AutonRotatePID_Gx[E_I_Ul],
                                         KaADAS_k_AutonRotatePID_Gx[E_I_Ll],
                                         KaADAS_k_AutonRotatePID_Gx[E_D_Ul],
                                         KaADAS_k_AutonRotatePID_Gx[E_D_Ll],
                                         KaADAS_k_AutonRotatePID_Gx[E_Max_Ul],
                                         KaADAS_k_AutonRotatePID_Gx[E_Max_Ll]);

        *LeADAS_Deg_DesiredPose = LeADAS_Deg_TargetAngle;
    }
    else
    {
        /* We have been at the correct location for the set amount of time. */
        *LeADAS_Pct_FwdRev = 0;
        *LeADAS_Pct_Strafe = 0;
        *LeADAS_Pct_Rotate = 0;
        VeADAS_t_DM_Debounce = 0;
        VeADAS_t_DM_StateTimer = 0;
        LeADAS_b_DM_StateComplete = true;
        VeADAS_b_DM_StateInit = false;
        VeADAS_l_DM_X_StartPosition = 0;
        VeADAS_l_DM_Y_StartPosition = 0;
        VeADAS_l_DM_X_TargetStartPosition = 0;
        VeADAS_l_DM_Y_TargetStartPosition = 0;
        V_ADAS_DM_X_ErrorPrev = 0;
        V_ADAS_DM_X_Integral = 0;
        V_ADAS_DM_Y_ErrorPrev = 0;
        V_ADAS_DM_Y_Integral = 0;
        VeADAS_Deg_DM_AngleErrorPrev = 0;
        VeADAS_Deg_DM_AngleIntegral = 0;
    }

    frc::SmartDashboard::PutNumber("DM Timer", VeADAS_t_DM_StateTimer);
    // frc::SmartDashboard::PutNumber("X Cmnd",   LeADAS_l_TargetPositionX);
    // frc::SmartDashboard::PutNumber("X Act",    LeADAS_l_RelativePosX);
    frc::SmartDashboard::PutNumber("X Err", LeADAS_l_X_Error);
    // frc::SmartDashboard::PutNumber("Y Cmnd",   LeADAS_l_TargetPositionY);
    // frc::SmartDashboard::PutNumber("Y Act",    LeADAS_l_RelativePosY);
    frc::SmartDashboard::PutNumber("Y Err", LeADAS_l_Y_Error);
    // frc::SmartDashboard::PutNumber("Rot Cmnd", LeADAS_Deg_TargetAngle);
    // frc::SmartDashboard::PutNumber("Rot Act",  LeADAS_Deg_RelativeAng);
    frc::SmartDashboard::PutNumber("Rot Err", LeADAS_Deg_RotateError);

    return (LeADAS_b_DM_StateComplete);
}