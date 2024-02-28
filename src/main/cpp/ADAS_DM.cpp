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

// TODO - cleanup function for 2024
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

    DtrmnSwerveBotLocationOut L_lookupOut;

    /* Set the things we are not using to off: */
    *LeADAS_b_SD_RobotOriented = false;

    /* Look up the desired target location point: */
    L_lookupOut = DesiredAutonLocation2(VeADAS_t_DM_StateTimer,
                                        LeADAS_e_ActiveFeature,
                                        LeLC_e_AllianceColor);

    /* Capture some of the things we need to save for this state control: */
    if (VeADAS_b_DM_StateInit == false)
    {
        VeADAS_l_DM_X_StartPosition = LeADAS_l_X_FieldPos;
        VeADAS_l_DM_Y_StartPosition = LeADAS_l_Y_FieldPos;
        VeADAS_l_DM_X_TargetStartPosition = L_lookupOut.L_valX;
        VeADAS_l_DM_Y_TargetStartPosition = L_lookupOut.L_valY;
        VeADAS_Deg_DM_StartAng = LeADAS_Deg_GyroAngle;
        VeADAS_Deg_DM_TargetStartAng = L_lookupOut.L_valDeg;
        VeADAS_b_DM_StateInit = true;
    }

    /* We need to offset the position by the start position since the odometry will
       start at zero, but the lookup table will not */
    L_lookupOut.L_valX -= VeADAS_l_DM_X_TargetStartPosition;
    L_lookupOut.L_valY -= VeADAS_l_DM_Y_TargetStartPosition;
    L_lookupOut.L_valDeg -= VeADAS_Deg_DM_TargetStartAng;

    LeADAS_l_RelativePosX = LeADAS_l_X_FieldPos - VeADAS_l_DM_X_StartPosition;
    LeADAS_l_RelativePosY = LeADAS_l_Y_FieldPos - VeADAS_l_DM_Y_StartPosition;
    LeADAS_Deg_RelativeAng = LeADAS_Deg_GyroAngle - VeADAS_Deg_DM_StartAng;

    LeADAS_l_X_Error = fabs(L_lookupOut.L_valX - LeADAS_l_RelativePosX);
    LeADAS_l_Y_Error = fabs(L_lookupOut.L_valY - LeADAS_l_RelativePosY);
    // LeADAS_Deg_RotateError = fabs(L_lookupOut.L_valDeg - LeADAS_Deg_RelativeAng);
    LeADAS_Deg_RotateErrorRaw = L_lookupOut.L_valDeg - LeADAS_Deg_RelativeAng;

    VeADAS_t_DM_StateTimer += C_ExeTime;

    if (LeADAS_Deg_RotateErrorRaw < -180)
    {
        L_lookupOut.L_valDeg += 360;
        LeADAS_Deg_RotateError = fabs(LeADAS_Deg_RotateErrorRaw + 360);
    }
    else if (LeADAS_Deg_RotateErrorRaw > 180)
    {
        L_lookupOut.L_valDeg -= 360;
        LeADAS_Deg_RotateError = fabs(LeADAS_Deg_RotateErrorRaw - 360);
    }
    else
    {
        LeADAS_Deg_RotateError = fabs(LeADAS_Deg_RotateErrorRaw);
    }

    if ((L_lookupOut.L_valDeg < -90) && (LeADAS_Deg_RelativeAng > 90))
    {
        L_lookupOut.L_valDeg += 360;
    }
    else if ((L_lookupOut.L_valDeg > 90) && (LeADAS_Deg_RelativeAng < -90))
    {
        L_lookupOut.L_valDeg -= 360;
    }

    /* Exit criteria: */
    if (LeADAS_Deg_RotateError <= K_ADAS_DM_RotateDeadbandAngle &&
        LeADAS_l_X_Error <= K_ADAS_DM_XY_Deadband &&
        LeADAS_l_Y_Error <= K_ADAS_DM_XY_Deadband &&
        VeADAS_t_DM_Debounce < KeADAS_t_DM_PathFollowDebounceTime &&
        L_lookupOut.L_timeEndReached == true)
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
        *LeADAS_Pct_Strafe = -Control_PID(L_lookupOut.L_valX,
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

        *LeADAS_Pct_FwdRev = -Control_PID(L_lookupOut.L_valY,
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

        *LeADAS_Pct_Rotate = Control_PID(L_lookupOut.L_valDeg,
                                         LeADAS_Deg_RelativeAng,
                                         &VeADAS_Deg_DM_AngleErrorPrev,
                                         &VeADAS_Deg_DM_AngleIntegral,
                                         KaADAS_k_AutonRotatePID_Gx[E_P_Gx],
                                         KaADAS_k_AutonRotatePID_Gx[E_I_Gx],
                                         KaADAS_k_AutonRotatePID_Gx[E_D_Gx],
                                         KaADAS_k_AutonRotatePID_Gx[E_P_Ul],
                                         KaADAS_k_AutonRotatePID_Gx[E_P_Ll],
                                         KaADAS_k_AutonRotatePID_Gx[E_I_Ul],
                                         KaADAS_k_AutonRotatePID_Gx[E_I_Ll],
                                         KaADAS_k_AutonRotatePID_Gx[E_D_Ul],
                                         KaADAS_k_AutonRotatePID_Gx[E_D_Ll],
                                         KaADAS_k_AutonRotatePID_Gx[E_Max_Ul],
                                         KaADAS_k_AutonRotatePID_Gx[E_Max_Ll]);

        *LeADAS_Deg_DesiredPose = L_lookupOut.L_valDeg;
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

    // frc::SmartDashboard::PutNumber("DM Timer", VeADAS_t_DM_StateTimer);
    // frc::SmartDashboard::PutNumber("X Cmnd",   L_lookupOut.L_valX);
    // // frc::SmartDashboard::PutNumber("X Act",    LeADAS_l_RelativePosX);
    // frc::SmartDashboard::PutNumber("X Err", LeADAS_l_X_Error);
    // frc::SmartDashboard::PutNumber("Y Cmnd",   L_lookupOut.L_valY);
    // // frc::SmartDashboard::PutNumber("Y Act",    LeADAS_l_RelativePosY);
    // frc::SmartDashboard::PutNumber("Y Err", LeADAS_l_Y_Error);
    // frc::SmartDashboard::PutNumber("Rot Cmnd", L_lookupOut.L_valDeg);
    // // frc::SmartDashboard::PutNumber("Rot Act",  LeADAS_Deg_RelativeAng);
    // frc::SmartDashboard::PutNumber("Rot Err", LeADAS_Deg_RotateError);
    // // frc::SmartDashboard::PutNumber("X start position", VeADAS_l_DM_X_TargetStartPosition);
    frc::SmartDashboard::PutBoolean("path complete", LeADAS_b_DM_StateComplete);

    return (LeADAS_b_DM_StateComplete);
}

/******************************************************************************
 * Function:     MoveWithGlobalCoords
 *
 * Description:  have the robot move to a pair of coordinates in a straight line
 ******************************************************************************/
bool ADAS_DM_MoveWithGlobalCoords(double *LeADAS_Pct_FwdRev,
                                  double *LeADAS_Pct_Strafe,
                                  double *LeADAS_Pct_Rotate,
                                  double L_CurrentOdomX,
                                  double L_CurrentOdomY,
                                  double L_CurrentYaw,
                                  double L_RequestedCoordX,
                                  double L_RequestedCoordY,
                                  double L_RequestedYaw)
{

    bool LeADAS_b_DM_StateComplete = false;

    double K_ADAS_DM_MinimumError = 5.0; // the amount of error (in inches) we're cool with

    double K_ADAS_DM_SlowDownError = 20.0; // with this much error or less we need to be more percise

    double L_ADAS_DM_SlopSpeeed = 0.0; // how fast we're gonna come at it

    // establish our errors
    double L_YawErrorRaw = L_CurrentYaw - L_RequestedYaw;
    double L_YawError = 0.0;
    double L_XError = L_CurrentOdomX - L_RequestedCoordX;
    double L_YError = L_CurrentOdomY - L_RequestedCoordY;

    if (L_YawErrorRaw < -180)
    {
        L_RequestedYaw += 360;
        L_YawError = fabs(L_YawErrorRaw + 360);
    }
    else if (L_YawErrorRaw > 180)
    {
        L_RequestedYaw -= 360;
        L_YawError = fabs(L_YawErrorRaw - 360);
    }
    else
    {
        L_YawError = fabs(L_YawErrorRaw);
    }

    // frc::SmartDashboard::PutNumber("odom X error", L_XError);
    // frc::SmartDashboard::PutNumber("odom Yaw error", L_YawError);
    // frc::SmartDashboard::PutNumber("raw odom yaw error", L_YawErrorRaw);

    if (fabs(L_YawError) > 5.0)
    {

        if (fabs(L_YawError) < 20.0)
        {
            
            // *LeADAS_Pct_Rotate = 0.2 * DesiredAutoRotateSpeed(L_YawError);
        }
        else
        {
            if((L_YawError > 0.0) && (L_YawErrorRaw > 0.0)){
                *LeADAS_Pct_Rotate = -DesiredAutoRotateSpeed(L_YawError);
            }
            else{
                *LeADAS_Pct_Rotate = DesiredAutoRotateSpeed(L_YawError);
            }
            
        }
    }
    else
    {
        *LeADAS_Pct_Rotate = 0.0;
    }

    // check if our error is big enough to bother moving
    if (fabs(L_XError) > K_ADAS_DM_MinimumError)
    {

        if (fabs(L_XError) < K_ADAS_DM_SlowDownError)
        {
            L_ADAS_DM_SlopSpeeed = 0.02; // TODO - arbitrary, change later
        }
        else
        {
            L_ADAS_DM_SlopSpeeed = 0.2;
        }

        *LeADAS_Pct_Strafe = 0.02 * L_XError;
    }
    else
    {
        *LeADAS_Pct_Strafe = 0.0;
    }
    if (fabs(L_YError) > K_ADAS_DM_MinimumError)
    {

        if (fabs(L_YError) < K_ADAS_DM_SlowDownError)
        {
            L_ADAS_DM_SlopSpeeed = 0.02; // TODO - arbitrary, change later
        }
        else
        {
            L_ADAS_DM_SlopSpeeed = .2;
        }

        *LeADAS_Pct_FwdRev = L_ADAS_DM_SlopSpeeed * L_YError;
    }
    else
    {
        *LeADAS_Pct_FwdRev = 0.0;
    }

    if ((fabs(L_XError) < K_ADAS_DM_MinimumError) && (fabs(L_YError) < K_ADAS_DM_MinimumError) && (fabs(L_YawError) < K_ADAS_DM_MinimumError))
    {
        LeADAS_b_DM_StateComplete = true;
    }
    return (LeADAS_b_DM_StateComplete);
}