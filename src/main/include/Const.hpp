#include "Enums.hpp"
#include <units/time.h>
#include <units/angle.h>
#include <units/length.h>

// Define the desired test state here: COMP (no test), DriveMotorTest, WheelAngleTest, ADAS_DM_Test
#define COMP

// Numerical constants
const double C_RadtoDeg = 57.2957795;
const double C_Deg2Rad = 0.017453292519943295;
const double C_MeterToIn = 39.37008;
const double C_PI = 3.14159265358979;
const double C_Tau = 6.28318530717958647;
const double C_ExeTime = 0.02;              // Set to match the the default controller loop time of 20 ms
const units::second_t C_ExeTime_t = 0.02_s; // Set to match the the default controller loop time of 20 ms
// CAN Device IDs:
static const int C_PDP_ID = 21;
static const int frontLeftSteerDeviceID = 1, frontLeftDriveDeviceID = 2, frontRightSteerDeviceID = 4, frontRightDriveDeviceID = 3;
static const int rearLeftSteerDeviceID = 5, rearLeftDriveDeviceID = 6, rearRightSteerDeviceID = 7, rearRightDriveDeviceID = 8;
static const int KeGRY_i_Gyro = 16;
static const int KeEnc_i_WheelAngleFL = 17;
static const int KeEnc_i_WheelAngleFR = 18;
static const int KeEnc_i_WheelAngleRL = 19;
static const int KeEnc_i_WheelAngleRR = 20;
/******************************************************************************
 * Section: Swerve_Constants
 *
 * Description:  All of our Swerve Cals and Calculations
 *
 * Author: 5561
 ******************************************************************************/

// Encoder / speed calculation related cals
/* KeENC_k_ReductionRatio: Reduction ratio for swerve drive module. mk4 */
const double KeENC_k_ReductionRatio = 8.33;

/* KeENC_In_WheelCircumfrence: Circumferance of wheel, in inches (4in nominal diameter). mk4 */
const double KeENC_In_WheelCircumfrence = 12.566;

// Constants and cals for Swerve Drive (SD) control:
/* C_SD_L: Robot wheelbase. [meters] */
const double C_SD_L = 0.5969;

/* C_SD_W: Robot track width. [meters] */
const double C_SD_W = 0.5969;

/* C_SD_R: Constant composed of the C_SD_W and C_SD_L constants: R = sqrt(L^2 + W^2) [meters]*/
const double C_SD_R = 0.8441;

/* K_SD_SteerMotorCurrentLimit: Max allowed current going to each swerve drive steer motor. */
const double K_SD_SteerMotorCurrentLimit = 25;

/* KeENC_Deg_SD_WheelOffsetAngle: Offset angle for each respective corder of the swerve drive wheel.  This is the angle
   reading from the absolute encoder that is indicated in order for the wheel to point straight. */
const double KeENC_Deg_SD_WheelOffsetAngle[E_RobotCornerSz] = {158.291015625, // E_FrontLeft 152.578125
                                                               218.14453125,  // E_FrontRight 212.783203
                                                               117.509765625, // E_RearLeft 118.740234
                                                               82.79296875};  // E_RearRight 76.289063

/* K_SD_WheelGx: Gain multiplied by each calculated desired speed.  Intended to account for variation in wheel size. */
const double K_SD_WheelGx[E_RobotCornerSz] = {-1.0,  // E_FrontLeft
                                              -1.0,  // E_FrontRight
                                              -1.0,  // E_RearLeft
                                              -1.0}; // E_RearRight

/* KeDRC_k_SD_MinGain: Min gain applied to the wheel speed for swerve drive. */
const double KeDRC_k_SD_MinGain = 0.2;

/* K_SD_MaxGain: Max gain allowed for swerve drive control. */
const double K_SD_MaxGain = 0.7;

/* KeDRC_k_SD_AutonGain: Gain applied to the auton commands coming in for swerve drive. */
const double KeDRC_k_SD_AutonGain = 0.9;

/* K_SD_WheelMaxSpeed: Max RPM speed of the swerve drive wheel motor.*/
const double K_SD_WheelMaxSpeed = 6000;

/* Ke_RPM_SD_WheelMinCmndSpeed: Min RPM speed of the swerve drive wheel to keep it under PID control.
  If the absolute value of the command, wheels will transition to 0 power (but still in brake
  mode).  There is a corresponding actual speed threshold. [RPM] */
const double Ke_RPM_SD_WheelMinCmndSpeed = 0.2;

/* RotateDeadBand: Check Rotation value approx 0 */
const double K_SD_RotateDeadBand = 0.05;

/* KeDRC_Pct_SD_StrafeDB: Check Rotation value approx 0 */
const double KeDRC_Pct_SD_StrafeDB = 0.75;

/* Ke_k_SD_SignX: Determines sign of the calculation for the X component of the swerve drive offset. */
const double Ke_k_SD_SignX[E_RobotCornerSz] = {1.0,   // E_FrontLeft
                                               1.0,   // E_FrontRight
                                               -1.0,  // E_RearLeft
                                               -1.0}; // E_RearRight

/* Ke_k_SD_SignY: Determines sign of the calculation for the Y component of the swerve drive offset. */
const double Ke_k_SD_SignY[E_RobotCornerSz] = {1.0,   // E_FrontLeft
                                               -1.0,  // E_FrontRight
                                               1.0,   // E_RearLeft
                                               -1.0}; // E_RearRight

/**********
 *
 * Wheel PIDs
 *
 ***********/
/* K_SD_WheelSpeedPID_V2_Gx: PID gains for the driven wheels that is within the motor controllers. */
const double K_SD_WheelSpeedPID_V2_Gx[E_PID_SparkMaxCalSz] = {0.000350, // kP
                                                              0.000001, // kI
                                                              0.000001, // kD
                                                              0.0,      // kIz
                                                              0.0,      // kFF
                                                              1.0,      // kMaxOutput
                                                              -1.0,     // kMinOutput
                                                              0.0,      // kMaxVel
                                                              0.0,      // kMinVel
                                                              150.0,    // kMaxAcc 150
                                                              0.0};     // kAllErr

/* K_SD_WheelAnglePID_Gx: PID gains for the angle of the swerve drive wheels.  PID control is within the RoboRio.  */
const double K_SD_WheelAnglePID_Gx[E_PID_CalSz] = {0.0035,   // P Gx  0.002
                                                   0.000001, // I Gx 0.000001
                                                   0.000005, // D Gx 0.0000005
                                                   1.0,      // P UL 0.6
                                                   -1.0,     // P LL -0.4
                                                   0.15,     // I UL 0.12
                                                   -0.15,    // I LL -0.12
                                                   1.0,      // D UL 0.5
                                                   -1.0,     // D LL -0.5
                                                   1.0,      // Max upper 0.9
                                                   -1.0};    // Max lower -0.9

/**********
 *
 * AutoCorrect Swerve Cals
 *
 ***********/
/* Ke_SD_AutoCorrectPID_Gx: PID gains for the auto correct.  PID control is within the RoboRio.  */
const double Ke_SD_AutoCorrectPID_Gx[E_PID_CalSz] = {40.0,    // P Gx  75
                                                     0.5,     // I Gx 0.03
                                                     0.0005,  // D Gx 0.0005
                                                     300.0,   // P UL 0.6
                                                     -300.0,  // P LL -0.4
                                                     300.0,   // I UL 0.12
                                                     -300.0,  // I LL -0.12
                                                     1.0,     // D UL 0.5
                                                     -1.0,    // D LL -0.5
                                                     400.0,   // Max upper 0.9
                                                     -400.0}; // Max lower -0.9

/* Ke_k_SD_AutoCorrectMaxWheelOffset: Max percent offset of wheelspeed.*/
const double Ke_k_SD_AutoCorrectMaxWheelOffset = 0.8;

/**********
 *
 * Joystick to Swerve Arrays
 *
 ***********/
/* K_SD_DesiredDriveSpeedAxis: Joystick scale axis for K_SD_DesiredDriveSpeed.  */
const double K_SD_DesiredDriveSpeedAxis[20] = {-0.95,
                                               -0.85,
                                               -0.75,
                                               -0.65,
                                               -0.55,
                                               -0.45,
                                               -0.35,
                                               -0.25,
                                               -0.15,
                                               -0.05,
                                               0.05,
                                               0.15,
                                               0.25,
                                               0.35,
                                               0.45,
                                               0.55,
                                               0.65,
                                               0.75,
                                               0.85,
                                               0.95};

/* K_SD_DesiredDriveSpeed: Joystick scaled output for swerve drive control.  Used as debouncing and to help limit speeds at lower joystick inputs values.  */
const double K_SD_DesiredDriveSpeed[20] = {-1.00, //-1
                                           -0.88, //-0.88
                                           -0.50, //-0.6
                                           -0.35, //-0.4
                                           -0.20, //-0.25
                                           -0.15, //-0.2
                                           -0.10, //-0.15
                                           -0.06, //-0.1
                                           -0.02, //-0.05
                                           0.00,  //-0.0
                                           0.00,  // 0.0
                                           0.02,  // 0.15
                                           0.06,  // 0.25
                                           0.10,  // 0.35
                                           0.15,  // 0.45
                                           0.20,  // 0.55
                                           0.35,  // 0.65
                                           0.50,  // 0.75
                                           0.88,  // 0.85
                                           1.00}; // 0.95

/* K_SD_DesiredRotateAxis: Joystick scale axis for K_SD_DesiredDriveSpeed.  */
const double K_SD_DesiredRotateAxis[20] = {-0.95,
                                           -0.85,
                                           -0.75,
                                           -0.65,
                                           -0.55,
                                           -0.45,
                                           -0.35,
                                           -0.25,
                                           -0.15,
                                           -0.05,
                                           0.05,
                                           0.15,
                                           0.25,
                                           0.35,
                                           0.45,
                                           0.55,
                                           0.65,
                                           0.75,
                                           0.85,
                                           0.95};

/* K_SD_DesiredRotate: Joystick scaled output for swerve drive control.  Used as debouncing and to help limit speeds at lower joystick inputs values.  */
const double K_SD_DesiredRotate[20] = {-1.00, //-1
                                       -0.50, //-0.88
                                       -0.20, //-0.6
                                       -0.11, //-0.4
                                       -0.09, //-0.25
                                       -0.07, //-0.2
                                       -0.05, //-0.15
                                       -0.03, //-0.1
                                       -0.01, //-0.05
                                       0.00,  //-0.0
                                       0.00,  // 0.0
                                       0.01,  // 0.15
                                       0.03,  // 0.25
                                       0.05,  // 0.35
                                       0.07,  // 0.45
                                       0.09,  // 0.55
                                       0.11,  // 0.65
                                       0.20,  // 0.75
                                       0.50,  // 0.85
                                       1.00}; // 0.95

/* KnLU_k_SD_DesiredAccelAxis: Joystick scale axis for KtLU_k_SD_DesiredAccel.  */
const double KnLU_k_SD_DesiredAccelAxis[20] = {-0.95,
                                               -0.85,
                                               -0.75,
                                               -0.65,
                                               -0.55,
                                               -0.45,
                                               -0.35,
                                               -0.25,
                                               -0.15,
                                               -0.10,
                                               0.10,
                                               0.15,
                                               0.25,
                                               0.35,
                                               0.45,
                                               0.55,
                                               0.65,
                                               0.75,
                                               0.85,
                                               0.95};

/* KtLU_k_SD_DesiredAccel: Joystick scaled output for swerve drive control.  Used as debouncing and to help limit speeds at lower joystick inputs values.  */
const double KtLU_k_SD_DesiredAccel[20] = {-1.00, //-1
                                           -0.88, //-0.88
                                           -0.50, //-0.6
                                           -0.35, //-0.4
                                           -0.20, //-0.25
                                           -0.15, //-0.2
                                           -0.10, //-0.15
                                           -0.06, //-0.1
                                           -0.02, //-0.05
                                           0.00,  //-0.0
                                           0.00,  // 0.0
                                           0.02,  // 0.15
                                           0.06,  // 0.25
                                           0.10,  // 0.35
                                           0.15,  // 0.45
                                           0.20,  // 0.55
                                           0.35,  // 0.65
                                           0.50,  // 0.75
                                           0.88,  // 0.85
                                           1.00}; // 0.95

/* ADAS DM cals*/
const double KaADAS_k_AutonXY_PID_Gx[E_PID_CalSz] = { 0.050,       // P Gx
                                                      0.00060,    // I Gx 0.000001
                                                      0.00007,      // D Gx
                                                      0.60,       // P UL
                                                     -0.60,       // P LL
                                                      0.3,      // I UL
                                                     -0.3,      // I LL
                                                      0.5,       // D UL
                                                     -0.5,       // D LL
                                                      1.0,       // Max upper
                                                     -1.0};      // Max lower

                                                     
const double KaADAS_k_AutonRotatePID_Gx[E_PID_CalSz] = { 0.00020,     // P Gx
                                                         0.00020, // I Gx
                                                         0.00007,  // D Gx
                                                         0.29,      // P UL
                                                        -0.29,      // P LL
                                                         0.175,     // I UL
                                                        -0.175,     // I LL
                                                         0.16,      // D UL
                                                        -0.16,      // D LL
                                                         0.4,      // Max upper
                                                        -0.4};     // Max lower


/* KeADAS_t_DM_PathFollowDebounceTime: This is the debounce time for the DM pathfollow state. [seconds] */
const double KeADAS_t_DM_PathFollowDebounceTime = 0.045;
/* K_ADAS_DM_RotateDeadbandAngle: This is the deband angle for the DM rotate state. [degrees] */
const double K_ADAS_DM_RotateDeadbandAngle = 2.0;

/* K_ADAS_DM_XY_Deadband: This is the deband position for the DM XY drive state. [meters] */
const double K_ADAS_DM_XY_Deadband = 3.0;



/*  Auto Rotation calibrations */
/* K_DesiredRotateSpeedAxis - This is the effective command axis, function of error calculation, in degrees */
const double K_DesiredRotateSpeedAxis[10] = {-20.0,
                                             -4.0,
                                             -2.0,
                                             -1.0,
                                             -0.2,
                                             0.2,
                                             1.0,
                                             2.0,
                                             4.0,
                                             20.0};

/* K_DesiredRotateSpeed - This is the effective command, equivalent to the rotate joystick */
const double K_DesiredRotateSpeed[10] = {-0.60,  // -20.0
                                         -0.12,  //  -4.0
                                         -0.035, //  -2.0
                                         -0.018, //  -1.0
                                         0.02,   //  -0.2
                                         0.02,   //   0.2
                                         0.018,  //   1.0
                                         0.035,  //   2.0
                                         0.012,  //   4.0
                                         0.60};  //  20.0

/* K_DesiredAutoRotateSpeedAxis - This is the effective command axis, function of error calculation, in degrees */
const double K_DesiredAutoRotateSpeedAxis[10] = {-4.0,
                                                 -3.0,
                                                 -2.0,
                                                 -1.0,
                                                 -0.2,
                                                 0.2,
                                                 1.0,
                                                 2.0,
                                                 3.0,
                                                 4.0};

/* K_DesiredRotateSpeed - This is the effective command, equivalent to the rotate joystick */
const double K_DesiredAutoRotateSpeed[10] = {-0.15,  //  -4.0
                                             -0.02,  //  -3.0
                                             -0.008, //  -2.0
                                             -0.005, //  -1.0
                                             0.00,   //  -0.2
                                             0.00,   //   0.2
                                             -0.005, //   1.0
                                             0.008,  //   2.0
                                             0.02,   //   3.0
                                             0.15};  //   4.0

// Gyro cals
/* KeGRY_ms_GyroTimeoutMs: Set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action fails. */
const units::second_t KeGRY_ms_GyroTimeoutMs = 30_s; // Waits and reports to DS if fails

// 2023 bot
static const int KeMAN_i_LinearSlide = 10;
static const int KeMAN_i_ArmPivot = 11;
static const int KeMAN_i_Wrist = 12;
static const int KeMAN_i_Gripper = 13;
static const int KeINT_i_PCM = 22;
