#include "Enums.hpp"
#include <units/time.h>
#include <units/angle.h>
#include <units/length.h>

// Define the desired rotob state here: Bot2024, Bot2023
#define Bot2024
// Define the desired test mode, if none, leave commented out: SPK_Test, SPK_ShooterOverride, AMP_Test, CLMR_Test, DriveMotorTest, WheelAngleTest, ADAS_DM_Test
// #define AMP_Test

// Numerical constants
const double C_RadtoDeg = 57.2957795;
const double C_Deg2Rad = 0.017453292519943295;
const double C_MeterToIn = 39.37008;
const double C_PI = 3.14159265358979;
const double C_Tau = 6.28318530717958647;
const double C_ExeTime = 0.02;              // Set to match the the default controller loop time of 20 ms
const units::second_t C_ExeTime_t = 0.02_s; // Set to match the the default controller loop time of 20 ms
// CAN Device IDs:
static const int C_PDP_ID = 1;
static const int frontLeftSteerDeviceID = 4, frontLeftDriveDeviceID = 3, frontRightSteerDeviceID = 23, frontRightDriveDeviceID = 2; // ToDo: update RightSteer from 1 to 23
static const int rearLeftSteerDeviceID = 7, rearLeftDriveDeviceID = 8, rearRightSteerDeviceID = 5, rearRightDriveDeviceID = 6;

static const int KeDJ_Amp_i_Intake = 11;
static const int KeDJ_Amp_i_Wrist = 12; // NOTE: these are temporary mechinism ids they have not been flashed 10 - 15
static const int KeDJ_Spk_i_Underbelly = 13;
static const int KeDJ_Spk_i_Shooter1 = 14;
static const int KeDJ_Spk_i_Shooter2 = 15;
static const int KeGRY_i_Gyro = 16;
static const int KeEnc_i_WheelAngleFL = 18;
static const int KeEnc_i_WheelAngleFR = 17;
static const int KeEnc_i_WheelAngleRL = 20;
static const int KeEnc_i_WheelAngleRR = 19;
static const int KeCLMR_i_ClimberArm1 = 21;
static const int KeCLMR_i_ClimberArm2 = 22;
static const int KeDJ_Spk_i_IAssist = 24;

// LED Constansts;
const int C_LedLength = 900;
const int C_LEDPort = 9;


// DIO Device IDs:
static const int KeSPK_i_BreakBeamIO  = 0;
static const int KeSPK_i_ElevatorSwitchIO  = 2;

#ifdef Bot2024
static const int KeDJ_Amp_i_Elevator = 10;
#else
static const int KeMAN_i_LinearSlide = 10;
#endif

/******************************************************************************
 * Section: Swerve_Constants
 *
 * Description:  All of our Swerve Cals and Calculations
 *
 * Author: 5561
 ******************************************************************************/

// Encoder / speed calculation related cals
#ifdef Bot2023
/* KeENC_k_ReductionRatio: Reduction ratio for swerve drive module. mk4 */
const double KeENC_k_ReductionRatio = 8.33; 

// Constants and cals for Swerve Drive (SD) control:
/* C_SD_L: Robot wheelbase. [meters] */
const double C_SD_L = 0.5969;

/* C_SD_W: Robot track width. [meters] */
const double C_SD_W = 0.5969;

/* C_SD_R: Constant composed of the C_SD_W and C_SD_L constants: R = sqrt(L^2 + W^2) [meters]*/
const double C_SD_R = 0.8441;
#else
/* KeENC_k_ReductionRatio: Reduction ratio for swerve drive module. mk4 */
const double KeENC_k_ReductionRatio = 6.12; 

// Constants and cals for Swerve Drive (SD) control:
/* C_SD_L: Robot wheelbase. [meters] */
const double C_SD_L = 0.53;

/* C_SD_W: Robot track width. [meters] */
const double C_SD_W = 0.53;

/* C_SD_R: Constant composed of the C_SD_W and C_SD_L constants: R = sqrt(L^2 + W^2) [meters]*/
const double C_SD_R = 0.7495;
#endif
/* KeENC_In_WheelCircumfrence: Circumferance of wheel, in inches (4in nominal diameter). mk4 */
const double KeENC_In_WheelCircumfrence = 12.566;

/* K_SD_SteerMotorCurrentLimit: Max allowed current going to each swerve drive steer motor. */
const double K_SD_SteerMotorCurrentLimit = 25;

/* K_SD_SteerMotorCurrentLimit: Max allowed current going to each swerve drive steer motor. */
const double K_SD_DriveMotorCurrentLimit = 48;

#ifdef Bot2023
/* KeENC_Deg_SD_WheelOffsetAngle: Offset angle for each respective corder of the swerve drive wheel.  This is the angle
   reading from the absolute encoder that is indicated in order for the wheel to point straight. */
const double KeENC_Deg_SD_WheelOffsetAngle[E_RobotCornerSz] = {158.291015625, // E_FrontLeft 152.578125
                                                               218.14453125,  // E_FrontRight 212.783203
                                                               117.509765625, // E_RearLeft 118.740234
                                                               82.79296875};  // E_RearRight 76.289063

const double K_ENC_WheelAngleGx = 1.0;
#else
/* KeENC_Deg_SD_WheelOffsetAngle: Offset angle for each respective corder of the swerve drive wheel.  This is the angle
   reading from the absolute encoder that is indicated in order for the wheel to point straight. */
const double KeENC_Deg_SD_WheelOffsetAngle[E_RobotCornerSz] = {157.466796, // E_FrontLeft 157.466796
                                                               357.1875,  // E_FrontRight 357.1875
                                                               134.65, // E_RearLeft 176.394531  172.89453
                                                               258.4863};  // E_RearRight 258.4863

const double K_ENC_WheelAngleGx = -1.0;
#endif
/* K_SD_WheelGx: Gain multiplied by each calculated desired speed.  Intended to account for variation in wheel size. */
const double K_SD_WheelGx[E_RobotCornerSz] = {-1.0,  // E_FrontLeft
                                              -1.0,  // E_FrontRight
                                              -1.0,  // E_RearLeft
                                              -1.0}; // E_RearRight

/* KeDRC_k_SD_MinGain: Min gain applied to the wheel speed for swerve drive. */
const double KeDRC_k_SD_MinGain = 0.2;

/* K_SD_MaxGain: Max gain allowed for swerve drive control. */
const double K_SD_MaxGain = 0.9; // 0.62

/* KeDRC_k_SD_AutonGain: Gain applied to the auton commands coming in for swerve drive. */
const double KeDRC_k_SD_AutonGain = 0.9;

/* K_SD_TeleWheelMaxSpeed: Max RPM speed of the swerve drive wheel motor.*/
const double K_SD_TeleWheelMaxSpeed = 6000;

const double K_SD_AutonWheelMaxSpeed = 4500;

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

/*****************************
 *
 * Wheel PIDs
 *
 *****************************/
#ifdef Bot2023
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
#else
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
const double K_SD_WheelAnglePID_Gx[E_PID_CalSz] = {0.003,   // P Gx  0.002
                                                   0.000003, // I Gx 0.000001
                                                   0.000005, // D Gx 0.0000005
                                                   1.0,      // P UL 0.6
                                                   -1.0,     // P LL -0.4
                                                   0.15,     // I UL 0.12
                                                   -0.15,    // I LL -0.12
                                                   1.0,      // D UL 0.5
                                                   -1.0,     // D LL -0.5
                                                   1.0,      // Max upper 0.9
                                                   -1.0};    // Max lower -0.9
#endif
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
const double KaADAS_k_AutonXY_PID_Gx[E_PID_CalSz] = {0.050,   // P Gx
                                                     0.000000001, // I Gx 0.000001
                                                     0.00007, // D Gx
                                                     0.7,    // P UL
                                                     -0.7,   // P LL
                                                     0.03,     // I UL
                                                     -0.03,    // I LL
                                                     0.5,     // D UL
                                                     -0.5,    // D LL
                                                     1.0,     // Max upper
                                                     -1.0};   // Max lower

const double KaADAS_k_AutonRotatePID_Gx[E_PID_CalSz] = {0.0045, // P Gx
                                                        0.00000000005, // I Gx
                                                        0.00007, // D Gx
                                                        0.25,    // P UL
                                                        -0.25,   // P LL
                                                        0.04,   // I UL
                                                        -0.04,  // I LL
                                                        0.16,    // D UL
                                                        -0.16,   // D LL
                                                        0.4,     // Max upper
                                                        -0.4};   // Max lower

/* KeADAS_t_DM_PathFollowDebounceTime: This is the debounce time for the DM pathfollow state. [seconds] */
const double KeADAS_t_DM_PathFollowDebounceTime = 0.045; // 0.045
/* K_ADAS_DM_RotateDeadbandAngle: This is the deband angle for the DM rotate state. [degrees] */
const double K_ADAS_DM_RotateDeadbandAngle = 5.0;  // 3.0

/* K_ADAS_DM_XY_Deadband: This is the deband position for the DM XY drive state. [meters] */
const double K_ADAS_DM_XY_Deadband = 3.0; // 3.0

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

/******************************************************************************
 * Section: DJ_Const
 *
 * Description:  Any calculations, PIDs, and anything else needed to the run Amp and Spk mechanisms
 *
 * Author: 5561
 ******************************************************************************/

/* KaMAN_e_ControllingTable: Table that contains the commanded state of the manipulator and intake based on the current attained state and schedueld state. */
const T_DJ_Amp_States KaDJ_Amp_e_ControllingTable[E_DJ_Amp_State_Sz][E_DJ_Amp_State_Sz] = // [Sched][Attnd]
    {{E_DJ_Amp_Init,    E_DJ_Amp_Init,      E_DJ_Amp_Init,      E_DJ_Amp_Driving,  E_DJ_Amp_Driving,  E_DJ_Amp_Driving,   E_DJ_Amp_Driving,   E_DJ_Amp_Driving},
     {E_DJ_Amp_Init,    E_DJ_Amp_Init,      E_DJ_Amp_Init,      E_DJ_Amp_Driving,  E_DJ_Amp_Driving,  E_DJ_Amp_Driving,   E_DJ_Amp_Driving,   E_DJ_Amp_Driving},
     {E_DJ_Amp_Driving, E_DJ_Amp_Intake,    E_DJ_Amp_Intake,    E_DJ_Amp_Driving,  E_DJ_Amp_Driving,  E_DJ_Amp_Driving,   E_DJ_Amp_Intake,    E_DJ_Amp_Driving},
     {E_DJ_Amp_Driving, E_DJ_Amp_PreScore,  E_DJ_Amp_Driving,   E_DJ_Amp_PreScore, E_DJ_Amp_PreScore, E_DJ_Amp_Driving,   E_DJ_Amp_Driving,   E_DJ_Amp_PreScore},
     {E_DJ_Amp_Driving, E_DJ_Amp_PreScore,  E_DJ_Amp_Driving,   E_DJ_Amp_Score,    E_DJ_Amp_Score,    E_DJ_Amp_Driving,   E_DJ_Amp_Driving,   E_DJ_Amp_PreScore},
     {E_DJ_Amp_Driving, E_DJ_Amp_Trade_Off, E_DJ_Amp_Trade_Off, E_DJ_Amp_Driving,  E_DJ_Amp_Driving,  E_DJ_Amp_Trade_Off, E_DJ_Amp_Trade_Off, E_DJ_Amp_Driving},
     {E_DJ_Amp_Driving, E_DJ_Amp_iAssist,   E_DJ_Amp_iAssist,   E_DJ_Amp_Driving,  E_DJ_Amp_Driving,  E_DJ_Amp_iAssist,   E_DJ_Amp_iAssist,   E_DJ_Amp_Driving},
     {E_DJ_Amp_Source,  E_DJ_Amp_Source,    E_DJ_Amp_Driving,   E_DJ_Amp_PreScore, E_DJ_Amp_Source,   E_DJ_Amp_Driving,   E_DJ_Amp_Driving,   E_DJ_Amp_Source}};

/* KeAmp_I_WristCurrentLimit: Max allowed current going to wrist motor. */
const double KeAmp_I_WristCurrentLimit = 30;

/* KeAmp_t_WristResetTime: Amount of time allowed to reset wrist when commanded to init. */
const double KeAmp_t_WristResetTime = 0.5;

/* KeAmp_t_WristResetTime2: Amount of time allowed to reset wrist when commanded to init. */
const double KeAmp_t_WristResetTime2 = 2;

/* KeAmp_k_WristResetPwr: Amount of power commanded to reset wrist. */
const double KeAmp_k_WristResetPwr = -0.05; // -0.2

/* KeSPK_I_ElevatorCurrentLimit: Max allowed current going to elevator motor. */
const double KeAmp_I_ElevatorCurrentLimit = 20;

/* KeAmp_k_ElevatorResetPwr: Amount of power commanded to reset elevator. */
const double KeAmp_k_ElevatorResetPwr = 0.1;

/* KaDJ_Amp_k_ElevatorPID_Gx: PID gains for the Elevator control. */
const double KaDJ_Amp_k_ElevatorPID_Gx[E_PID_SparkMaxCalSz] = {0.1,      // kP
                                                               0.000001, // kI
                                                               0.002000, // kD
                                                               0.0,      // kIz
                                                               0.0,      // kFF
                                                               1.0,      // kMaxOut
                                                              -1.0,     // kMinOut
                                                               1.05,     // kMaxVel
                                                               0.5,      // kMinVel
                                                               0.0,      // kMaxAcc
                                                               0.0};     // kAllErr

/* KaDJ_Amp_k_WristPID_Gx: PID gains for the Wrist control. */
const double KaDJ_Amp_k_WristPID_Gx[E_PID_SparkMaxCalSz] = {0.73,      // kP 0.1
                                                            0.000006, // kI 0.000001
                                                            0.002000, // kD
                                                            0.0,      // kIz
                                                            0.0,      // kFF
                                                            1.0,      // kMaxOut
                                                           -1.0,     // kMinOut
                                                            1.05,     // kMaxVel
                                                            0.5,      // kMinVel
                                                            0.0,      // kMaxAcc
                                                            0.0};     // kAllErr

/* KeAmp_I_IntakeCurrentLimit: Max allowed current going to amp intake motor. */
const double KeAmp_I_IntakeCurrentLimit = 40;

/* KaDJ_Amp_k_IntakePID_Gx: PID gains for the Gripper control. */
const double KaDJ_Amp_k_IntakePID_Gx[E_PID_SparkMaxCalSz] = {0.1,      // kP
                                                             0.000001, // kI
                                                             0.002000, // kD
                                                             0.0,      // kIz
                                                             0.0,      // kFF
                                                             1.0,      // kMaxOut
                                                            -1.0,     // kMinOut
                                                             1.05,     // kMaxVel
                                                             0.5,      // kMinVel
                                                             0.0,      // kMaxAcc
                                                             0.0};     // kAllErr

/* KaDJ_Amp_k_TestPower: Test power output for the manipulator controls. ONLY used in test mode!! */
const double KaDJ_Amp_k_TestPower[E_Amp_Sz] = {0.10,  // E_Amp_Elevator 
                                               0.95,  // E_Amp_Wrist
                                               0.40}; // E_Amp_Intake

/* KeDJ_Amp_t_StateTimeOut: Sets transition time out. */
const double KeDJ_Amp_t_StateTimeOut = 2.0;

/* KeDJ_Amp_k_HoldNote: Sets rollers to hold note. */
const double KeDJ_Amp_k_HoldNote = -0.1;

/* KeAmp_t_IntakeOnTm: Amount of time Amp intake will remain on after it is initially commanded on. */
const double KeAmp_t_IntakeOnTm = 0.5; //NOTE - set calibration time

/* KaDJ_Amp_RPM_IntakePower: sets Intake power for each state */
const double KaDJ_Amp_RPM_IntakePower[E_DJ_Amp_State_Sz] = { 0.0,   // Sched - Init
                                                             0.0,   // Sched - Driving
                                                            -0.45, // Sched - Main Intake
                                                             0.0,   // Sched - PreScore
                                                             0.45,  // Sched - Score
                                                             0.30,  // Sched - Trade_Off
                                                             0.30, // Sched - iAssist
                                                             -0.45}; // Sched - Source

/* KaDJ_Amp_Deg_WristAngle: sets Wrist final positons for each state */
const double KaDJ_Amp_Deg_WristAngle[E_DJ_Amp_State_Sz] = {  0.00,  // Sched - Init
                                                             31.00,  // Sched - Driving
                                                             150.00,  // Sched -  Intake 118 
                                                             112.00,  // Sched - PreScore 85
                                                             112.00, // Sched - Score 85
                                                             77.00, // Sched - Trade_Off 50
                                                             110.00, // Sched - iAssist
                                                             49.10}; // Sched - Source 22.1

/* KaDJ_Amp_In_ElevatorPosition: sets Elevator final positons for each state */
const double KaDJ_Amp_In_ElevatorPosition[E_DJ_Amp_State_Sz] = {0.0,  // Sched - Init
                                                                0.0,  // Sched - Driving
                                                                0.0,  // Sched - Intake
                                                                12.32,  // Sched - PreScore
                                                                12.32, // Sched - Score
                                                                0.0,  // Sched - Trade_Off
                                                                0.0, // Sched - iAssist
                                                                7.40};// Sched - Source

/* KaDJ_Amp_Deg_WristDb: Sets Wrist dead band. */
const double KaDJ_Amp_Deg_WristDb[E_DJ_Amp_State_Sz] = {2.0,  // Sched - Init
                                                        2.0,  // Sched - Driving
                                                        2.0,  // Sched - Intake
                                                        2.0,  // Sched - PreScore
                                                        2.0, // Sched - Score
                                                        2.0,  // Sched - Trade_off
                                                        2.0,  // Sched - iAssist
                                                        2.0}; // Sched - Source

/* KaDJ_Amp_In_ElevatorDb: Sets Elevator dead band. */
const double KaDJ_Amp_In_ElevatorDb[E_DJ_Amp_State_Sz] = {1.0,  // Sched - Init
                                                          1.0,  // Sched - Driving
                                                          1.0,  // Sched - Intake
                                                          1.0,  // Sched - PreScore
                                                          1.0,  // Sched - Score
                                                          1.0,  // Sched - Trade_Off
                                                          1.0,  // Sched - iAssist
                                                          1.0}; // Sched - Source

/* KaMAN_InS_LinearSlideRate: Table that contains the linear slide transition rate. */
const double KaDJ_Amp_InS_ElevatorRate[E_DJ_Amp_State_Sz][E_DJ_Amp_State_Sz] = // [Cmnd][Attnd]
    {{1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1},
     {1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1},
     {1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1},
     {1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1},
     {1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1},
     {1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1},
     {1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1},
     {1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1, 1.1}};

const double KaDJ_Amp_DegS_WristRate[E_DJ_Amp_State_Sz][E_DJ_Amp_State_Sz] = // [Cmnd][Attnd]
    {{0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12},
     {0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12},
     {0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12},
     {0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12},
     {0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12},
     {0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12},
     {0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12},
     {0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12, 0.12}};

// Encoder / speed calculation related cals
/* KeENC_k_AMP_IntakeRatio: Reduction ratio for AMP intake. */
const double KeENC_k_AMP_IntakeRatio = 1.0;

/* KeENC_k_AMP_ElevatorRatio: Ratio to convert encoder counts to linear distance. */
const double KeENC_k_AMP_ElevatorRatio = - 0.1484992;

/* KeENC_k_AMP_ElevatorRatio: Ratio to convert encoder counts to angle. */
const double KeENC_k_AMP_WristRatio = 18.75; // 33.2103321   4.8  


/* KaSPK_e_ControllingTable: Table that contains the commanded state of the speaker based on the current attained state and schedueld state. */
const TeSPK_CtrlStates KaSPK_e_ControllingTable[E_SPK_Ctrl_StateSz][E_SPK_Ctrl_StateSz] = // [Sched][Attnd]
    {{E_SPK_Ctrl_Init,    E_SPK_Ctrl_Driving,  E_SPK_Ctrl_Intake,  E_SPK_Ctrl_Driving,  E_SPK_Ctrl_Driving},
     {E_SPK_Ctrl_Driving, E_SPK_Ctrl_Driving,  E_SPK_Ctrl_Driving, E_SPK_Ctrl_Driving,  E_SPK_Ctrl_Driving},
     {E_SPK_Ctrl_Driving, E_SPK_Ctrl_Intake,   E_SPK_Ctrl_Intake,  E_SPK_Ctrl_Driving,  E_SPK_Ctrl_Driving},
     {E_SPK_Ctrl_Driving, E_SPK_Ctrl_PreScore, E_SPK_Ctrl_Driving, E_SPK_Ctrl_PreScore, E_SPK_Ctrl_PreScore},
     {E_SPK_Ctrl_Driving, E_SPK_Ctrl_PreScore, E_SPK_Ctrl_Driving, E_SPK_Ctrl_Score,    E_SPK_Ctrl_Score}};

/* KaSPK_k_IntakePID_Gx: PID gains for the intake control. */
const double KaSPK_k_IntakePID_Gx[E_PID_SparkMaxCalSz] = { 0.1,      // kP
                                                           0.000001, // kI
                                                           0.002000, // kD
                                                           0.0,      // kIz
                                                           0.0,      // kFF
                                                           1.0,      // kMaxOut
                                                          -1.0,      // kMinOut
                                                           1.05,     // kMaxVel
                                                           0.5,      // kMinVel
                                                           0.0,      // kMaxAcc
                                                           0.0};     // kAllErr

/* KaSPK_k_IAssistPID_Gx: PID gains for the IAssist control. */
const double KaSPK_k_IAssistPID_Gx[E_PID_SparkMaxCalSz] = { 0.1,      // kP
                                                           0.000001, // kI
                                                           0.002000, // kD
                                                           0.0,      // kIz
                                                           0.0,      // kFF
                                                           1.0,      // kMaxOut
                                                          -1.0,      // kMinOut
                                                           1.05,     // kMaxVel
                                                           0.5,      // kMinVel
                                                           0.0,      // kMaxAcc
                                                           0.0};     // kAllErr

/* KaSPK_k_Shooter1PID_Gx: PID gains for the shooter 1 control. */
const double KaSPK_k_Shooter1PID_Gx[E_PID_SparkMaxCalSz] = { 0.0002,      // kP
                                                             0.000001, // kI
                                                             0.001000, // kD
                                                             0.0,      // kIz
                                                             0.0,      // kFF
                                                             1.0,      // kMaxOut
                                                            -1.0,      // kMinOut
                                                             1.05,     // kMaxVel
                                                             0.5,      // kMinVel
                                                             0.0,      // kMaxAcc
                                                             0.0};     // kAllErr

/* KaSPK_k_Shooter2PID_Gx: PID gains for the shooter 2 control. */
const double KaSPK_k_Shooter2PID_Gx[E_PID_SparkMaxCalSz] = { 0.0002,      // kP
                                                             0.000001, // kI
                                                             0.001000, // kD
                                                             0.0,      // kIz
                                                             0.0,      // kFF
                                                             1.0,      // kMaxOut
                                                            -1.0,      // kMinOut
                                                             1.05,     // kMaxVel
                                                             0.5,      // kMinVel
                                                             0.0,      // kMaxAcc
                                                             0.0};     // kAllErr

/* KeSPK_RPMs_Shooter1Rate: Table that contains the shooter 1 ramp rate. */
const double KeSPK_RPMs_Shooter1Rate = 75.0;

/* KeSPK_RPMs_Shooter2Rate: Table that contains the shooter 2 ramp rate. */
const double KeSPK_RPMs_Shooter2Rate = 75.0;

/* KeSPK_I_ShooterCurrentLimit: Max allowed current going to shooter motor. This is shared between 1 & 2 */
const double KeSPK_I_ShooterCurrentLimit = 30;

/* KeSPK_I_IntakeCurrentLimit: Max allowed current going to intake motor. */
const double KeSPK_I_IntakeCurrentLimit = 50;

/* KeSPK_I_IAssistCurrentLimit: Max allowed current going to IAssist motor. */
const double KeSPK_I_IAssistCurrentLimit = 30;

/* KaSPK_k_Intake: Sets intake final power for each state */
const double KaSPK_k_Intake[E_SPK_Ctrl_StateSz] = { 0.0,  // Sched - Init
                                                    0.0,  // Sched - Driving
                                                   -0.38,  // Sched - Intake  -.38
                                                    0.0,  // Sched - PreScore
                                                   -0.45}; // Sched - Score

/* KaSPK_k_IAssist: Sets IAssist final power for each state */
const double KaSPK_k_IAssist[E_SPK_Ctrl_StateSz] = { 0.0,  // Sched - Init
                                                     0.0,  // Sched - Driving
                                                    -0.55,  // Sched - Intake
                                                     0.0,  // Sched - PreScore
                                                     0.0}; // Sched - Score

/* KaSPK_RPM_Shooter1: Sets shooter 1 final speed for each state */
const double KaSPK_RPM_Shooter1[E_SPK_Ctrl_StateSz] = {   0.0,  // Sched - Init
                                                          0.0,  // Sched - Driving
                                                         -100.0,  // Sched - Intake
                                                          3000.0,  // Sched - PreScore 3000 1450
                                                          3000.0}; // Sched - Score 3000  1450

                                                          
// /* KaSPK_RPM_Shooter1: Sets shooter 1 final speed for each state */
// const double KaSPK_RPM_Shooter1[E_SPK_Ctrl_StateSz] = {   0.0,  // Sched - Init
//                                                           0.0,  // Sched - Driving
//                                                          -100.0,  // Sched - Intake
//                                                           1500.0,  // Sched - PreScore 3000 1450
//                                                           1500.0}; // Sched - Score 3000  1450

/* KaSPK_RPM_Shooter2: Sets shooter 2 final speed for each state */
const double KaSPK_RPM_Shooter2[E_SPK_Ctrl_StateSz] = {   0.0,  // Sched - Init
                                                          0.0,  // Sched - Driving
                                                         -100.0,  // Sched - Intake
                                                          3000.0,  // Sched - PreScore 3000 800
                                                          3000.0}; // Sched - Score 3000  800

/* KaSPK_RPM_Shooter1Db: Sets shooter 1 deadband */
const double KaSPK_RPM_Shooter1Db[E_SPK_Ctrl_StateSz] = {30.0,  // Sched - Init
                                                         3000.0,  // Sched - Driving
                                                         30.0,  // Sched - Intake
                                                         30.0,  // Sched - PreScore
                                                         30.0}; // Sched - Score

/* KaSPK_RPM_Shooter2Db: Sets shooter 2 deadband */
const double KaSPK_RPM_Shooter2Db[E_SPK_Ctrl_StateSz] = {30.0,  // Sched - Init
                                                         3000.0,  // Sched - Driving
                                                         30.0,  // Sched - Intake
                                                         30.0,  // Sched - PreScore
                                                         30.0}; // Sched - Score

/* KeSPK_t_StateTimeOut: Sets transition time out. */
const double KeSPK_t_StateTimeOut = 1.2;

/* KaSPK_k_TestPower: Test power output for the manipulator controls. ONLY used in test mode!! */
const double KaSPK_k_TestPower[E_SPK_m_Sz] = { 0.5, // E_SPK_m_Intake
                                               0.5, // E_SPK_m_IAssist
                                               0.5, // E_SPK_m_Shooter1
                                               0.5}; // E_SPK_m_Shooter2

/* KeENC_k_SPK_IntakeRatio: Ratio to convert encoder counts to intake. */
const double KeENC_k_SPK_IntakeRatio = 1.0;

/* KeENC_k_SPK_IAssistRatio: Ratio to convert encoder counts to IAssist. */
const double KeENC_k_SPK_IAssistRatio = 1.0;

/* KeENC_k_SPK_Shooter1Ratio: Ratio to convert shooter 1 encoder counts to RPM. */
const double KeENC_k_SPK_Shooter1Ratio = 1.0;

/* KeENC_k_SPK_Shooter2Ratio: Ratio to convert shooter 1 encoder counts to RPM. */
const double KeENC_k_SPK_Shooter2Ratio = 1.0;


/* KaCLMR_e_ControllingTable: Table that contains the commanded state of the climber based on the current attained state and schedueld state. */
const TeCLMR_CtrlStates KaCLMR_e_ControllingTable[E_CLMR_Ctrl_StateSz][E_CLMR_Ctrl_StateSz] = // [Sched][Attnd]
    {{E_CLMR_Ctrl_Init,       E_CLMR_Ctrl_Init,       E_CLMR_Ctrl_Init},
     {E_CLMR_Ctrl_MidClimb,   E_CLMR_Ctrl_MidClimb,   E_CLMR_Ctrl_MidClimb},
     {E_CLMR_Ctrl_FullExtend, E_CLMR_Ctrl_FullExtend, E_CLMR_Ctrl_FullExtend}};

/* KaCLMR_k_LeftPID_Gx: PID gains for the left control. */
const double KaCLMR_k_LeftPID_Gx[E_PID_SparkMaxCalSz] = { 0.1,      // kP
                                                          0.000001, // kI
                                                          0.002000, // kD
                                                          0.0,      // kIz
                                                          0.0,      // kFF
                                                          1.0,      // kMaxOut
                                                         -1.0,      // kMinOut
                                                          1.05,     // kMaxVel
                                                          0.5,      // kMinVel
                                                          0.0,      // kMaxAcc
                                                          0.0};     // kAllErr

/* KaCLMR_k_RightPID_Gx: PID gains for the right control. */
const double KaCLMR_k_RightPID_Gx[E_PID_SparkMaxCalSz] = { 0.1,      // kP
                                                           0.000001, // kI
                                                           0.002000, // kD
                                                           0.0,      // kIz
                                                           0.0,      // kFF
                                                           1.0,      // kMaxOut
                                                          -1.0,      // kMinOut
                                                           1.05,     // kMaxVel
                                                           0.5,      // kMinVel
                                                           0.0,      // kMaxAcc
                                                           0.0};     // kAllErr

/* KeCLMR_ins_LiftRate: Rate at which the arms will pull the robot up. */
const double KeCLMR_ins_LiftRate = 0.4;

/* KeCLMR_ins_ExtendRate: Rate at which the arms will extend up. */
const double KeCLMR_ins_ExtendRate = 0.8;

/*KeCLMR_Pct_CntrlDb: Deadband for manual control of the climber hooks. */
const double KeCLMR_Pct_CntrlDb = 0.05;

/*KeCLMR_k_CntrlGx: Rate control for manual/mid climb control. */
const double KeCLMR_k_CntrlGx = 0.04;

/* KeCLMR_I_ClimberCurrentLimit: Max allowed current going to climber motors. */
const double KeCLMR_I_ClimberCurrentLimit = 60;

/* KaCLMR_in_Position: Sets climber position for each state */
const double KaCLMR_in_Position[E_SPK_Ctrl_StateSz] = {   0.0,  // Sched - E_CLMR_Ctrl_Init
                                                          0.0,  // Sched - E_CLMR_Ctrl_MidClimb
                                                         13.50}; // Sched - E_CLMR_Ctrl_FullExtend

/* KaCLMR_in_LeftDb: Sets left deadband */
const double KaCLMR_in_LeftDb[E_SPK_Ctrl_StateSz] = {2.0,  // Sched - E_CLMR_Ctrl_Init
                                                     2.0,  // Sched - E_CLMR_Ctrl_MidClimb
                                                     2.0}; // Sched - E_CLMR_Ctrl_FullExtend

/* KaCLMR_in_RightDb: Sets right deadband */
const double KaCLMR_in_RightDb[E_SPK_Ctrl_StateSz] = {2.0,  // Sched - E_CLMR_Ctrl_Init
                                                      2.0,  // Sched - E_CLMR_Ctrl_MidClimb
                                                      2.0}; // Sched - E_CLMR_Ctrl_FullExtend

/* KeCLMR_t_StateTimeOut: Sets transition time out. */
const double KeCLMR_t_StateTimeOut = 30.0;

/* KaCLMR_k_TestPower: Test power output for the manipulator controls. ONLY used in test mode!! */
const double KaCLMR_k_TestPower[E_CLMR_m_Sz] = { 0.1, // E_CLMR_m_Left
                                                 0.1}; // E_CLMR_m_Right

/* KeENC_k_AMP_ElevatorRatio: Ratio to convert encoder counts to linear distance. */
const double KeENC_k_CLMR_RightRatio = 0.251497;

/* KeENC_k_CLMR_LeftRatio: Ratio to convert encoder counts to linear distance. */
const double KeENC_k_CLMR_LeftRatio = -0.251497;

/* KeSPK_t_ShooterOnTm: Amount of time Speaker Shooter will remain on after it is initially commanded on. */
const double KeSPK_t_ShooterOnTm = 0.08; //NOTE - set calibration time






/////////////////*Vision configs*//////////////////////
const double KeVIS_AmbiguityThreshold = 0.2;
const double KeVIS_in_MaxY = 323.25;
const double KeVIS_in_MaxX = 651.25;
const double KeVIS_t_VisionTimeout = 6.0;
const double KeVis_dIn_DeltaThreshold = 1.0;

/////////////////*Game Piece Coords*//////////////////////
const std::pair C_in_RedSpeakerCoords = std::make_pair(616.6309, 218.5039); // takes you right to the edge of the speaker ramp
const std::pair C_in_BlueSpeakerCoords = std::make_pair(34.550197, 218.5039);
const std::pair C_in_RedAmpCoords = std::make_pair(578.7402, 322.8346); // puts you directly against the amp, add an offset if you need to be back a bit
const std::pair C_in_BlueAmpCoords = std::make_pair(72.44094, 322.8346);