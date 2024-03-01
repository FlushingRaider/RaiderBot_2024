/*
 * Team 5561 2024 Code
 *
 *  This code will run our 2024 robot
 *
 */

#include "Robot.h"

#include <fmt/core.h>
#include <frc/DriverStation.h>
#include <cameraserver/CameraServer.h>

#include "Encoders.hpp"
#include "Gyro.hpp"
#include "DriveControl.hpp"
#include "Driver_inputs.hpp"
#include "ADAS.hpp"
#include "Odometry.hpp"
#include "Amp.hpp"
#include "SpeakerCntrl.hpp"
#include "Climber.hpp"
#include "ADAS_DJ.hpp"
#include "DataLogger.hpp"
#include "Vision.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

#include <units/angle.h>

T_RobotState VeROBO_e_RobotState = E_Init;
std::optional<frc::DriverStation::Alliance> VeROBO_e_AllianceColor;
double VeROBO_t_MatchTimeRemaining = 0;
bool VeROBO_b_TestState = false;


/******************************************************************************
 * Function:     RobotMotorCommands
 *
 * Description:  Contains the outputs for the motors.
 *               Run at the end of periodic functions to load calculated data in
 ******************************************************************************/
void Robot::RobotMotorCommands()
{
  // Motor output commands:
  // Swerve drive motors
  // Swerve stear motors
  if (VeDRC_b_DriveWheelsInPID == true)
  {
    m_frontLeftDrivePID.SetReference(VaDRC_RPM_WheelSpeedCmnd[E_FrontLeft], rev::CANSparkMax::ControlType::kVelocity); // rev::ControlType::kVelocity
    m_frontRightDrivePID.SetReference(VaDRC_RPM_WheelSpeedCmnd[E_FrontRight], rev::CANSparkMax::ControlType::kVelocity);
    m_rearLeftDrivePID.SetReference(VaDRC_RPM_WheelSpeedCmnd[E_RearLeft], rev::CANSparkMax::ControlType::kVelocity);
    m_rearRightDrivePID.SetReference(VaDRC_RPM_WheelSpeedCmnd[E_RearRight], rev::CANSparkMax::ControlType::kVelocity);
    m_frontLeftSteerMotor.Set(VaDRC_Pct_WheelAngleCmnd[E_FrontLeft]);
    m_frontRightSteerMotor.Set(VaDRC_Pct_WheelAngleCmnd[E_FrontRight]);
    m_rearLeftSteerMotor.Set(VaDRC_Pct_WheelAngleCmnd[E_RearLeft]);
    m_rearRightSteerMotor.Set(VaDRC_Pct_WheelAngleCmnd[E_RearRight]);
  }
  else
  {
    m_frontLeftDriveMotor.Set(0);
    m_frontRightDriveMotor.Set(0);
    m_rearLeftDriveMotor.Set(0);
    m_rearRightDriveMotor.Set(0);
    m_frontLeftSteerMotor.Set(0);
    m_frontRightSteerMotor.Set(0);
    m_rearLeftSteerMotor.Set(0);
    m_rearRightSteerMotor.Set(0);
  }

#ifdef Bot2024
  m_ElevatorPID.SetReference(VsAmp_s_Motors.k_MotorCmnd[E_Amp_Elevator], rev::ControlType::kPosition);
  m_Intake.Set(VsAmp_s_Motors.k_MotorCmnd[E_Amp_Intake]);
  m_Underbelly.Set(VsSPK_s_Motors.k_MotorCmnd[E_SPK_m_Intake]);
  m_Shooter1PID.SetReference(VsSPK_s_Motors.k_MotorCmnd[E_SPK_m_Shooter1], rev::ControlType::kVelocity);
  m_Shooter2PID.SetReference(VsSPK_s_Motors.k_MotorCmnd[E_SPK_m_Shooter2], rev::ControlType::kVelocity);
  m_ClimberLeftPID.SetReference(VsCLMR_s_Motors.k_MotorCmnd[E_CLMR_m_Left], rev::ControlType::kPosition);
  m_ClimberRightPID.SetReference(VsCLMR_s_Motors.k_MotorCmnd[E_CLMR_m_Right], rev::ControlType::kPosition);

  if (VsAmp_s_Motors.e_MotorControlType[E_Amp_Wrist] = E_MotorControlPctCmnd)
  {
    m_Wrist.Set(VsAmp_s_Motors.k_MotorCmnd[E_Amp_Wrist]);
  }
  else
  {
    m_WristPID.SetReference(VsAmp_s_Motors.k_MotorCmnd[E_Amp_Wrist], rev::ControlType::kPosition); 
  }
#else
  m_Intake.Set(0.0);
  m_Wrist.Set(0.0);
  m_Underbelly.Set(0.0);
  m_LinearSlide.Set(0.0);
#endif
}

/******************************************************************************
 * Function:     RobotInit
 *
 * Description:  Called during initialization of the robot.
 ******************************************************************************/
void Robot::RobotInit()
{
  DataLogRobotInit();

  // Default to a length of 60, start empty output
  // Length is expensive to set, so only set it once, then just update data
  // m_led.SetLength(kLength);
  // m_led.SetData(m_ledBuffer);
  // m_led.Start();

  EncodersInitSwerve(m_encoderFrontRightSteer,
                     m_encoderFrontLeftSteer,
                     m_encoderRearRightSteer,
                     m_encoderRearLeftSteer,
                     m_encoderFrontRightDrive,
                     m_encoderFrontLeftDrive,
                     m_encoderRearRightDrive,
                     m_encoderRearLeftDrive);


  frc::CameraServer::StartAutomaticCapture();

  GyroInit();

  VisionInit();

  m_frontLeftSteerMotor.SetSmartCurrentLimit(K_SD_SteerMotorCurrentLimit);
  m_frontRightSteerMotor.SetSmartCurrentLimit(K_SD_SteerMotorCurrentLimit);
  m_rearLeftSteerMotor.SetSmartCurrentLimit(K_SD_SteerMotorCurrentLimit);
  m_rearRightSteerMotor.SetSmartCurrentLimit(K_SD_SteerMotorCurrentLimit);

  m_frontLeftSteerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_frontLeftDriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_frontRightSteerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_frontRightDriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rearLeftSteerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rearLeftDriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rearRightSteerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_rearRightDriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_Wrist.SetSmartCurrentLimit(KeSPK_I_WristCurrentLimit);

  m_Elevator.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_ClimberLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_ClimberRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_Wrist.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_Intake.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_Underbelly.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_Shooter1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_Shooter2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  m_WristreverseLimit.EnableLimitSwitch(false);

  SwerveDriveMotorConfigsInit(m_frontLeftDrivePID,
                              m_frontRightDrivePID,
                              m_rearLeftDrivePID,
                              m_rearRightDrivePID);

  ADAS_Main_Init();
  ADAS_Main_Reset();
#ifdef Bot2024 
  Amp_MotorConfigsInit(m_ElevatorPID,
                       m_WristPID,
                       m_IntakePID);

  SPK_MotorConfigsInit(m_UnderbellyPID,
                       m_Shooter1PID,
                       m_Shooter2PID);

  CLMR_MotorConfigsInit(m_ClimberLeftPID,
                        m_ClimberRightPID);

  ADAS_DJ_Reset();

  Amp_ControlInit();
  SPK_ControlInit();
  CLMR_ControlInit();

  Encoders_AMP_SPK_CLMR_Init(m_encoderElevator,
                             m_encoderClimberLeft,
                             m_encoderClimberRight,
                             m_encoderWrist,
                             m_encoderIntake,
                             m_encoderUnderbelly,
                             m_encoderShooter1,
                             m_encoderShooter2);
#endif
}


/******************************************************************************
 * Function:     RobotPeriodic
 *
 * Description:  This function is called every 20 ms, no matter the mode. Use
 *               this for items like diagnostics that you want ran during disabled,
 *               autonomous, teleoperated and test.
 *
 *               <p> This runs after the mode specific periodic functions, but before
 *               LiveWindow and SmartDashboard integrated updating.
 ******************************************************************************/
void Robot::RobotPeriodic()
{
  // Fill the buffer with a rainbow
  // Rainbow();
  // // Set the LED
  // m_led.SetData(m_ledBuffer);

  VeROBO_t_MatchTimeRemaining = frc::Timer::GetMatchTime().value();

  Joystick1_robot_mapping(c_joyStick.GetRawButton(7),
                          c_joyStick.GetRawButton(8),
                          c_joyStick.GetRawAxis(1),
                          c_joyStick.GetRawAxis(0),
                          c_joyStick.GetRawAxis(4),
                          c_joyStick.GetRawAxis(3),
                          c_joyStick.GetRawButton(1),
                          c_joyStick.GetRawButton(3),
                          c_joyStick.GetRawButton(4),
                          c_joyStick.GetRawButton(6),
                          c_joyStick.GetRawButton(2),
                          c_joyStick.GetRawButton(5),
                          c_joyStick.GetPOV());

#ifdef Bot2024
  Joystick2_robot_mapping(c_joyStick2.GetRawButton(1),
                          c_joyStick2.GetRawButton(2),
                          c_joyStick2.GetRawButton(6),
                          c_joyStick2.GetRawButton(5),
                          c_joyStick2.GetRawButton(8),
                          c_joyStick2.GetRawButton(3),
                          c_joyStick2.GetRawButton(4),
                          c_joyStick2.GetRawAxis(1),
                          c_joyStick2.GetRawAxis(4),
                          c_joyStick2.GetPOV(),
                          c_joyStick2.GetRawButton(7),
                          c_joyStick2.GetRawAxis(2),
                          c_joyStick2.GetRawAxis(3));
#endif

  Encoders_Drive_CompBot(m_encoderWheelAngleCAN_FL.GetAbsolutePosition().GetValue(),
                         m_encoderWheelAngleCAN_FR.GetAbsolutePosition().GetValue(),
                         m_encoderWheelAngleCAN_RL.GetAbsolutePosition().GetValue(),
                         m_encoderWheelAngleCAN_RR.GetAbsolutePosition().GetValue(),
                         m_encoderFrontLeftDrive,
                         m_encoderFrontRightDrive,
                         m_encoderRearLeftDrive,
                         m_encoderRearRightDrive);

  ReadGyro2(VsCONT_s_DriverInput.b_ZeroGyro);

  DtrmnSwerveBotLocation(VeGRY_Rad_GyroYawAngleRad,
                         &VaENC_Rad_WheelAngleFwd[0],
                         &VaENC_In_WheelDeltaDistance[0],
                         VsCONT_s_DriverInput.b_ZeroGyro);




  VisionRun();

  ADAS_DetermineMode();

  frc::SmartDashboard::PutNumber("adas state", float(VeADAS_e_ActiveFeature));

  VeADAS_e_ActiveFeature = ADAS_ControlMain(&VeADAS_Pct_SD_FwdRev,
                                            &VeADAS_Pct_SD_Strafe,
                                            &VeADAS_Pct_SD_Rotate,
                                            &VeADAS_Deg_SD_DesiredPose,
                                            &VeADAS_b_SD_RobotOriented,
                                            &VeADAS_b_X_Mode,
                                            VsCONT_s_DriverInput.b_JoystickActive,
                                            VsCONT_s_DriverInput.b_SwerveGoalAutoCenter,
                                            VeGRY_Deg_GyroYawAngleDegrees,
                                            VeODO_In_RobotDisplacementX,
                                            VeODO_In_RobotDisplacementY,
                                            VeROBO_e_RobotState,
                                            VeADAS_e_ActiveFeature,
                                            VeROBO_e_AllianceColor);

  DriveControlMain(VsCONT_s_DriverInput.pct_SwerveForwardBack, // swerve control forward/back
                   VsCONT_s_DriverInput.pct_SwerveStrafe,      // swerve control strafe
                   VsCONT_s_DriverInput.deg_SwerveRotate,      // rotate the robot joystick
                   VsCONT_s_DriverInput.v_SwerveSpeed,         // extra speed trigger
                   VsCONT_s_DriverInput.b_SwerveRotateTo0,     // auto rotate to 0 degrees
                   VsCONT_s_DriverInput.b_SwerveRotateTo180,   // auto rotate to 180 degrees
                   VsCONT_s_DriverInput.b_ZeroGyro,
                   VeADAS_b_X_Mode, // X mode req from ADAS
                   VsCONT_s_DriverInput.b_X_Mode,
                   VeROBO_e_RobotState,
                   VeADAS_e_ActiveFeature,
                   VeADAS_Pct_SD_FwdRev, // similar to driver input but used for auton
                   VeADAS_Pct_SD_Strafe,
                   VeADAS_Pct_SD_Rotate,
                   VeADAS_Deg_SD_DesiredPose,
                   VeADAS_b_SD_RobotOriented,
                   VeGRY_Deg_GyroYawAngleDegrees,
                   VeGRY_Rad_GyroYawAngleRad,
                   &VaENC_Deg_WheelAngleFwd[0],
                   &VaENC_Deg_WheelAngleRev[0],
                   &VaDRC_RPM_WheelSpeedCmnd[0],
                   &VaDRC_Pct_WheelAngleCmnd[0]);

  log_swerve_WheelAngleFwd.Append(VaENC_Deg_WheelAngleFwd);
  log_swerve_WheelAngleRev.Append(VaENC_Deg_WheelAngleRev);
  log_swerve_WheelSpeedCmnd.Append(VaDRC_RPM_WheelSpeedCmnd);
  log_swerve_WheelAngleCmnd.Append(VaDRC_Pct_WheelAngleCmnd);

  // frc::SmartDashboard::PutNumberArray("swerve_WheelAngleFwd", VaENC_Deg_WheelAngleFwd);
  // frc::SmartDashboard::PutNumberArray("swerve_WheelAngleRev", VaENC_Deg_WheelAngleRev);
  // frc::SmartDashboard::PutNumberArray("swerve_WheelSpeedCmnd", VaDRC_RPM_WheelSpeedCmnd);
  // frc::SmartDashboard::PutNumberArray("swerve_WheelAngleCmnd", VaDRC_Pct_WheelAngleCmnd);

  frc::SmartDashboard::PutNumber("GyroYaw", -VeGRY_Deg_GyroYawAngleDegrees);

#ifdef Bot2024
  Encoders_AMP_SPK_CLMR_Run(m_WristreverseLimit.Get(),
                            m_ElevatorLimitSwitch.Get(),
                            breakbeam_shooter.Get(),
                            m_encoderElevator,
                            m_encoderClimberLeft,
                            m_encoderClimberRight,
                            m_encoderWrist,
                            m_encoderIntake,
                            m_encoderUnderbelly,
                            m_encoderShooter1,
                            m_encoderShooter2);

  Amp_MotorConfigsCal(m_ElevatorPID,
                      m_WristPID,
                      m_IntakePID);

  SPK_MotorConfigsCal(m_UnderbellyPID,
                      m_Shooter1PID,
                      m_Shooter2PID);

  CLMR_MotorConfigsCal(m_ClimberLeftPID,
                       m_ClimberRightPID);
  
  Amp_ControlMain(VeADAS_e_Amp_SchedState,
                  VeROBO_b_TestState);

  SPK_SpeakerControlMain(VeADAS_e_SPK_SchedState,
                         VeROBO_b_TestState);

  CLMR_SpeakerControlMain(VeADAS_e_CLMR_SchedState,
                          VeROBO_b_TestState);

  Encoders_AMP_ResetWrist(m_encoderWrist,
                          VeAmp_b_WristEncoderReset);
#endif
}


/******************************************************************************
 * Function:     AutonomousInit
 *
 * Description:  This autonomous (along with the chooser code above) shows how to select
 *               between different autonomous modes using the dashboard. The sendable chooser
 *               code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 *               remove all of the chooser code and uncomment the GetString line to get the
 *               auto name from the text box below the Gyro.
 *
 *               You can add additional auto modes by adding additional comparisons to the
 *               if-else structure below with additional strings. If using the SendableChooser
 *               make sure to add them to the chooser code above as well.
 ******************************************************************************/
void Robot::AutonomousInit()
{

  VeROBO_e_RobotState = E_Auton;
  VeROBO_e_AllianceColor = frc::DriverStation::GetAlliance();

  //GyroInit();

  DriveControlInit();

  OdometryInit();

  ADAS_Main_Reset();
  #ifdef Bot2024
  if (VeROBO_b_TestState == true)
  {
    /* If we were in "test state", the motors may have been moved from there initialized position,
       we need to rezero all of the encoders/sensors */
    ADAS_DJ_Reset();
    Amp_ControlInit();
    SPK_ControlInit();
    CLMR_ControlInit();
  }
  #endif

  VeROBO_b_TestState = false;
}


/******************************************************************************
 * Function:     AutonomousPeriodic
 *
 * Description:  Function called periodically in autonomous.  This is where we
 *               should place our primary autonomous control code.
 ******************************************************************************/
void Robot::AutonomousPeriodic()
{
  RobotMotorCommands();
}


/******************************************************************************
 * Function:     TeleopInit
 *
 * Description:  Function called when starting out in teleop mode.
 *               We should zero out all of our global varibles.
 ******************************************************************************/
void Robot::TeleopInit()
{
  VeROBO_e_RobotState = E_Teleop;
  VeROBO_e_AllianceColor = frc::DriverStation::GetAlliance();
  ADAS_Main_Reset();
  DriveControlInit();
  OdometryInit();
#ifdef Bot2024
  if (VeROBO_b_TestState == true)
  {
    /* If we were in "test state", the motors may have been moved from there initialized position,
       we need to rezero all of the encoders/sensors */
    ADAS_DJ_Reset();
    Amp_ControlInit();
    SPK_ControlInit();
    CLMR_ControlInit();
  }
  #endif
  VeROBO_b_TestState = false;
}


/******************************************************************************
 * Function:     TeleopPeriodic
 *
 * Description:  Primary function called when in teleop mode.
 ******************************************************************************/
void Robot::TeleopPeriodic()
{
  RobotMotorCommands();
}


void Robot::DisabledInit() {}


/******************************************************************************
 * Function:     DisabledPeriodic
 *
 * Description:  Called during the disabled phase initiated on the driver station.
 ******************************************************************************/
void Robot::DisabledPeriodic()
{

}

void Robot::TestInit() {}


/******************************************************************************
 * Function:     TestPeriodic
 *
 * Description:  Called during the test phase initiated on the driver station.
 ******************************************************************************/
void Robot::TestPeriodic()
{
  VeROBO_b_TestState = true;

  #ifdef Bot2024
  Amp_ControlManualOverride(&VsCONT_s_DriverInput);
  SPK_ControlManualOverride(&VsCONT_s_DriverInput);
  CLMR_ControlManualOverride(&VsCONT_s_DriverInput);
  #endif

  if (VsCONT_s_DriverInput.b_ResetEnocders == true)
    {
      EncodersInitSwerve(m_encoderFrontRightSteer,
                         m_encoderFrontLeftSteer,
                         m_encoderRearRightSteer,
                         m_encoderRearLeftSteer,
                         m_encoderFrontRightDrive,
                         m_encoderFrontLeftDrive,
                         m_encoderRearRightDrive,
                         m_encoderRearLeftDrive);
    #ifdef Bot2024
    Encoders_AMP_SPK_CLMR_Init(m_encoderElevator,
                               m_encoderClimberLeft,
                               m_encoderClimberRight,
                               m_encoderWrist,
                               m_encoderIntake,
                               m_encoderUnderbelly,
                               m_encoderShooter1,
                               m_encoderShooter2);
    #endif
    }

  m_frontLeftDriveMotor.Set(0);
  m_frontRightDriveMotor.Set(0);
  m_rearLeftDriveMotor.Set(0);
  m_rearRightDriveMotor.Set(0);

  m_frontLeftSteerMotor.Set(0);
  m_frontRightSteerMotor.Set(0);
  m_rearLeftSteerMotor.Set(0);
  m_rearRightSteerMotor.Set(0);

#ifdef Bot2024
  m_Underbelly.Set(VsSPK_s_Motors.k_MotorTestPower[E_SPK_m_Intake]);
  m_Shooter1.Set(VsSPK_s_Motors.k_MotorTestPower[E_SPK_m_Shooter1]);
  m_Shooter2.Set(VsSPK_s_Motors.k_MotorTestPower[E_SPK_m_Shooter2]);
  m_Elevator.Set(VsAmp_s_Motors.k_MotorTestPower[E_Amp_Elevator]);
  m_Wrist.Set(VsAmp_s_Motors.k_MotorTestPower[E_Amp_Wrist]);
  m_Intake.Set(VsAmp_s_Motors.k_MotorTestPower[E_Amp_Intake]);
  m_ClimberLeft.Set(VsCLMR_s_Motors.k_MotorTestPower[E_CLMR_m_Left]);
  m_ClimberRight.Set(VsCLMR_s_Motors.k_MotorTestPower[E_CLMR_m_Right]);
#endif
}


void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
/******************************************************************************
 * Function:     main
 *
 * Description:  This is the main calling function for the robot.
 ******************************************************************************/
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
