/*
 * Team 5561 2024 Code
 *
 *  This code will run our 2024 robot
 * 
 */

#include "Robot.h"

#include <fmt/core.h>
#include <frc/DriverStation.h>

#include "Encoders.hpp"
#include "Gyro.hpp"
#include "DriveControl.hpp"
#include "Driver_inputs.hpp"
#include "ADAS.hpp"
#include "Odometry.hpp"

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
  bool LeROBO_b_IntakeArmExtend = false;
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

  m_ArmPivot.Set(0.0);
  m_Wrist.Set(0.0);
  m_Gripper.Set(0.0); 
  m_LinearSlide.Set(0.0);




}

/******************************************************************************
 * Function:     RobotInit
 *
 * Description:  Called during initialization of the robot.
 ******************************************************************************/
void Robot::RobotInit()
{
  
  EncodersInitSwerve(m_encoderFrontRightSteer,
                     m_encoderFrontLeftSteer,
                     m_encoderRearRightSteer,
                     m_encoderRearLeftSteer,
                     m_encoderFrontRightDrive,
                     m_encoderFrontLeftDrive,
                     m_encoderRearRightDrive,
                     m_encoderRearLeftDrive);

  GyroInit();

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
  

    SwerveDriveMotorConfigsInit(m_frontLeftDrivePID,
                              m_frontRightDrivePID,
                              m_rearLeftDrivePID,
                              m_rearRightDrivePID);


  ADAS_Main_Init();
  ADAS_Main_Reset();

}


/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

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

#ifdef CompBot
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


  ADAS_DetermineMode();

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
                                            V_OdomCentered,
                                            VeROBO_e_AllianceColor,
                                            V_OffsetXOut,
                                            V_OffsetYOut,
                                            VeADAS_in_GlobalRequestX,
                                            VeADAS_in_GlobalRequestY,
                                            VeADAS_in_OffsetRequestX,
                                            VeADAS_in_OffsetRequestY);



  DriveControlMain(VsCONT_s_DriverInput.pct_SwerveForwardBack, // swerve control forward/back
                   VsCONT_s_DriverInput.pct_SwerveStrafe,      // swerve control strafe
                   VsCONT_s_DriverInput.deg_SwerveRotate,      // rotate the robot joystick
                   VsCONT_s_DriverInput.v_SwerveSpeed,         // extra speed trigger
                   VsCONT_s_DriverInput.b_SwerveRotateTo0,     // auto rotate to 0 degrees
                   VsCONT_s_DriverInput.b_SwerveRotateTo180,   // auto rotate to 180 degrees
                   VsCONT_s_DriverInput.b_ZeroGyro,
                   VeADAS_b_X_Mode, // X mode req from ADAS
                   VsCONT_s_DriverInput.b_X_Mode,
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
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {

  VeROBO_e_RobotState = E_Auton;
  VeROBO_e_AllianceColor = frc::DriverStation::GetAlliance();
  VeROBO_b_TestState = false;


  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  
  GyroInit();
  
  DriveControlInit();

  OdometryInit();

  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  // if (m_autoSelected == kAutoNameCustom) {
  //   // Custom Auto goes here
  // } else {
  //   // Default Auto goes here
  // }

  RobotMotorCommands();

}

void Robot::TeleopInit() {

  VeROBO_e_RobotState = E_Teleop;
  VeROBO_e_AllianceColor = frc::DriverStation::GetAlliance();
  VeROBO_b_TestState = false;

  DriveControlInit();

  OdometryInit();
}

void Robot::TeleopPeriodic() {

  RobotMotorCommands();

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
