#ifndef ROBOT
#define ROBOT
#define PID_Calibrate

#pragma once

#include <string>

#include <frc/TimedRobot.h>

// #include <frc/AddressableLED.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include "rev/CANSparkMax.h"
#include <frc/motorcontrol/Spark.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "Const.hpp"

#include <ctre/phoenix6/core/CoreCANcoder.hpp>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;
  void RobotMotorCommands();

  frc::ShuffleboardTab& FrontLeft = frc::Shuffleboard::GetTab("Front Left");
  frc::ShuffleboardTab& FrontRight = frc::Shuffleboard::GetTab("Front Right");
  frc::ShuffleboardTab& BackLeft = frc::Shuffleboard::GetTab("Back Left");
  frc::ShuffleboardTab& BackRight = frc::Shuffleboard::GetTab("Back Right");
  
  ctre::phoenix6::hardware::CANcoder          m_encoderWheelAngleCAN_FL     {KeEnc_i_WheelAngleFL, "rio"};
  ctre::phoenix6::hardware::CANcoder          m_encoderWheelAngleCAN_FR     {KeEnc_i_WheelAngleFR, "rio"};
  ctre::phoenix6::hardware::CANcoder          m_encoderWheelAngleCAN_RL     {KeEnc_i_WheelAngleRL, "rio"};
  ctre::phoenix6::hardware::CANcoder          m_encoderWheelAngleCAN_RR     {KeEnc_i_WheelAngleRR, "rio"};

  // PDP - Power Distribution Panel - CAN
  frc::PowerDistribution                     PDP                   {C_PDP_ID,               frc::PowerDistribution::ModuleType::kRev};

  // CAN Motor Controllers
  rev::CANSparkMax                           m_frontLeftSteerMotor {frontLeftSteerDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_frontLeftDriveMotor {frontLeftDriveDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_frontRightSteerMotor{frontRightSteerDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_frontRightDriveMotor{frontRightDriveDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_rearLeftSteerMotor  {rearLeftSteerDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_rearLeftDriveMotor  {rearLeftDriveDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_rearRightSteerMotor {rearRightSteerDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_rearRightDriveMotor {rearRightDriveDeviceID,  rev::CANSparkMax::MotorType::kBrushless};

  // CAN Encoders
  rev::SparkRelativeEncoder               m_encoderFrontLeftSteer  = m_frontLeftSteerMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_encoderFrontLeftDrive  = m_frontLeftDriveMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_encoderFrontRightSteer = m_frontRightSteerMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_encoderFrontRightDrive = m_frontRightDriveMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);

  rev::SparkRelativeEncoder               m_encoderRearLeftSteer   = m_rearLeftSteerMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_encoderRearLeftDrive   = m_rearLeftDriveMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_encoderRearRightSteer  = m_rearRightSteerMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_encoderRearRightDrive  = m_rearRightDriveMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);

  rev::SparkMaxPIDController                 m_frontLeftDrivePID    = m_frontLeftDriveMotor.GetPIDController();
  rev::SparkMaxPIDController                 m_frontRightDrivePID   = m_frontRightDriveMotor.GetPIDController();
  rev::SparkMaxPIDController                 m_rearLeftDrivePID     = m_rearLeftDriveMotor.GetPIDController();
  rev::SparkMaxPIDController                 m_rearRightDrivePID    = m_rearRightDriveMotor.GetPIDController();

#ifdef Bot2024
  rev::CANSparkMax                           m_Elevator            {KeDJ_Amp_i_Elevator,        rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_ClimberLeft         {KeCLMR_i_ClimberArm1,           rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_ClimberRight        {KeCLMR_i_ClimberArm2,           rev::CANSparkMax::MotorType::kBrushless};
#endif  
  rev::CANSparkMax                           m_Wrist               {KeDJ_Amp_i_Wrist,               rev::CANSparkMax::MotorType::kBrushless};        
  rev::CANSparkMax                           m_Intake              {KeDJ_Amp_i_Intake,              rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_Underbelly          {KeDJ_Spk_i_Underbelly,          rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_Shooter1            {KeDJ_Spk_i_Shooter1,            rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_Shooter2            {KeDJ_Spk_i_Shooter2,            rev::CANSparkMax::MotorType::kBrushless};
#ifdef Bot2024
  rev::SparkRelativeEncoder                  m_encoderElevator     = m_Elevator.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder                  m_encoderClimberLeft  = m_ClimberLeft.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder                  m_encoderClimberRight = m_ClimberRight.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder                  m_encoderWrist        = m_Wrist.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder                  m_encoderIntake       = m_Intake.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder                  m_encoderUnderbelly   = m_Underbelly.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder                  m_encoderShooter1     = m_Shooter1.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder                  m_encoderShooter2     = m_Shooter2.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);

  rev::SparkMaxPIDController                 m_ElevatorPID         = m_Elevator.GetPIDController();
  rev::SparkMaxPIDController                 m_ClimberLeftPID      = m_ClimberLeft.GetPIDController();
  rev::SparkMaxPIDController                 m_ClimberRightPID     = m_ClimberRight.GetPIDController();
#endif
  rev::SparkMaxPIDController                 m_WristPID            = m_Wrist.GetPIDController();
  rev::SparkMaxPIDController                 m_IntakePID           = m_Intake.GetPIDController();
  rev::SparkMaxPIDController                 m_UnderbellyPID       = m_Underbelly.GetPIDController();
  rev::SparkMaxPIDController                 m_Shooter1PID         = m_Shooter1.GetPIDController();
  rev::SparkMaxPIDController                 m_Shooter2PID         = m_Shooter2.GetPIDController();
  #ifdef Bot2024
  frc::DigitalInput                          breakbeam_shooter{KeSPK_i_BreakBeamIO};
  frc::DigitalInput                          m_ElevatorLimitSwitch{KeSPK_i_ElevatorSwitchIO};
  rev::SparkMaxLimitSwitch                   m_WristreverseLimit   = m_Intake.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
  #endif
  #ifdef Bot2023
  WPI_TalonSRX                               m_LinearSlide          {KeMAN_i_LinearSlide};
  #endif


    // Driver Inputs
  frc::Joystick c_joyStick{0};
  #ifdef Bot2024
  frc::Joystick c_joyStick2{1};
  #endif


 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

// // Light Controls
//     static constexpr int kLength = 60;

//   // PWM port 9
//   // Must be a PWM header!
//   frc::AddressableLED m_led{9};
//   std::array<frc::AddressableLED::LEDData, kLength>
//   m_ledBuffer;  // Reuse the buffer
//   // Store what the last hue of the first pixel is
//   int firstPixelHue = 0;

// void Robot::Rainbow() {
//   // For every pixel
//   for (int i = 0; i < kLength; i++) {
//     // Calculate the hue - hue is easier for rainbows because the color
//     // shape is a circle so only one value needs to precess
//     const auto pixelHue = (firstPixelHue + (i * 180 / kLength)) % 180;
//     // Set the value
//     m_ledBuffer[i].SetHSV(pixelHue, 255, 128);
//   }
//   // Increase by to make the rainbow "move"
//   firstPixelHue += 3;
//   // Check bounds
//   firstPixelHue %= 180;
// }

};

  frc::ShuffleboardTab& FrontLeft = frc::Shuffleboard::GetTab("Front Left");
  frc::ShuffleboardTab& FrontRight = frc::Shuffleboard::GetTab("Front Right");
  frc::ShuffleboardTab& BackLeft = frc::Shuffleboard::GetTab("Back Left");
  frc::ShuffleboardTab& BackRight = frc::Shuffleboard::GetTab("Back Right");
#endif