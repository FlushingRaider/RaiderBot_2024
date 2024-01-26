#ifndef ROBOT
#define ROBOT
#define PID_Calibrate

#pragma once

#include <string>

#include <frc/TimedRobot.h>

#include <frc/shuffleboard/Shuffleboard.h>
// #include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>
//NOTE - phoenix isn't currently used but when it is needed phoenix6 is gonna be imported different
#include "rev/CANSparkMax.h"
#include <frc/motorcontrol/Spark.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "Const.hpp"

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

  frc::ShuffleboardTab& FrontLeft = frc::Shuffleboard::GetTab("Front Left");
  frc::ShuffleboardTab& FrontRight = frc::Shuffleboard::GetTab("Front Right");
  frc::ShuffleboardTab& BackLeft = frc::Shuffleboard::GetTab("Back Left");
  frc::ShuffleboardTab& BackRight = frc::Shuffleboard::GetTab("Back Right");
 
  

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
  //NOTE - its no longer SparkRelativeEncoder its now SparkRelativeEncoder
  rev::SparkRelativeEncoder               m_encoderFrontLeftSteer  = m_frontLeftSteerMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_encoderFrontLeftDrive  = m_frontLeftDriveMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_encoderFrontRightSteer = m_frontRightSteerMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_encoderFrontRightDrive = m_frontRightDriveMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);

  rev::SparkRelativeEncoder               m_encoderRearLeftSteer   = m_rearLeftSteerMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_encoderRearLeftDrive   = m_rearLeftDriveMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_encoderRearRightSteer  = m_rearRightSteerMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_encoderRearRightDrive  = m_rearRightDriveMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);


 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};

  frc::ShuffleboardTab& FrontLeft = frc::Shuffleboard::GetTab("Front Left");
  frc::ShuffleboardTab& FrontRight = frc::Shuffleboard::GetTab("Front Right");
  frc::ShuffleboardTab& BackLeft = frc::Shuffleboard::GetTab("Back Left");
  frc::ShuffleboardTab& BackRight = frc::Shuffleboard::GetTab("Back Right");
#endif