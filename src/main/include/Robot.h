#ifndef ROBOT
#define ROBOT
#define PID_Calibrate

#pragma once

#include <string>

#include <frc/TimedRobot.h>

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


// needed for 2023 bot compatibility
#include <ctre/Phoenix.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>



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
  //NOTE - its no longer SparkRelativeEncoder its now SparkRelativeEncoder
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


  rev::CANSparkMax                           m_Elevator            {KeDJ_Amp_i_Elevator,        rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_Wrist               {KeDJ_Amp_i_Wrist,           rev::CANSparkMax::MotorType::kBrushless};        
  rev::CANSparkMax                           m_Intake              {KeDJ_Amp_i_Intake,         rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_Underbelly          {KeDJ_Spk_i_Underbelly,           rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_Shooter1            {KeDJ_Spk_i_Shooter1,           rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_Shooter2            {KeDJ_Spk_i_Shooter2,           rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxPIDController                 m_ElevatorPID         = m_Elevator.GetPIDController();
  rev::SparkMaxPIDController                 m_WristPID            = m_Wrist.GetPIDController();
  rev::SparkMaxPIDController                 m_IntakePID           = m_Intake.GetPIDController();
  rev::SparkMaxPIDController                 m_UnderbellyPID       = m_Underbelly.GetPIDController();
  rev::SparkMaxPIDController                 m_Shooter1PID         = m_Shooter1.GetPIDController();
  rev::SparkMaxPIDController                 m_Shooter2PID         = m_Shooter2.GetPIDController();
  // rev::SparkMaxLimitSwitch                   m_WristforwardLimit   = m_Wrist.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
  //rev::SparkMaxLimitSwitch                   m_WristreverseLimit   = m_Wrist.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
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
};

  frc::ShuffleboardTab& FrontLeft = frc::Shuffleboard::GetTab("Front Left");
  frc::ShuffleboardTab& FrontRight = frc::Shuffleboard::GetTab("Front Right");
  frc::ShuffleboardTab& BackLeft = frc::Shuffleboard::GetTab("Back Left");
  frc::ShuffleboardTab& BackRight = frc::Shuffleboard::GetTab("Back Right");
#endif