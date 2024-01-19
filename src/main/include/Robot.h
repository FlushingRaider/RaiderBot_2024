#ifndef ROBOT
#define ROBOT
#define PID_Calibrate

#pragma once

#include <string>

#include <frc/TimedRobot.h>

//NOTE gonna need to go through and add some of these libraries 
#include <frc/shuffleboard/Shuffleboard.h>
// #include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include <frc/motorcontrol/Spark.h>

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
