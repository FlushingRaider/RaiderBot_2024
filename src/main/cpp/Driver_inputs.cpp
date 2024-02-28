/*
  Driver_inputs.cpp

   Created on: Feb 05, 2022
   Author: Lauren

  Function that maps the driver inputs to the robot controls.
  This is where the "CONT" variables are set.  Everything gets mapped to the structure VsCONT_s_DriverInput.

  Imported to 2024: January 18, 2024
 */
#include "Enums.hpp"
#include <frc/DriverStation.h>
#include "Lookup.hpp"
#include <math.h>
#include "Const.hpp"

RobotUserInput VsCONT_s_DriverInput;

/******************************************************************************
 * Function:     Joystick1_robot_mapping
 *
 * Description:  Captures and maps driver inputs from controller 1.
 ******************************************************************************/
void Joystick1_robot_mapping(bool LeCONT_b_Driver1ButtonBack,
                             bool LeCONT_b_Driver1ButtonStart,
                             double LeCONT_Cmd_Driver1LeftAxisY,
                             double LeCONT_Cmd_Driver1LeftAxisX,
                             double LeCONT_Cmd_Driver1RightAxisX,
                             double LeCONT_Cmd_Driver1LeftTriggerAxis,
                             bool LeCONT_b_Driver1ButtonA,
                             bool LeCONT_b_Driver1ButtonX,
                             bool LeCONT_b_Driver1ButtonY, ///
                             bool LeCONT_b_Driver1ButtonRB,
                             bool LeCONT_b_Driver1ButtonB, ///
                             bool LeCONT_b_Driver1ButtonLB,
                             int LeCONT_Deg_Driver1POV)
{
  double LeCONT_Pct_AxisTotal = 0;
  bool LeCONT_b_JoystickActive = false;
  bool LeCONT_b_X_Mode = false;

  VsCONT_s_DriverInput.b_ZeroGyro = (LeCONT_b_Driver1ButtonBack || LeCONT_b_Driver1ButtonStart); // Controller 1, Back button (7), (robot.cpp, gyro.cpp) zeroes out the gyro
  VsCONT_s_DriverInput.pct_SwerveForwardBack = ScaleJoystickAxis(LeCONT_Cmd_Driver1LeftAxisY);   // Scale the axis, also used for debouncing
  VsCONT_s_DriverInput.pct_SwerveStrafe = ScaleJoystickAxis(LeCONT_Cmd_Driver1LeftAxisX);        // Scale the axis, also used for debouncing
  VsCONT_s_DriverInput.deg_SwerveRotate = ScaleJoystickRotateAxis(LeCONT_Cmd_Driver1RightAxisX); // Scale the axis, also used for debouncing
  VsCONT_s_DriverInput.v_SwerveSpeed = ScaleAccelAxis(LeCONT_Cmd_Driver1LeftTriggerAxis);        // Scale the axis, also used for debouncing
  // VsCONT_s_DriverInput.b_MidIntakeOut                  = LeCONT_b_Driver1ButtonX;
  VsCONT_s_DriverInput.b_SwerveRotateTo180 = LeCONT_b_Driver1ButtonY; // Auto align to 180 degrees
  VsCONT_s_DriverInput.b_SwerveRotateTo0 = LeCONT_b_Driver1ButtonB;   // Auto align to 0 degrees
  // VsCONT_s_DriverInput.b_ConeAlign                     = LeCONT_b_Driver1ButtonLB;   //Aligns the robot to score a cone
  VsCONT_s_DriverInput.b_goToSpeaker = LeCONT_b_Driver1ButtonA;
  LeCONT_Pct_AxisTotal = (fabs(VsCONT_s_DriverInput.pct_SwerveStrafe) + fabs(VsCONT_s_DriverInput.deg_SwerveRotate) + fabs(VsCONT_s_DriverInput.v_SwerveSpeed));

  if (LeCONT_Pct_AxisTotal > 0.1)
  {
    LeCONT_b_JoystickActive = true;
  }

  VsCONT_s_DriverInput.b_JoystickActive = LeCONT_b_JoystickActive;

  if (LeCONT_Deg_Driver1POV == 0)
  {
  }
  else if (LeCONT_Deg_Driver1POV == 180)
  {
    LeCONT_b_X_Mode = true;
  }
  else if (LeCONT_Deg_Driver1POV == 270)
  {
  }
  else if (LeCONT_Deg_Driver1POV == 90)
  {
  }
  VsCONT_s_DriverInput.b_X_Mode = LeCONT_b_X_Mode;
}

/******************************************************************************
 * Function:     Joystick2_robot_mapping
 *
 * Description:  Captures and maps driver inputs from controller 2.
 ******************************************************************************/
void Joystick2_robot_mapping(bool LeCONT_b_Driver2ButtonA,
                             bool LeCONT_b_Driver2ButtonB,
                             bool LeCONT_b_Driver2ButtonRB,
                             bool LeCONT_b_Driver2ButtonLB,
                             bool LeCONT_b_Driver2ButtonStart,
                             bool LeCONT_b_Driver2ButtonX,
                             bool LeCONT_b_Driver2ButtonY,
                             double LeCONT_Pct_Driver2LeftAxisY,
                             double LeCONT_Pct_Driver2RightAxisX,
                             int LeCONT_Deg_Driver2POV,
                             bool LeCONT_b_Driver2ButtonBack,
                             double LeCont_Pct_Driver2AxisRB,
                             double LeCont_Pct_Driver2AxisLB)
{
  double LeCONT_Pct_Amp_Wrist_Test = 0.0;
  double LeCONT_Pct_Amp_Elevator_Test = 0.0;
  double LeCONT_Pct_Amp_Intake_Test = 0.0;
  double LeCONT_Pct_CLMR_Left_Test = 0.0;
  double LeCONT_Pct_CLMR_Right_Test = 0.0;
  bool LeCONT_b_SPK_Driving = false;
  bool LeCONT_b_SPK_Intake = false;
  bool LeCONT_b_SPK_PreScore = false;
  bool LeCONT_b_SPK_Score = false;
  double LeCONT_Pct_Man_CLMR = 0.0;


  VsCONT_s_DriverInput.b_Amp_DrivingPosition         = LeCONT_b_Driver2ButtonB; // COMP - Controller 2, B button brings the Amp mechanism into driver position
  VsCONT_s_DriverInput.b_Spk_IntakeForward_Test      = LeCONT_b_Driver2ButtonY; // TEST - Controller 2, Y button tests the intake going foward so intaking a note
  VsCONT_s_DriverInput.b_Amp_Intake                  = LeCONT_b_Driver2ButtonY; // COMP - Controller 2, Y
  VsCONT_s_DriverInput.b_Spk_IntakeBackward_Test     = LeCONT_b_Driver2ButtonA; // Controller 2, A button (1), (robot.cpp) intake in TEST BUTTON
  VsCONT_s_DriverInput.b_Amp_PreScore                = LeCONT_b_Driver2ButtonA;
  VsCONT_s_DriverInput.b_Amp_Score                   = LeCONT_b_Driver2ButtonX;
  VsCONT_s_DriverInput.b_ResetEnocders               = LeCONT_b_Driver2ButtonStart; // controller 2 start button (8), (robot.cpp) Starts robot shooter speed based on distance
  // VsCONT_s_DriverInput.pct_RightHookDown_Test           = LeCont_Pct_Driver2AxisRB;
  // VsCONT_s_DriverInput.pct_LeftHookDown_Test            = LeCont_Pct_Driver2AxisLB;
  VsCONT_s_DriverInput.b_CLMR_Init               = LeCONT_b_Driver2ButtonRB;   // Boolean is being written to float
  VsCONT_s_DriverInput.b_CLMR_FullExtend             = LeCONT_b_Driver2ButtonLB; // Boolean is being written to float
  VsCONT_s_DriverInput.Pct_Shooter1_Test             = LeCONT_Pct_Driver2RightAxisX;
  VsCONT_s_DriverInput.Pct_Shooter2_Test             = LeCONT_Pct_Driver2LeftAxisY;

 /*Tests the climber hooks - in test mode*/
  if (LeCONT_b_Driver2ButtonB == true)
  {
    LeCONT_Pct_Amp_Intake_Test = 1;
  }
  else if (LeCONT_b_Driver2ButtonX == true)
  {
    LeCONT_Pct_Amp_Intake_Test = -1;
  }
  VsCONT_s_DriverInput.Pct_Amp_Intake_Test = LeCONT_Pct_Amp_Intake_Test;

  if (LeCONT_b_Driver2ButtonLB == true)
  {
    LeCONT_Pct_CLMR_Left_Test = -1;
  }
  else if (LeCont_Pct_Driver2AxisLB > 0.01) // ToDo: Add deadband cal
  {
    LeCONT_Pct_CLMR_Left_Test = LeCont_Pct_Driver2AxisLB; // ToDo: I think this direction flip is correct, need to verify
  }

  VsCONT_s_DriverInput.pct_LeftHook_Test = LeCONT_Pct_CLMR_Left_Test;

  if (LeCONT_b_Driver2ButtonRB == true)
  {
    LeCONT_Pct_CLMR_Right_Test = 1;
  }
  else if (LeCont_Pct_Driver2AxisRB > 0.01) // ToDo: Add deadband cal
  {
    LeCONT_Pct_CLMR_Right_Test = -LeCont_Pct_Driver2AxisRB; // ToDo: I think this direction flip is correct, need to verify
  }
 

  VsCONT_s_DriverInput.pct_RightHook_Test = LeCONT_Pct_CLMR_Right_Test;


/*Test for amp's elevator and wrist motors - test mode*/
/*controls the Speaker using D-pad - teleop mode*/
  if (LeCONT_Deg_Driver2POV == 0)
  {
    LeCONT_Pct_Amp_Wrist_Test = 1.0;
    LeCONT_b_SPK_Intake = true;
  }
  else if (LeCONT_Deg_Driver2POV == 180)
  {
    LeCONT_Pct_Amp_Wrist_Test = -1.0;
    LeCONT_b_SPK_PreScore = true;
  }
  else if (LeCONT_Deg_Driver2POV == 270)
  {
    LeCONT_Pct_Amp_Elevator_Test = -1.0;
    LeCONT_b_SPK_Score = true;
  }
  else if (LeCONT_Deg_Driver2POV == 90)
  {
    LeCONT_Pct_Amp_Elevator_Test = 1.0;
    LeCONT_b_SPK_Driving = true;
  }
  VsCONT_s_DriverInput.b_SPK_Intake = LeCONT_b_SPK_Intake;
  VsCONT_s_DriverInput.b_SPK_DrivingPosition = LeCONT_b_SPK_Driving;
  VsCONT_s_DriverInput.b_SPK_PreScore = LeCONT_b_SPK_PreScore;
  VsCONT_s_DriverInput.b_SPK_Score = LeCONT_b_SPK_Score;

  VsCONT_s_DriverInput.Pct_Amp_Wrist_Test = LeCONT_Pct_Amp_Wrist_Test;
  VsCONT_s_DriverInput.Pct_Amp_Elevator_Test = LeCONT_Pct_Amp_Elevator_Test;

/* Manual Climb controls on the comp bot*/
  if (LeCont_Pct_Driver2AxisRB > KeCLMR_Pct_CntrlDb)
    {
      LeCONT_Pct_Man_CLMR = -LeCont_Pct_Driver2AxisRB;
    }

  if (LeCont_Pct_Driver2AxisLB > KeCLMR_Pct_CntrlDb)
    {
      LeCONT_Pct_Man_CLMR = LeCont_Pct_Driver2AxisLB;
    }

  VsCONT_s_DriverInput.Pct_Manual_CLMR = LeCONT_Pct_Man_CLMR;
}