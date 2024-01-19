/*
  Driver_inputs.hpp

   Created on: Feb 05, 2022
   Author: Lauren and Chloe uwu

  Function that maps the driver inputs to the robot controls. 

  Imported to 2024: January 18, 2024
 */

extern RobotUserInput    VsCONT_s_DriverInput;
 
void Joystick1_robot_mapping(bool    LeCont_b_Driver1ButtonBack,
                             bool    LeCont_b_Driver1ButtonStart,
                             double  LeCont_Cmd_Driver1LeftAxisY,
                             double  LeCont_Cmd_Driver1LeftAxisX,
                             double  LeCont_Cmd_Driver1RightAxisX,
                             double  LeCont_Cmd_Driver1LeftTriggerAxis,
                             bool    LeCont_b_Driver1ButtonA,
                             bool    LeCont_b_Driver1ButtonX,
                             bool    LeCont_b_Driver1ButtonY,
                             bool    LeCont_b_Driver1ButtonRB,
                             bool    LeCont_b_Driver1ButtonB,
                             bool    LeCont_b_Driver1ButtonLB,
                             int     LeCont_Deg_Driver1POV);

void Joystick2_robot_mapping(bool    LeCont_b_Driver2ButtonA,
                             bool    LeCont_b_Driver2ButtonB,
                             bool    LeCont_b_Driver2ButtonRB,
                             bool    LeCont_b_Driver2ButtonLB,
                             bool    LeCont_b_Driver2ButtonStart,
                             bool    LeCont_b_Driver2ButtonX,
                             bool    LeCont_b_Driver2ButtonY,
                             double  LeCont_Pct_Driver2LeftAxisY,
                             double  LeCont_Pct_Driver2RightAxisX,
                             int     LeCont_Deg_Driver2POV,
                             bool    LeCont_b_Driver2ButtonBack,
                             double  LeCont_Pct_Driver2AxisRB,
                             double  LeCont_Pct_Driver2AxisLB);