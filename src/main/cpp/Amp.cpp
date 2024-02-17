/*
  DJ.cpp

  Created on: January 31, 2024
  Author: Lauren

 */

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include <frc/DriverStation.h>
#include "Const.hpp"
#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Driver_inputs.hpp"
#include "Encoders.hpp"
#include "DJ.hpp"


TeAmp_MotorControl      VsAmp_s_Motors; // All of the motor commands for the Amp Mechanism motors
TeAmp_MotorControl      VsAmp_s_MotorsTemp; // Temporary commands for the motors, not the final output
TeAmp_MotorControl      VsAmp_s_MotorsTest; // Temporary commands for the motors, not the final output

TsDJ_Sensor            VsAmp_s_Sensors; // All of the sensor values for the manipulator/intake motors

T_DJ_Amp_States VeAmp_e_CmndState  = E_DJ_Amp_Init; // What is our next/current step?
T_DJ_Amp_States VeAmp_e_AttndState = E_DJ_Amp_Init; // What is our desired end state?

double                  VaAmp_k_ElevatorPID_Gx[E_PID_SparkMaxCalSz];
double                  VaAmp_k_WristPID_Gx[E_PID_SparkMaxCalSz];
double                  VaAmp_k_IntakePID_Gx[E_PID_SparkMaxCalSz];

double                  VaAmp_In_ElevatorError;
double                  VaAmp_k_ElevatorIntegral;

bool                    VeAmp_b_CriteriaMet = false;
bool                    VeAmp_t_TransitionTime = 0;
bool                    VeAmp_b_Hold = false;  // Used for the holding power.
bool                    VeAmp_t_HoldTime = 0;

#ifdef Amp_Test
bool                    VeAmp_b_TestState = true; // temporary, we don't want to use the manual overrides
#else
bool                    VeAmp_b_TestState = false;
#endif

/******************************************************************************
 * Function:     Amp_MotorConfigsInit
 *
 * Description:  Contains the motor configurations for the amp mechinism motors.
 ******************************************************************************/
 void Amp_MotorConfigsInit(rev::SparkMaxPIDController m_ElevatorPID,
                            rev::SparkMaxPIDController m_WristPID,
                           rev::SparkMaxPIDController m_IntakePID)
  {
    TeDJ_Amp_e_AmpActuator LeAmp_i_Index;
    T_PID_Cal LeAmp_i_Index3 = E_P_Gx;

  // set PID coefficients
    m_ElevatorPID.SetP(KaDJ_Amp_k_ElevatorPID_Gx[E_kP]);
    m_ElevatorPID.SetI(KaDJ_Amp_k_ElevatorPID_Gx[E_kI]);
    m_ElevatorPID.SetD(KaDJ_Amp_k_ElevatorPID_Gx[E_kD]);
    m_ElevatorPID.SetIZone(KaDJ_Amp_k_ElevatorPID_Gx[E_kIz]);
    m_ElevatorPID.SetFF(KaDJ_Amp_k_ElevatorPID_Gx[E_kFF]);
    m_ElevatorPID.SetOutputRange(KaDJ_Amp_k_ElevatorPID_Gx[E_kMinOutput], KaDJ_Amp_k_ElevatorPID_Gx[E_kMaxOutput]);
  
    m_WristPID.SetP(KaDJ_Amp_k_WristPID_Gx[E_kP]);
    m_WristPID.SetI(KaDJ_Amp_k_WristPID_Gx[E_kI]);
    m_WristPID.SetD(KaDJ_Amp_k_WristPID_Gx[E_kD]);
    m_WristPID.SetIZone(KaDJ_Amp_k_WristPID_Gx[E_kIz]);
    m_WristPID.SetFF(KaDJ_Amp_k_WristPID_Gx[E_kFF]);
    m_WristPID.SetOutputRange(KaDJ_Amp_k_WristPID_Gx[E_kMinOutput], KaDJ_Amp_k_WristPID_Gx[E_kMaxOutput]);  
  
    m_IntakePID.SetP(KaDJ_Amp_k_IntakePID_Gx[E_kP]);
    m_IntakePID.SetI(KaDJ_Amp_k_IntakePID_Gx[E_kI]);
    m_IntakePID.SetD(KaDJ_Amp_k_IntakePID_Gx[E_kD]);
    m_IntakePID.SetIZone(KaDJ_Amp_k_IntakePID_Gx[E_kIz]);
    m_IntakePID.SetFF(KaDJ_Amp_k_IntakePID_Gx[E_kFF]);
    m_IntakePID.SetOutputRange(KaDJ_Amp_k_IntakePID_Gx[E_kMinOutput], KaDJ_Amp_k_IntakePID_Gx[E_kMaxOutput]);    
  
   for (LeAmp_i_Index = E_Amp_Elevator;
       LeAmp_i_Index < E_Amp_Sz;
       LeAmp_i_Index = TeDJ_Amp_e_AmpActuator(int(LeAmp_i_Index) + 1))
    {
      VsAmp_s_Motors.k_MotorCmnd[LeAmp_i_Index] = 0.0;
      VsAmp_s_MotorsTemp.k_MotorCmnd[LeAmp_i_Index] = 0.0;
      VsAmp_s_MotorsTest.k_MotorCmnd[LeAmp_i_Index] = 0.0;
    }

  VsAmp_s_MotorsTest.k_MotorRampRate[E_Amp_Elevator] = KaDJ_Amp_InS_ElevatorRate[E_DJ_Amp_Init][E_DJ_Amp_Init];
  VsAmp_s_MotorsTest.k_MotorRampRate[E_Amp_Wrist] = KaDJ_Amp_DegS_WristRate[E_DJ_Amp_Init][E_DJ_Amp_Init];

  VaDJ_Amp_In_ElevatorError = 0.0;
  VaDJ_Amp_k_ElevatorIntegral = 0.0;

  for (LeAmp_i_Index3 = E_P_Gx;
       LeAmp_i_Index3 < E_PID_CalSz;
       LeAmp_i_Index3 = T_PID_Cal(int(LeAmp_i_Index3) + 1))
    {
    VaAmp_k_ElevatorPID_Gx[LeAmp_i_Index3] = KaDJ_Amp_k_ElevatorPID_Gx[LeAmp_i_Index3];
    }

  #ifdef Bot_Testing
  T_PID_SparkMaxCal LeAmp_i_Index2 = E_kP;

  for (LeAmp_i_Index2 = E_kP;
       LeAmp_i_Index2 < E_PID_SparkMaxCalSz;
       LeAmp_i_Index2 = T_PID_SparkMaxCal(int(LeAmp_i_Index2) + 1))
    {
    VaAmp_k_WristPID_Gx[LeAmp_i_Index2] = KaDJ_Amp_k_WristPID_Gx[LeAmp_i_Index2];
    VaAmp_k_IntakePID_Gx[LeAmp_i_Index2] = KaDJ_Amp_k_IntakePID_Gx[LeAmp_i_Index2];
    }

  for (LeAmp_i_Index3 = E_P_Gx;
       LeAmp_i_Index3 < E_PID_CalSz;
       LeAmp_i_Index3 = T_PID_Cal(int(LeAmp_i_Index3) + 1))
    {
    VaAmp_k_ElevatorPID_Gx[LeAmp_i_Index3] = KaDJ_Amp_k_ElevatorPID_Gx[LeAmp_i_Index3];
    }

  // display PID coefficients on SmartDashboard

  // frc::SmartDashboard::PutNumber("P Gain - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kP]);
  // frc::SmartDashboard::PutNumber("I Gain - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kI]);
  // frc::SmartDashboard::PutNumber("D Gain - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kD]);
  // frc::SmartDashboard::PutNumber("I Zone - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kIz]);
  // frc::SmartDashboard::PutNumber("Max Output - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kMaxOutput]);
  // frc::SmartDashboard::PutNumber("Min Output - Wrist", KaDJ_Amp_k_WristPID_Gx[E_kMinOutput]);

  // frc::SmartDashboard::PutNumber("P Gain - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kP]);
  // frc::SmartDashboard::PutNumber("I Gain - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kI]);
  // frc::SmartDashboard::PutNumber("D Gain - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kD]);
  // frc::SmartDashboard::PutNumber("I Zone - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kIz]);
  // frc::SmartDashboard::PutNumber("Max Output - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kMaxOutput]);
  // frc::SmartDashboard::PutNumber("Min Output - Intake", KaDJ_Amp_k_IntakePID_Gx[E_kMinOutput]);

  // frc::SmartDashboard::PutNumber("P Gain - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_P_Gx]);
  // frc::SmartDashboard::PutNumber("I Gain - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_I_Gx]);
  // frc::SmartDashboard::PutNumber("D Gain - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_D_Gx]);
  // frc::SmartDashboard::PutNumber("I Upper - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_I_Ul]);
  // frc::SmartDashboard::PutNumber("I Lower - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_I_Ll]);
  // frc::SmartDashboard::PutNumber("Max Output - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_Max_Ul]);
  // frc::SmartDashboard::PutNumber("Min Output - Elevator", KaDJ_Amp_k_ElevatorPID_Gx[E_Max_Ll]);

  // display secondary coefficients
  frc::SmartDashboard::PutNumber("KaDJ_Amp_InS_ElevatorRate", KaAmp_DegS_WristRate[E_DJ_Amp_Init][E_DJ_Amp_Init]);
  frc::SmartDashboard::PutNumber("KaDJ_Amp_DegS_WristRate", [E_DJ_Amp_Init][E_DJ_Amp_Init]);

  // display target positions/speeds
  frc::SmartDashboard::PutNumber("Set Position Wrist", 0);
  frc::SmartDashboard::PutNumber("Set Elevator", 0);
  frc::SmartDashboard::PutNumber("Set Speed Intake", 0);
  #endif
  }




/******************************************************************************
 * Function:     Amp_MotorConfigsCal
 *
 * Description:  Contains the motor configurations for the amp mechinism motors.  This 
 *               allows for rapid calibration, but must not be used for comp. 
 *               For PIDs
 ******************************************************************************/
void Amp_MotorConfigsCal(     rev::SparkMaxPIDController m_ElevatorPID,
                              rev::SparkMaxPIDController m_WristPID,
                              rev::SparkMaxPIDController m_IntakePID)
  { //TODO - old code had smart dashboard PID stuff sooo This is going to need Shuffleboard equivlents
  }

/******************************************************************************
 * Function:     AmpControlInit
 *
 * Description:  Initialization function for the amp moter controls.
 ******************************************************************************/
void AmpControlInit()
  {
  VeAmp_e_CmndState  = E_DJ_Amp_Init;
  VeAmp_e_AttndState = E_DJ_Amp_Init;

  VeAmp_b_CriteriaMet = false;
  VeAmp_t_TransitionTime = 0.0;
  }

/******************************************************************************
 * Function:     Amp_ControlManualOverride
 *
 * Description:  Manual override control used during the FRC test section. Use incase of Y2K -J 
 ******************************************************************************/
void Amp_ControlManualOverride(RobotUserInput *LsCONT_s_DriverInput)
  {
  TeDJ_Amp_e_AmpActuator LeMAN_i_Index;

  for (LeMAN_i_Index = E_Amp_Elevator;
       LeMAN_i_Index < E_Amp_Sz;
       LeMAN_i_Index = TeDJ_Amp_e_AmpActuator(int(LeMAN_i_Index) + 1))
    {
      VsAmp_s_Motors.k_MotorTestPower[LeMAN_i_Index] = 0.0;
    }

  VsAmp_s_Motors.k_MotorTestPower[E_Amp_Intake] = KaDJ_Amp_k_TestPower[E_Amp_Intake];
  
  VsAmp_s_Motors.k_MotorTestPower[E_Amp_Wrist] = LsCONT_s_DriverInput->Pct_WristTest * KaDJ_Amp_k_TestPower[E_Amp_Wrist];

  VsAmp_s_Motors.k_MotorTestPower[E_Amp_Elevator] = LsCONT_s_DriverInput->Pct_ArmPivotTest * KaDJ_Amp_k_TestPower[E_Amp_Elevator];

  }

/******************************************************************************
 * Function:     Update_Command_Attained_State
 *
 * Description:  Updates the commanded and attained states for the manipulator
 ******************************************************************************/
bool Update_Command_Atained_State(bool                    LeAmp_b_CriteriaMet,
                                  T_DJ_Amp_States LeDJ_Amp_e_SchedState)
  {
  T_DJ_Amp_States LeDJ_Amp_e_CmndState = VeAmp_e_CmndState;

  if(LeAmp_b_CriteriaMet == true)
    {
    VeAmp_e_AttndState = LeDJ_Amp_e_CmndState;
    LeAmp_b_CriteriaMet = false;
    }

  if((LeDJ_Amp_e_SchedState != VeAmp_e_AttndState) &&
     (VeAmp_e_CmndState  == VeAmp_e_AttndState))
    {
    LeDJ_Amp_e_CmndState = KaDJ_Amp_e_ControllingTable[LeDJ_Amp_e_SchedState][VeAmp_e_AttndState];
    }

  VeAmp_e_CmndState = LeDJ_Amp_e_CmndState;

  return(LeAmp_b_CriteriaMet);
  }

/******************************************************************************
 * Function:     CmndStateReached
 *
 * Description:  Checks to see if we have reached the desired commanded state
 ******************************************************************************/
bool CmndStateReached(T_DJ_Amp_States LeDJ_Amp_e_CmndState)
  {
  bool LeAmp_b_CriteriaMet = false;

  VeAmp_t_TransitionTime += C_ExeTime;

  if((VeAmp_t_TransitionTime >= KeDJ_Amp_t_StateTimeOut) ||

     ((VsAmp_s_Sensors.In_Elevator <= (KaDJ_Amp_In_ElevatorPosition[LeDJ_Amp_e_CmndState] + KaDJ_Amp_In_ElevatorDb[LeDJ_Amp_e_CmndState])) &&
      (VsAmp_s_Sensors.In_Elevator >= (KaDJ_Amp_In_ElevatorPosition[LeDJ_Amp_e_CmndState] - KaDJ_Amp_In_ElevatorDb[LeDJ_Amp_e_CmndState])) &&

      (VsAmp_s_Sensors.Deg_Wrist <= (KaDJ_Amp_Deg_WristAngle[LeDJ_Amp_e_CmndState] + KaDJ_Amp_Deg_WristDb[LeDJ_Amp_e_CmndState]))&&
      (VsAmp_s_Sensors.Deg_Wrist >= (KaDJ_Amp_Deg_WristAngle[LeDJ_Amp_e_CmndState] - KaDJ_Amp_Deg_WristDb[LeDJ_Amp_e_CmndState]))))
      {
      LeAmp_b_CriteriaMet = true;
      VeAmp_t_TransitionTime = 0.0;
      }

  return(LeAmp_b_CriteriaMet);
  }

/******************************************************************************
 * Function:     Update_Amp_Actuators
 *
 * Description:  Updates the intermediate state of the actuartors for the 
 *               Amp control
 ******************************************************************************/
void Update_Amp_Actuators(      T_DJ_Amp_States LeDJ_Amp_e_CmndState,
                                T_DJ_Amp_States LeDJ_Amp_e_AttndState)
  {
   double    LeAmp_InS_ElevatorRate = 0.0;
   double    LeAmp_DegS_WristRate = 0.0;
   T_PID_Cal LeAmp_i_Index = E_P_Gx;

   LeAmp_InS_ElevatorRate = KaDJ_Amp_InS_ElevatorRate[LeDJ_Amp_e_CmndState][LeDJ_Amp_e_AttndState];

   VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Elevator] = RampTo(KaDJ_Amp_In_ElevatorPosition[LeDJ_Amp_e_CmndState],  
                                                              VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Elevator],
                                                              LeAmp_InS_ElevatorRate);

   LeAmp_DegS_WristRate = KaDJ_Amp_DegS_WristRate[LeDJ_Amp_e_CmndState][LeDJ_Amp_e_AttndState];

   VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Wrist] = RampTo(KaDJ_Amp_Deg_WristAngle[LeDJ_Amp_e_CmndState] / KeENC_Deg_Wrist, 
                                                        VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Wrist],
                                                        LeAmp_DegS_WristRate);

   VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Intake] = KaDJ_Amp_RPM_IntakePower[LeDJ_Amp_e_CmndState];
  }

/******************************************************************************
 * Function:     UpdateAmp_hold_Actuator
 *
 * Description:  Updates the gripper roller control //NOTE - fix
 ******************************************************************************/
void UpdateGripperActuator(T_DJ_Amp_States LeDJ_Amp_e_CmndState,
                           T_DJ_Amp_States LeDJ_Amp_e_AttndState,
                           bool                    LeDJ_Amp_b_DropObject)
  {
   double LeDJ_Amp_k_TempCmnd = 0.0;
   bool   LeDJ_Amp_b_AllowedReleaseState = false;
   bool   LeAmp_b_Note = false;

 
   /* Determine if we are attempting to drop a cube or cone: */
   if ((LeDJ_Amp_e_AttndState == E_DJ_Amp_Intake))
     {
      LeAmp_b_Note = true;
     }

   /* Determine if we are in an allowed state to drop: */
   if ((LeDJ_Amp_e_AttndState == LeDJ_Amp_e_CmndState) &&
       ((LeAmp_b_Note == true) ||
       (VsAmp_s_Sensors.b_Amp_ObjDetected == true)))
     {
      LeDJ_Amp_b_AllowedReleaseState = true;
     }


   if ((LeDJ_Amp_b_AllowedReleaseState == true))
     {
      VeDJ_Amp_t_HoldTime = 0;
      if(VsAmp_s_Sensors.b_Amp_ObjDetected)
       {
        /* We are eitehr in cone mode or main intake*/
        LeDJ_Amp_k_TempCmnd = KeDJ_Amp_k_ReleaseNote;
       }
     }

   VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Intake] = LeDJ_Amp_k_TempCmnd;
  }

/******************************************************************************
 * Function:     AmpControlMain
 *
 * Description:  Main calling function for manipulator control.
 ******************************************************************************/
void AmpControlMain(T_DJ_Amp_States LeDJ_Amp_e_SchedState,
                            bool                    LeDJ_Amp_b_TestPowerOverride,
                            bool                    LeDJ_Amp_b_DropObject)
  {
  TeDJ_Amp_e_AmpActuator LeMAN_i_Index;
  double LeDJ_Amp_Deg_Error = 0.0;
  double LeDJ_Amp_k_P_Gain = 0.0;

  if (LeDJ_Amp_b_TestPowerOverride == true)
    {
    // Do nothing.  Robot is in test state using power commands for all the acutators
    }
  else if (VeAmp_b_TestState == true)
    {
    /* Only used for testing/calibration. */

     VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Elevator] = RampTo(VsAmp_s_MotorsTest.k_MotorCmnd[E_Amp_Elevator], 
                                                                VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Elevator],
                                                                VsAmp_s_MotorsTest.k_MotorRampRate[E_Amp_Elevator]);

     VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Wrist] = RampTo(VsAmp_s_MotorsTest.k_MotorCmnd[E_Amp_Wrist] / KeENC_Deg_Wrist, 
                                                          VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Wrist],
                                                          VsAmp_s_MotorsTest.k_MotorRampRate[E_Amp_Wrist]);

     VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Intake] = RampTo(VsAmp_s_MotorsTest.k_MotorCmnd[E_Amp_Intake] / KeENC_RPM_Intake, 
                                                                  VsAmp_s_MotorsTemp.k_MotorCmnd[E_Amp_Intake],
                                                                  VsAmp_s_MotorsTest.k_MotorRampRate[E_Amp_Intake]);
    }
  else
    {
    /* This is the actual manipulator control */
    VeAmp_b_CriteriaMet = Update_Command_Atained_State(VeAmp_b_CriteriaMet,
                                                       LeDJ_Amp_e_SchedState);

    Update_Amp_Actuators(VeAmp_e_CmndState, VeAmp_e_AttndState);

    UpdateAmp_hold_Actuator(VeAmp_e_CmndState,
                          VeAmp_e_AttndState,
                          LeDJ_Amp_b_DropObject);  // Need to come up with object detected

    if ((LeDJ_Amp_e_SchedState != VeAmp_e_CmndState) ||
        (LeDJ_Amp_e_SchedState != VeAmp_e_AttndState))
      {
        Update_Amp_Actuators(VeAmp_e_CmndState, VeAmp_e_AttndState);

        VeAmp_b_CriteriaMet = CmndStateReached(VeAmp_e_CmndState);
      }
    }

    /* Final output to the motor command that will be sent to the motor controller: */

    VsAmp_s_Motors.k_MotorCmnd[E_MAN_LinearSlide] =  -Control_PID( VsAmp_s_MotorsTemp.k_MotorCmnd[E_MAN_LinearSlide],
                                                                  VsAmp_s_Sensors.In_Elevator,
                                                                 &VaDJ_Amp_In_ElevatorError,
                                                                 &VaDJ_Amp_k_ElevatorIntegral,
                                                                  KaDJ_Amp_k_ElevatorPID_Gx[E_P_Gx],
                                                                  KaDJ_Amp_k_ElevatorPID_Gx[E_I_Gx],
                                                                  KaDJ_Amp_k_ElevatorPID_Gx[E_D_Gx],
                                                                  KaDJ_Amp_k_ElevatorPID_Gx[E_P_Ul],
                                                                  KaDJ_Amp_k_ElevatorPID_Gx[E_P_Ll],
                                                                  KaDJ_Amp_k_ElevatorPID_Gx[E_I_Ul],
                                                                  KaDJ_Amp_k_ElevatorPID_Gx[E_I_Ll],
                                                                  KaDJ_Amp_k_ElevatorPID_Gx[E_D_Ul],
                                                                  KaDJ_Amp_k_ElevatorPID_Gx[E_D_Ll],
                                                                  KaDJ_Amp_k_ElevatorPID_Gx[E_Max_Ul],
                                                                  KaDJ_Amp_k_ElevatorPID_Gx[E_Max_Ll]);

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_Wrist] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Wrist];

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_IntakeRollers] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_IntakeRollers];
  }