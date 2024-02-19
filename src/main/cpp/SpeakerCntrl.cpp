/*
  SpeakerCntrl.cpp

   Created on: Feb 17, 2024
   Author: 5561

   The underbelly intake and launch rollers state machine.
 */

#include "rev/CANSparkMax.h"
#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "Const.hpp"
#include "control_pid.hpp"
#include "Lookup.hpp"
#include "Driver_inputs.hpp"
#include "Encoders.hpp"
#include "ADAS_MN.hpp"

// the rpm where under it we consider the gripper holding something
double                  C_GripperRPMHoldingThreshold = 0.0;
double                  C_GripperRPMReadyThreshold = 1.0;
bool                    VeSPK_b_ReadyToGrab;
bool                    VeSPK_b_HasObject;

TeMAN_ManipulatorStates VeSPK_e_CmndState  = E_MAN_Init; // What is our next/current step?
TeMAN_ManipulatorStates VeSPK_e_AttndState = E_MAN_Init; // What is our desired end state?

TeSKP_MotorControl      VsSPK_s_Motors; // All of the motor commands for the speaker motors
TeSKP_MotorControl      VsSPK_s_MotorsTemp; // Temporary commands for the motors, not the final output
TeSKP_MotorControl      VsSPK_s_MotorsTest; // Temporary commands for the motors, not the final output
TsSPK_Sensor            VsSPK_s_Sensors; // All of the sensor values for the speaker  motors

double                  VeSPK_t_TransitionTime = 0;
double                  VeSPK_t_GripperHoldTime = 0; // Amount of time after object is detected, to continue pulling in

double                  VaSPK_k_IntakePID_Gx[E_PID_SparkMaxCalSz];
double                  VaSPK_k_Shooter1PID_Gx[E_PID_SparkMaxCalSz];
double                  VaSPK_k_Shooter2PID_Gx[E_PID_SparkMaxCalSz];

bool                    VeSPK_b_CriteriaMet = false;
bool                    VeSPK_b_NoteHold = false;  // Used for the holding power.  If cone, use cone cal, otherwise use cube.

#ifdef Manipulator_Test
bool                    VeSPK_b_TestState = true; // temporary, we don't want to use the manual overrides
#else
bool                    VeSPK_b_TestState = false;
#endif

/******************************************************************************
 * Function:     ManipulatorMotorConfigsInit
 *
 * Description:  Contains the motor configurations for the Arm and intake motors.
 ******************************************************************************/
void ManipulatorMotorConfigsInit(rev::SparkMaxPIDController m_UnderbellyPID,
                                 rev::SparkMaxPIDController m_Shooter1PID,
                                 rev::SparkMaxPIDController m_Shooter2PID)
  {
  TeSPK_Actuators LeSPK_i_Index  = E_SPK_m_Intake;
  T_PID_Cal       LeSPK_i_Index3 = E_P_Gx;

  rev::SparkMaxPIDController                 m_UnderbellyPID       = m_Underbelly.GetPIDController();
  rev::SparkMaxPIDController                 m_Shooter1PID         = m_Shooter1.GetPIDController();
  rev::SparkMaxPIDController                 m_Shooter2PID         = m_Shooter2.GetPIDController();

  // set PID coefficients
  m_UnderbellyPID.SetP(KaSPK_k_IntakePID_Gx[E_kP]);
  m_UnderbellyPID.SetI(KaSPK_k_IntakePID_Gx[E_kI]);
  m_UnderbellyPID.SetD(KaSPK_k_IntakePID_Gx[E_kD]);
  m_UnderbellyPID.SetIZone(KaSPK_k_IntakePID_Gx[E_kIz]);
  m_UnderbellyPID.SetFF(KaSPK_k_IntakePID_Gx[E_kFF]);
  m_UnderbellyPID.SetOutputRange(KaSPK_k_IntakePID_Gx[E_kMinOutput], KaSPK_k_IntakePID_Gx[E_kMaxOutput]);

  m_Shooter1PID.SetP(KaSPK_k_Shooter1PID_Gx[E_kP]);
  m_Shooter1PID.SetI(KaSPK_k_Shooter1PID_Gx[E_kI]);
  m_Shooter1PID.SetD(KaSPK_k_Shooter1PID_Gx[E_kD]);
  m_Shooter1PID.SetIZone(KaSPK_k_Shooter1PID_Gx[E_kIz]);
  m_Shooter1PID.SetFF(KaSPK_k_Shooter1PID_Gx[E_kFF]);
  m_Shooter1PID.SetOutputRange(KaSPK_k_Shooter1PID_Gx[E_kMinOutput], KaSPK_k_Shooter1PID_Gx[E_kMaxOutput]);

  m_Shooter2PID.SetP(KaSPK_k_Shooter2PID_Gx[E_kP]);
  m_Shooter2PID.SetI(KaSPK_k_Shooter2PID_Gx[E_kI]);
  m_Shooter2PID.SetD(KaSPK_k_Shooter2PID_Gx[E_kD]);
  m_Shooter2PID.SetIZone(KaSPK_k_Shooter2PID_Gx[E_kIz]);
  m_Shooter2PID.SetFF(KaSPK_k_Shooter2PID_Gx[E_kFF]);
  m_Shooter2PID.SetOutputRange(KaSPK_k_Shooter2PID_Gx[E_kMinOutput], KaSPK_k_Shooter2PID_Gx[E_kMaxOutput]);

  for (LeSPK_i_Index = E_SPK_m_Intake;
       LeSPK_i_Index < E_SPK_m_Sz;
       LeSPK_i_Index = TeSPK_Actuators(int(LeSPK_i_Index) + 1))
    {
      VsSPK_s_Motors.k_MotorCmnd[LeSPK_i_Index] = 0.0;
      VsSPK_s_MotorsTemp.k_MotorCmnd[LeSPK_i_Index] = 0.0;
      VsSPK_s_MotorsTest.k_MotorCmnd[LeSPK_i_Index] = 0.0;
    }

 /* We don't have a ramp rate for the intake motor as we are running as power based only... */
  VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter1] = KeSPK_RPMs_Shooter1Rate;
  VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter2] = KeSPK_RPMs_Shooter2Rate;

  #ifdef Manipulator_Test
  T_PID_SparkMaxCal LeMAN_i_Index2 = E_kP;

  for (LeMAN_i_Index2 = E_kP;
       LeMAN_i_Index2 < E_PID_SparkMaxCalSz;
       LeMAN_i_Index2 = T_PID_SparkMaxCal(int(LeMAN_i_Index2) + 1))
    {
    VaSPK_k_IntakePID_Gx[LeMAN_i_Index2] = KaSPK_k_IntakePID_Gx[LeMAN_i_Index2];
    VaSPK_k_Shooter1PID_Gx[LeMAN_i_Index2] = KaSPK_k_Shooter1PID_Gx[LeMAN_i_Index2];
    VaSPK_k_Shooter2PID_Gx[LeMAN_i_Index2] = KaSPK_k_Shooter2PID_Gx[LeMAN_i_Index2];
    }

  // display PID coefficients on SmartDashboard
  frc::SmartDashboard::PutNumber("P Gain - Intake", KaSPK_k_IntakePID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - Intake", KaSPK_k_IntakePID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - Intake", KaSPK_k_IntakePID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - Intake", KaSPK_k_IntakePID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - Intake", KaSPK_k_IntakePID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - Intake", KaSPK_k_IntakePID_Gx[E_kMinOutput]);

  frc::SmartDashboard::PutNumber("P Gain - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kMinOutput]);

  frc::SmartDashboard::PutNumber("P Gain - Shooter2", KaSPK_k_Shooter1PID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain - Shooter2", KaSPK_k_Shooter1PID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain - Shooter2", KaSPK_k_Shooter1PID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone - Shooter2", KaSPK_k_Shooter1PID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Max Output - Shooter2", KaSPK_k_Shooter1PID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output - Shooter2", KaSPK_k_Shooter1PID_Gx[E_kMinOutput]);

  // display secondary coefficients
  frc::SmartDashboard::PutNumber("KeSPK_RPMs_Shooter1Rate", KeSPK_RPMs_Shooter1Rate);
  frc::SmartDashboard::PutNumber("KeSPK_RPMs_Shooter2Rate", KeSPK_RPMs_Shooter2Rate);

  // display target positions/speeds
  frc::SmartDashboard::PutNumber("Set Intake Power",   0);
  frc::SmartDashboard::PutNumber("Set Shooter1 Speed", 0);
  frc::SmartDashboard::PutNumber("Set Shooter2 Speed", 0);
  #endif
  }


/******************************************************************************
 * Function:     ManipulatorMotorConfigsCal
 *
 * Description:  Contains the motor configurations for the manipulator motors.  This 
 *               allows for rapid calibration, but must not be used for comp.
 ******************************************************************************/
void ManipulatorMotorConfigsCal(rev::SparkMaxPIDController m_UnderbellyPID,
                                rev::SparkMaxPIDController m_Shooter1PID,
                                rev::SparkMaxPIDController m_Shooter2PID)
  {
  // read PID coefficients from SmartDashboard
  #ifdef Manipulator_Test
  bool LeMAN_b_IntakePosition = false;  // false is retracted, true extended
  double L_p_Intake   = frc::SmartDashboard::GetNumber("P Gain - Intake", KaSPK_k_IntakePID_Gx[E_kP]);
  double L_i_Intake   = frc::SmartDashboard::GetNumber("I Gain - Intake", KaSPK_k_IntakePID_Gx[E_kI]);
  double L_d_Intake   = frc::SmartDashboard::GetNumber("D Gain - Intake", KaSPK_k_IntakePID_Gx[E_kD]);
  double L_iz_Intake  = frc::SmartDashboard::GetNumber("I Zone - Intake", KaSPK_k_IntakePID_Gx[E_kIz]);
  double L_max_Intake = frc::SmartDashboard::GetNumber("Max Output - Intake", KaSPK_k_IntakePID_Gx[E_kMaxOutput]);
  double L_min_Intake = frc::SmartDashboard::GetNumber("Min Output - Intake", KaSPK_k_IntakePID_Gx[E_kMinOutput]);

  double L_p_Shooter1   = frc::SmartDashboard::GetNumber("P Gain - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kP]);
  double L_i_Shooter1   = frc::SmartDashboard::GetNumber("I Gain - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kI]);
  double L_d_Shooter1   = frc::SmartDashboard::GetNumber("D Gain - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kD]);
  double L_iz_Shooter1  = frc::SmartDashboard::GetNumber("I Zone - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kIz]);
  double L_max_Shooter1 = frc::SmartDashboard::GetNumber("Max Output - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kMaxOutput]);
  double L_min_Shooter1 = frc::SmartDashboard::GetNumber("Min Output - Shooter1", KaSPK_k_Shooter1PID_Gx[E_kMinOutput]);

  double L_p_Shooter2   = frc::SmartDashboard::GetNumber("P Gain - Shooter2", KaSPK_k_Shooter2PID_Gx[E_kP]);
  double L_i_Shooter2   = frc::SmartDashboard::GetNumber("I Gain - Shooter2", KaSPK_k_Shooter2PID_Gx[E_kI]);
  double L_d_Shooter2   = frc::SmartDashboard::GetNumber("D Gain - Shooter2", KaSPK_k_Shooter2PID_Gx[E_kD]);
  double L_iz_Shooter2  = frc::SmartDashboard::GetNumber("I Zone - Shooter2", KaSPK_k_Shooter2PID_Gx[E_kIz]);
  double L_max_Shooter2 = frc::SmartDashboard::GetNumber("Max Output - Shooter2", KaSPK_k_Shooter2PID_Gx[E_kMaxOutput]);
  double L_min_Shooter2 = frc::SmartDashboard::GetNumber("Min Output - Shooter2", KaSPK_k_Shooter2PID_Gx[E_kMinOutput]);

  VsSPK_s_MotorsTest.k_MotorCmnd[E_SPK_m_Intake]   = frc::SmartDashboard::GetNumber("Set Intake Power", 0);
  VsSPK_s_MotorsTest.k_MotorCmnd[E_SPK_m_Shooter1] = frc::SmartDashboard::GetNumber("Set Shooter1 Speed", 0);
  VsSPK_s_MotorsTest.k_MotorCmnd[E_SPK_m_Shooter2] = frc::SmartDashboard::GetNumber("Set Shooter2 Speed", 0);

  if(L_p_Intake != VaSPK_k_IntakePID_Gx[E_kP])   { m_UnderbellyPID.SetP(L_p_Intake); VaSPK_k_IntakePID_Gx[E_kP] = L_p_Intake; }
  if(L_i_Intake != VaSPK_k_IntakePID_Gx[E_kI])   { m_UnderbellyPID.SetI(L_i_Intake); VaSPK_k_IntakePID_Gx[E_kI] = L_i_Intake; }
  if(L_d_Intake != VaSPK_k_IntakePID_Gx[E_kD])   { m_UnderbellyPID.SetD(L_d_Intake); VaSPK_k_IntakePID_Gx[E_kD] = L_d_Intake; }
  if(L_iz_Intake != VaSPK_k_IntakePID_Gx[E_kIz]) { m_UnderbellyPID.SetIZone(L_iz_Intake); VaSPK_k_IntakePID_Gx[E_kIz] = L_iz_Intake; }
  if((L_max_Intake != VaSPK_k_IntakePID_Gx[E_kMaxOutput]) || (L_min_Intake != VaSPK_k_IntakePID_Gx[E_kMinOutput])) { m_UnderbellyPID.SetOutputRange(L_min_Intake, L_max_Intake); VaSPK_k_IntakePID_Gx[E_kMinOutput] = L_min_Intake; VaSPK_k_IntakePID_Gx[E_kMaxOutput] = L_max_Intake; }

  if(L_p_Shooter1 != VaSPK_k_Shooter1PID_Gx[E_kP])   { m_Shooter1PID.SetP(L_p_Shooter1); VaSPK_k_Shooter1PID_Gx[E_kP] = L_p_Shooter1; }
  if(L_i_Shooter1 != VaSPK_k_Shooter1PID_Gx[E_kI])   { m_Shooter1PID.SetI(L_i_Shooter1); VaSPK_k_Shooter1PID_Gx[E_kI] = L_i_Shooter1; }
  if(L_d_Shooter1 != VaSPK_k_Shooter1PID_Gx[E_kD])   { m_Shooter1PID.SetD(L_d_Shooter1); VaSPK_k_Shooter1PID_Gx[E_kD] = L_d_Shooter1; }
  if(L_iz_Shooter1 != VaSPK_k_Shooter1PID_Gx[E_kIz]) { m_Shooter1PID.SetIZone(L_iz_Shooter1); VaSPK_k_Shooter1PID_Gx[E_kIz] = L_iz_Shooter1; }
  if((L_max_Shooter1 != VaSPK_k_Shooter1PID_Gx[E_kMaxOutput]) || (L_min_Shooter1 != VaSPK_k_Shooter1PID_Gx[E_kMinOutput])) { m_Shooter1PID.SetOutputRange(L_min_Shooter1, L_max_Shooter1); VaSPK_k_Shooter1PID_Gx[E_kMinOutput] = L_min_Shooter1; VaSPK_k_Shooter1PID_Gx[E_kMaxOutput] = L_max_Shooter1; }

  if(L_p_Shooter2 != VaSPK_k_Shooter2PID_Gx[E_kP])   { m_Shooter2PID.SetP(L_p_Shooter2); VaSPK_k_Shooter2PID_Gx[E_kP] = L_p_Shooter2; }
  if(L_i_Shooter2 != VaSPK_k_Shooter2PID_Gx[E_kI])   { m_Shooter2PID.SetI(L_i_Shooter2); VaSPK_k_Shooter2PID_Gx[E_kI] = L_i_Shooter2; }
  if(L_d_Shooter2 != VaSPK_k_Shooter2PID_Gx[E_kD])   { m_Shooter2PID.SetD(L_d_Shooter2); VaSPK_k_Shooter2PID_Gx[E_kD] = L_d_Shooter2; }
  if(L_iz_Shooter2 != VaSPK_k_Shooter2PID_Gx[E_kIz]) { m_Shooter2PID.SetIZone(L_iz_Shooter2); VaSPK_k_Shooter2PID_Gx[E_kIz] = L_iz_Shooter2; }
  if((L_max_Shooter2 != VaSPK_k_Shooter2PID_Gx[E_kMaxOutput]) || (L_min_Shooter2 != VaSPK_k_Shooter2PID_Gx[E_kMinOutput])) { m_Shooter2PID.SetOutputRange(L_min_Shooter2, L_max_Shooter2); VaSPK_k_Shooter2PID_Gx[E_kMinOutput] = L_min_Shooter2; VaSPK_k_Shooter2PID_Gx[E_kMaxOutput] = L_max_Shooter2; }

  VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter1] = frc::SmartDashboard::GetNumber("KeSPK_RPMs_Shooter1Rate", VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter1]);
  VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter2] = frc::SmartDashboard::GetNumber("KeSPK_RPMs_Shooter2Rate", VsSPK_s_MotorsTest.k_MotorRampRate[E_SPK_m_Shooter2]);
   #endif
  }


/******************************************************************************
 * Function:     ManipulatorControlInit
 *
 * Description:  Initialization function for the Manipulator controls.
 ******************************************************************************/
void ManipulatorControlInit()
  {
  VeSPK_e_CmndState  = E_MAN_Init;
  VeSPK_e_AttndState = E_MAN_Init;

  VeSPK_b_CriteriaMet = false;
  VeSPK_t_TransitionTime = 0.0;
  VeSPK_b_NoteHold = false;
  }


/******************************************************************************
 * Function:     ManipulatorControlManualOverride
 *
 * Description:  Manual override control used during the FRC test section. Use incase of Y2K -J 
 ******************************************************************************/
void ManipulatorControlManualOverride(RobotUserInput *LsCONT_s_DriverInput)
  {
  TeSPK_Actuators LeSPK_i_Index;

  for (LeSPK_i_Index = E_SPK_m_Intake;
       LeSPK_i_Index < E_SPK_m_Sz;
       LeSPK_i_Index = TeSPK_Actuators(int(LeSPK_i_Index) + 1))
    {
      VsSPK_s_Motors.k_MotorTestPower[LeSPK_i_Index] = 0.0;
    }

  if (LsCONT_s_DriverInput->b_SPK_IntakeFwdTest == true)
    {
    VsSPK_s_Motors.k_MotorTestPower[LeSPK_i_Index] = KaSPK_k_TestPower[E_SPK_m_Intake];
    }
  else if (LsCONT_s_DriverInput->b_SPK_IntakeBkTest == true)
    {
    VsSPK_s_Motors.k_MotorTestPower[LeSPK_i_Index] = -KaSPK_k_TestPower[E_SPK_m_Intake];
    }

  VsSPK_s_Motors.k_MotorTestPower[E_SPK_m_Shooter1] = LsCONT_s_DriverInput->Pct_SPK_Shooter1Test * KaSPK_k_TestPower[E_SPK_m_Shooter1];

  VsSPK_s_Motors.k_MotorTestPower[E_SPK_m_Shooter2] = LsCONT_s_DriverInput->Pct_SPK_Shooter2Test * KaSPK_k_TestPower[E_SPK_m_Shooter2];
  }

/******************************************************************************
 * Function:     UpdateSpeakerCommandAttainedState
 *
 * Description:  Updates the commanded and attained states for the speaker manipulator
 ******************************************************************************/
bool UpdateSpeakerCommandAttainedState(bool             LeSPK_b_CriteriaMet,
                                       TeSPK_CtrlStates LeSPK_e_SchedState)
  {
  TeSPK_CtrlStates LeSPK_e_CmndState = VeSPK_e_CmndState;

  if(LeSPK_b_CriteriaMet == true)
    {
    VeSPK_e_AttndState = LeMAN_e_CmndState;
    LeSPK_b_CriteriaMet = false;
    }

  if((LeSPK_e_SchedState != VeSPK_e_AttndState) &&
     (VeSPK_e_CmndState  == VeSPK_e_AttndState))
    {
    LeSPK_e_CmndState = KaSPK_e_ControllingTable[LeSPK_e_SchedState][VeSPK_e_AttndState];
    }

  VeSPK_e_CmndState = LeSPK_e_CmndState;

  return(LeSPK_b_CriteriaMet);
  }


/******************************************************************************
 * Function:     CmndStateReachedSpeaker
 *
 * Description:  Checks to see if we have reached the desired commanded state
 ******************************************************************************/
bool CmndStateReachedSpeaker(TeSPK_CtrlStates LeSPK_e_CmndState)
  {
  bool LeSPK_b_CriteriaMet = false;

  VeSPK_t_TransitionTime += C_ExeTime;

  if((VeSPK_t_TransitionTime >= KeMAN_t_StateTimeOut) ||

     ((VsSPK_s_Sensors.RPM_Shooter1 <= (KaSPK_RPM_Shooter1[LeSPK_e_CmndState] + KaSPK_RPM_Shooter1Db[LeSPK_e_CmndState])) &&
      (VsSPK_s_Sensors.RPM_Shooter1 >= (KaSPK_RPM_Shooter1[LeSPK_e_CmndState] - KaSPK_RPM_Shooter1Db[LeSPK_e_CmndState])) &&

      (VsSPK_s_Sensors.RPM_Shooter2 <= (KaSPK_RPM_Shooter2[LeSPK_e_CmndState] + KaSPK_RPM_Shooter2Db[LeSPK_e_CmndState])) &&
      (VsSPK_s_Sensors.RPM_Shooter2 >= (KaSPK_RPM_Shooter2[LeSPK_e_CmndState] - KaSPK_RPM_Shooter2Db[LeSPK_e_CmndState])))))
      {
      LeSPK_b_CriteriaMet = true;
      VeSPK_t_TransitionTime = 0.0;
      }

  return(LeSPK_b_CriteriaMet);
  }


/******************************************************************************
 * Function:     UpdateManipulatorActuators
 *
 * Description:  Updates the intermediate state of the actuartors for the 
 *               manipulator
 ******************************************************************************/
void UpdateManipulatorActuators(TeSPK_CtrlStates LeSPK_e_CmndState,
                                TeSPK_CtrlStates LeSPK_e_AttndState)
  {
  //  T_PID_Cal LeSPK_i_Index = E_P_Gx;

   VsMAN_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter1] = RampTo(KaSPK_RPM_Shooter1[LeSPK_e_CmndState], 
                                                             VsMAN_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter1],
                                                             KeSPK_RPMs_Shooter1Rate);

   VsMAN_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter2] = RampTo(KaSPK_RPM_Shooter2[LeSPK_e_CmndState], 
                                                             VsMAN_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Shooter2],
                                                             KeSPK_RPMs_Shooter2Rate);
  }


/******************************************************************************
 * Function:     UpdateGripperActuator
 *
 * Description:  Updates the gripper roller control
 ******************************************************************************/
void UpdateGripperActuator(TeSPK_CtrlStates LeSPK_e_CmndState,
                           TeSPK_CtrlStates LeSPK_e_AttndState,
                           bool                    LeMAN_b_DropObjectSlow,
                           bool                    LeMAN_b_DropObjectFast)
  {
   double LeMAN_k_TempCmnd = 0.0;
   bool   LeMAN_b_AllowedReleaseState = false;
   bool   LeMAN_b_ConeState = false;
   bool   LeMAN_b_CubeState = false;

  // frc::SmartDashboard::PutBoolean("Gripper Has Object", VeSPK_b_HasObject);
  // frc::SmartDashboard::PutBoolean("Gripper ready to grab", VeSPK_b_ReadyToGrab);

  // if (fabs(VsMAN_s_Sensors.RPM_Gripper) > C_GripperRPMReadyThreshold)
  // {
  //   VeSPK_b_ReadyToGrab = true;
  // }
  // else if ((fabs(VsMAN_s_Sensors.RPM_Gripper) <= C_GripperRPMReadyThreshold) && (VeSPK_b_HasObject == false))
  // {
  //   VeSPK_b_ReadyToGrab = false;
  // }

  // if ((fabs(VsMAN_s_Sensors.RPM_Gripper) <= C_GripperRPMHoldingThreshold) && (VeSPK_b_ReadyToGrab) &&
  //     (LeMAN_e_CmndState == E_MAN_MidCubeIntake ||
  //      LeMAN_e_CmndState == E_MAN_MidConeIntake ||
  //      LeMAN_e_CmndState == E_MAN_FloorConeDrop ||
  //      LeMAN_e_CmndState == E_MAN_MainIntake))
  // {
  //   VeSPK_b_HasObject = true;
  // }
  // else if (fabs(VsMAN_s_Sensors.RPM_Gripper) > C_GripperRPMHoldingThreshold)
  // {
  //   VeSPK_b_HasObject = false;
  // }

   /* Determine if we are attempting to drop a cube or cone: */
   if ((LeSPK_e_CmndState == E_SPK_Ctrl_Intake) ||
       (LeMAN_e_AttndState == E_MAN_LowCubeDrop)  ||
       (LeMAN_e_AttndState == E_MAN_MidCubeIntake) ||
       (LeMAN_e_AttndState == E_MAN_MainIntake))
     {
      LeMAN_b_CubeState = true;
      VeSPK_b_NoteHold = false;
     }

   if ((LeMAN_e_AttndState == E_MAN_HighConeDrop) ||
       (LeMAN_e_AttndState == E_MAN_LowConeDrop)  ||
       (LeMAN_e_AttndState == E_MAN_MidConeIntake) ||
       (LeMAN_e_AttndState == E_MAN_FloorConeDrop))
     {
      LeMAN_b_ConeState = true;
      VeSPK_b_NoteHold = true;
     }

   /* Determine if we are in an allowed state to drop: */
   if ((LeMAN_e_AttndState == LeMAN_e_CmndState) &&

       ((LeMAN_b_CubeState == true) ||
        (LeMAN_b_ConeState == true)  ||
        ((LeMAN_e_AttndState == E_MAN_Driving) && (VeSPK_b_NoteHold == false)) ||  // Do not allow a cone to be shot out in driving
        (LeMAN_e_AttndState == E_MAN_MainIntake)))
     {
      LeMAN_b_AllowedReleaseState = true;
     }


   if ((LeMAN_b_DropObjectSlow == true) && (LeMAN_b_AllowedReleaseState == true))
     {
      VeMAN_t_GripperHoldTime = 0;
      if (VeSPK_b_NoteHold == false)
       {
        LeMAN_k_TempCmnd = KeMAN_k_GripperReleaseCubeSlow;
       }
      else
       {
        /* We are eitehr in cone mode or main intake*/
        LeMAN_k_TempCmnd = KeMAN_k_GripperReleaseConeSlow;
       }
     }
   else if ((LeMAN_b_DropObjectFast == true) && (LeMAN_b_AllowedReleaseState == true))
     {
      VeMAN_t_GripperHoldTime = 0;
      if (VeSPK_b_NoteHold == false)
       {
        LeMAN_k_TempCmnd = KeMAN_k_GripperReleaseCubeFast;
       }
      else
       {
        /* We are eitehr in cone mode or main intake*/
        LeMAN_k_TempCmnd = KeMAN_k_GripperReleaseConeFast;
       }
     }
   else if ((VeMAN_t_GripperHoldTime < KeMAN_t_GripperPullInTm) &&
            (((LeMAN_e_AttndState == E_MAN_MainIntake)    && (LeMAN_e_CmndState  == E_MAN_MainIntake)) ||
             ((LeMAN_e_AttndState == E_MAN_MidCubeIntake) && (LeMAN_e_CmndState  == E_MAN_MidCubeIntake))))
     {
     LeMAN_k_TempCmnd = KeMAN_k_GripperIntakeCube;
     if (VsMAN_s_Sensors.b_GripperObjDetected == true)
       {
        VeMAN_t_GripperHoldTime += C_ExeTime;
       }
       else
       {
        VeMAN_t_GripperHoldTime = 0;
       }
     }
   else if ((VeMAN_t_GripperHoldTime < KeMAN_t_GripperPullInTm) &&
             (((LeMAN_e_AttndState == E_MAN_MidConeIntake)   && (LeMAN_e_CmndState  == E_MAN_MidConeIntake))))
     {
     LeMAN_k_TempCmnd = KeMAN_k_GripperIntakeCone;
      if (VsMAN_s_Sensors.b_GripperObjDetected == true)
       {
        VeMAN_t_GripperHoldTime += C_ExeTime;
       }
      else
       {
        VeMAN_t_GripperHoldTime = 0;
       }
     }
   else if ((VsMAN_s_Sensors.b_GripperObjDetected == true) || (VeMAN_t_GripperHoldTime >= KeMAN_t_GripperPullInTm))
     {
      if (VeSPK_b_NoteHold == true)
       {
        LeMAN_k_TempCmnd = KeMAN_k_GripperIntakeholdCone;
       }
      else
       {
        LeMAN_k_TempCmnd = KeMAN_k_GripperIntakeholdCube;
       }
     }

   VsSPK_s_MotorsTemp.k_MotorCmnd[E_SPK_m_Intake] = LeMAN_k_TempCmnd;
  }

/******************************************************************************
 * Function:     ManipulatorControlMain
 *
 * Description:  Main calling function for manipulator control.
 ******************************************************************************/
void ManipulatorControlMain(TeMAN_ManipulatorStates LeMAN_e_SchedState,
                            bool                    LeMAN_b_TestPowerOverride,
                            bool                    LeMAN_b_DropObjectSlow,
                            bool                    LeMAN_b_DropObjectFast)
  {
  TeMAN_e_ManipulatorActuator LeMAN_i_Index;
  double LeMAN_Deg_Error = 0.0;
  double LeMAN_k_P_Gain = 0.0;

  if (LeMAN_b_TestPowerOverride == true)
    {
    // Do nothing.  Robot is in test state using power commands for all the acutators
    }
  else if (VeMAN_b_TestState == true)
    {
    /* Only used for testing/calibration. */
     VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_ArmPivot] = RampTo(VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_ArmPivot] / KeENC_k_ArmPivot, 
                                                             VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_ArmPivot],
                                                             VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_ArmPivot]);

     VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_LinearSlide] = RampTo(VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_LinearSlide], 
                                                                VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_LinearSlide],
                                                                VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_LinearSlide]);

     VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Wrist] = RampTo(VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_Wrist] / KeENC_Deg_Wrist, 
                                                          VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Wrist],
                                                          VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_Wrist]);

     VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Gripper] = RampTo(VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_Gripper], 
                                                          VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Gripper],
                                                          1.0);

     VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_IntakeRollers] = RampTo(VsMAN_s_MotorsTest.k_MotorCmnd[E_MAN_IntakeRollers] / KeENC_RPM_IntakeRollers, 
                                                                  VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_IntakeRollers],
                                                                  VsMAN_s_MotorsTest.k_MotorRampRate[E_MAN_IntakeRollers]);

     VsMAN_s_MotorsTemp.e_MotorControlType[E_MAN_IntakeArm] = VsMAN_s_MotorsTest.e_MotorControlType[E_MAN_IntakeArm];
    }
  else
    {
    /* This is the actual manipulator control */
    VeSPK_b_CriteriaMet = UpdateSpeakerCommandAttainedState(VeSPK_b_CriteriaMet,
                                                            LeMAN_e_SchedState);

    UpdateManipulatorActuators(VeSPK_e_CmndState, VeSPK_e_AttndState);

    UpdateGripperActuator(VeSPK_e_CmndState,
                          VeSPK_e_AttndState,
                          LeMAN_b_DropObjectSlow,
                          LeMAN_b_DropObjectFast);  // Need to come up with object detected

    if ((LeMAN_e_SchedState != VeSPK_e_CmndState) ||
        (LeMAN_e_SchedState != VeSPK_e_AttndState))
      {
        UpdateManipulatorActuators(VeSPK_e_CmndState, VeSPK_e_AttndState);

        VeSPK_b_CriteriaMet = CmndStateReachedSpeaker(VeSPK_e_CmndState);
      }
    }

    /* Final output to the motor command that will be sent to the motor controller: */
    VsMAN_s_Motors.k_MotorCmnd[E_MAN_ArmPivot] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_ArmPivot];

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_LinearSlide] =  -Control_PID( VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_LinearSlide],
                                                                  VsMAN_s_Sensors.In_LinearSlide,
                                                                 &VaMAN_In_LinearSlideError,
                                                                 &VaMAN_k_LinearSlideIntegral,
                                                                  KaMAN_k_LinearSlidePID_Gx[E_P_Gx],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_I_Gx],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_D_Gx],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_P_Ul],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_P_Ll],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_I_Ul],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_I_Ll],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_D_Ul],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_D_Ll],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_Max_Ul],
                                                                  KaMAN_k_LinearSlidePID_Gx[E_Max_Ll]);

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_Wrist] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Wrist];

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_Gripper] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_Gripper];

    VsMAN_s_Motors.k_MotorCmnd[E_MAN_IntakeRollers] = VsMAN_s_MotorsTemp.k_MotorCmnd[E_MAN_IntakeRollers];

    VsMAN_s_Motors.e_MotorControlType[E_MAN_IntakeArm] = VsMAN_s_MotorsTemp.e_MotorControlType[E_MAN_IntakeArm];
  }