/*
  Climber.hpp

   Created on: Feb 21, 2024
   Author: Biggs

 */

extern TeCLMR_CtrlStates   VeCLMR_e_CmndState;
extern TeCLMR_CtrlStates   VeCLMR_e_AttndState;
extern TeCLMR_MotorControl VsCLMR_s_Motors;
extern TeCLMR_MotorControl VsCLMR_s_MotorsTemp;
extern TeCLMR_MotorControl VsCLMR_s_MotorsTest;
extern TsCLMR_Sensor       VsCLMR_s_Sensors; 

// void CLMR_MotorConfigsCal(rev::SparkMaxPIDController m_UnderbellyPID,
//                           rev::SparkMaxPIDController m_Shooter1PID,
//                           rev::SparkMaxPIDController m_Shooter2PID);

// void CLMR_MotorConfigsInit(rev::SparkMaxPIDController m_UnderbellyPID,
//                           rev::SparkMaxPIDController m_Shooter1PID,
//                           rev::SparkMaxPIDController m_Shooter2PID);

// void CLMR_ControlInit();

void CLMR_ControlManualOverride(RobotUserInput *LsCONT_s_DriverInput);

// void CLMR_SpeakerControlMain(TeSPK_CtrlStates LeSPK_e_SchedState,
//                              bool             LeSPK_b_TestPowerOverride);