/*
  SpeakerCntrl.hpp

   Created on: Feb 19, 2024
   Author: Biggs

 */

extern TeSPK_CtrlStates   VeSPK_e_CmndState;
extern TeSPK_CtrlStates   VeSPK_e_AttndState;
extern TeSKP_MotorControl VsSPK_s_Motors;
extern TeSKP_MotorControl VsSPK_s_MotorsTemp;
extern TeSKP_MotorControl VsSPK_s_MotorsTest;
extern TsSPK_Sensor       VsSPK_s_Sensors; 

void SPK_MotorConfigsCal(rev::SparkMaxPIDController m_UnderbellyPID,
                         rev::SparkMaxPIDController m_Shooter1PID,
                         rev::SparkMaxPIDController m_Shooter2PID);

void SPK_MotorConfigsInit(rev::SparkMaxPIDController m_UnderbellyPID,
                          rev::SparkMaxPIDController m_Shooter1PID,
                          rev::SparkMaxPIDController m_Shooter2PID);

void SPK_ControlInit();

void SPK_ControlManualOverride(RobotUserInput *LsCONT_s_DriverInput);

void SPK_SpeakerControlMain(TeSPK_CtrlStates LeSPK_e_SchedState,
                            bool             LeSPK_b_TestPowerOverride);