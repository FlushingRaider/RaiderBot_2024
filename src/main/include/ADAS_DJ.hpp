/*
  ADAS_DJ.hpp

  Created on: Feb 17, 2024
  Author: Lauren

  ADAS (Advanced Driver-Assistance Systems)
    DJ (our 2024 controls, Amp.cpp and Speaker_Cntrl.cpp)
 */

extern T_DJ_Amp_States  VeADAS_e_Amp_SchedState;
extern bool             VeADAS_b_Amp_DropObject;
extern TeSPK_CtrlStates VeADAS_e_SPK_SchedState;
extern TeCLMR_CtrlStates VeADAS_e_CLMR_SchedState;

void ADAS_DJ_Reset(void);

bool ADAS_DJ_Main(T_RobotState                  L_RobotState,
                  T_ADAS_ActiveFeature          LeADAS_e_ActiveFeature,
                  TeADAS_Auton_SPK_States       LeADAS_e_DJ_ReqAction);