/*
  ADAS.hpp

  Created on: Feb 25, 2022
  Author: Biggs

  ADAS (Advanced Driver-Assistance Systems)

  Changes:
  2022-02-25 -> Beta
 */

extern T_ADAS_ActiveFeature VeADAS_e_ActiveFeature;

extern double VeADAS_Pct_SD_FwdRev;
extern double VeADAS_Pct_SD_Strafe;
extern double VeADAS_Pct_SD_Rotate;
extern double VeADAS_Deg_SD_DesiredPose;

extern bool VeADAS_b_SD_RobotOriented;
extern bool VeADAS_b_X_Mode;

extern double VeADAS_in_OffsetRequestX;
extern double VeADAS_in_OffsetRequestY;

extern double VeADAS_in_GlobalRequestX;
extern double VeADAS_in_GlobalRequestY;
