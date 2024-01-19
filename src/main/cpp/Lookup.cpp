/*
  Lookup.cpp

  Created on: Jan 3, 2020
  Author: 5561

  This file contains functions related to lookup and interpolation.

  Imported to 2024: January 18, 2024
 */
#include "Const.hpp"
#include "Pathloader.hpp"
#include <frc/DriverStation.h>
#include <math.h>
#include "MotionProfiles/BlueP1.hpp"
#include "MotionProfiles/BlueP2.hpp"
#include "MotionProfiles/BlueP3.hpp"
#include "MotionProfiles/BlueP4.hpp"
#include "MotionProfiles/BlueFWD.hpp"


/******************************************************************************
 * Function:     LookUp1D_Table
 *
 * Description:  Single dimension lookup table.
 ******************************************************************************/
double LookUp1D_Table(const double *LKeLU_Cmd_XAxis,
                      const double *LKLU_Cmd_TableData1D,
                      int LeLU_Int_AxisSize,
                      int LaLU_CalArraySize,
                      double LeLU_Cmd_Input)
{
  int LeLU_Int_Index = 0;
  double LeLU_Int_LookupX1 = 0.0;
  double LeLU_Int_LookupX2 = 0.0;
  double LeLU_Int_LookupXDiff = 0.0;
  double LeLU_Int_LookupY1 = 0.0;
  double LeLU_Int_LookupY2 = 0.0;
  double LeLU_Int_LookupYDiff = 0.0;
  double LeLU_Int_LookupDiv = 0.0;
  bool LeLU_b_LookUpPt1Found = false;
  double LeLU_Int_Output = 0.0;

  /* Table length MUST equal axis length. */
  if (LaLU_CalArraySize == LeLU_Int_AxisSize)
  {
    if (LeLU_Cmd_Input >= (LKeLU_Cmd_XAxis[LeLU_Int_AxisSize - 1]))
    {
      // We have gone off or are at the end of the axis
      return (LKLU_Cmd_TableData1D[LeLU_Int_AxisSize - 1]);
    }
    else if (LeLU_Cmd_Input <= (LKeLU_Cmd_XAxis[0]))
    {
      // We have gone off or are at the beginning of the axis
      return (LKLU_Cmd_TableData1D[0]);
    }
    else
    {
      for (LeLU_Int_Index = 0; ((LeLU_Int_Index < (LeLU_Int_AxisSize - 1)) && (LeLU_b_LookUpPt1Found == false)); LeLU_Int_Index++)
      {
        if ((LeLU_Cmd_Input >= LKeLU_Cmd_XAxis[LeLU_Int_Index]) &&
            (LeLU_Cmd_Input < LKeLU_Cmd_XAxis[LeLU_Int_Index + 1]) &&
            (LeLU_b_LookUpPt1Found == false))
        {
          LeLU_Int_LookupX1 = LKeLU_Cmd_XAxis[LeLU_Int_Index];
          LeLU_Int_LookupY1 = LKLU_Cmd_TableData1D[LeLU_Int_Index];
          LeLU_Int_LookupX2 = LKeLU_Cmd_XAxis[LeLU_Int_Index + 1];
          LeLU_Int_LookupY2 = LKLU_Cmd_TableData1D[LeLU_Int_Index + 1];
          LeLU_b_LookUpPt1Found = true;

          LeLU_Int_Index = LeLU_Int_AxisSize;
        }
      }

      if ((LeLU_b_LookUpPt1Found == true))
      {
        LeLU_Int_LookupXDiff = LeLU_Int_LookupX2 - LeLU_Int_LookupX1;
        LeLU_Int_LookupYDiff = LeLU_Int_LookupY2 - LeLU_Int_LookupY1;
        if (LeLU_Int_LookupXDiff != 0.0)
        {
          /* Protect for zero division */
          LeLU_Int_LookupDiv = LeLU_Int_LookupYDiff / LeLU_Int_LookupXDiff;
        }
        else
        {
          LeLU_Int_LookupDiv = 0.0;
        }
        LeLU_Int_Output = LeLU_Int_LookupY1 + (LeLU_Cmd_Input - LeLU_Int_LookupX1) * LeLU_Int_LookupDiv;

        return LeLU_Int_Output;
      }
    }
  }

  // Not in range...
  return 0;
}

/******************************************************************************
 * Function:     RampTo
 *
 * Description:  Function to ramp from one value to another.
 *****************************************************************************/
double RampTo(double LeLU_Cmd_Final,
              double LeLU_Cmd_Current,
              double LeLU_Cmd_Slope)
{
  if (LeLU_Cmd_Final - LeLU_Cmd_Current > 0)
  {
    LeLU_Cmd_Current += LeLU_Cmd_Slope;

    if (LeLU_Cmd_Current >= LeLU_Cmd_Final)
    {
      LeLU_Cmd_Current = LeLU_Cmd_Final;
    }
  }
  else if (LeLU_Cmd_Final - LeLU_Cmd_Current < 0)
  {
    LeLU_Cmd_Current -= LeLU_Cmd_Slope;
    if (LeLU_Cmd_Current <= LeLU_Cmd_Final)
    {
      LeLU_Cmd_Current = LeLU_Cmd_Final;
    }
  }
  else
  {
    LeLU_Cmd_Current = LeLU_Cmd_Final;
  }
  return (LeLU_Cmd_Current);
}

/******************************************************************************
 * Function:     RampTo_2Ramp
 *
 * Description:  Function to ramp from one value to another. Will use a second 
 *               ramp when in deadband.
 *****************************************************************************/
double RampTo_2Ramp(double LeLU_Cmd_Final,
                    double LeLU_Cmd_Current,
                    double LeLU_Cmd_SlopeFast,
                    double LeLU_Cmd_SlopeSlow,
                    double LeLU_K_Db)
{
double LeLU_Cmd_DeltaToFinish = fabs(LeLU_Cmd_Final - LeLU_Cmd_Current);
double LeLU_Cmd_SlopeActual = 0;

  if (LeLU_Cmd_DeltaToFinish > LeLU_K_Db)
    {
      /* We have a large enough delta to remain in the fast slope */
      LeLU_Cmd_SlopeActual = LeLU_Cmd_SlopeFast;
    }
  else
    {
      LeLU_Cmd_SlopeActual = LeLU_Cmd_SlopeSlow;
    }

  if (LeLU_Cmd_Final - LeLU_Cmd_Current > 0)
  {
    LeLU_Cmd_Current += LeLU_Cmd_SlopeActual;

    if (LeLU_Cmd_Current >= LeLU_Cmd_Final)
    {
      LeLU_Cmd_Current = LeLU_Cmd_Final;
    }
  }
  else if (LeLU_Cmd_Final - LeLU_Cmd_Current < 0)
  {
    LeLU_Cmd_Current -= LeLU_Cmd_SlopeActual;
    if (LeLU_Cmd_Current <= LeLU_Cmd_Final)
    {
      LeLU_Cmd_Current = LeLU_Cmd_Final;
    }
  }
  else
  {
    LeLU_Cmd_Current = LeLU_Cmd_Final;
  }
  return (LeLU_Cmd_Current);
}

/******************************************************************************
 * Function:     LookUp1D_Axis
 *
 * Description:  Single axis lookup.
 ******************************************************************************/
void LookUp1D_Axis(const double *LKLU_Cmd_Axis,
                   int LeLU_Int_AxisSize,
                   int *LeLU_Int_Index_i, 
                   int *LeLU_Int_Index_j,
                   double LeLU_Cmd_Input,
                   double *LeLU_Cmd_InputScalar,
                   double *LeLU_Cmd_InputScalar1Minus)
{
  int LeLU_Int_Index = 0;
  bool LeLU_b_LookUpPt1Found = false;
  double LeLU_Int_Denomenator = 0.0;

  if (LeLU_Cmd_Input >= (LKLU_Cmd_Axis[LeLU_Int_AxisSize - 1]))
  {
    // We have gone off or are at the end of the axis
    *LeLU_Int_Index_i = LeLU_Int_AxisSize - 2;
    *LeLU_Int_Index_j = LeLU_Int_AxisSize - 1;
    *LeLU_Cmd_InputScalar = 1;
    *LeLU_Cmd_InputScalar1Minus = 0;
  }
  else if (LeLU_Cmd_Input <= (LKLU_Cmd_Axis[0]))
  {
    // We have gone off or are at the beginning of the axis
    *LeLU_Int_Index_i = 0;
    *LeLU_Int_Index_j = 1;
    *LeLU_Cmd_InputScalar = 0;
    *LeLU_Cmd_InputScalar1Minus = 1;
  }
  else
  {
    for (LeLU_Int_Index = 0; ((LeLU_Int_Index < (LeLU_Int_AxisSize - 1)) && (LeLU_b_LookUpPt1Found == false)); LeLU_Int_Index++)
    {
      if ((LeLU_Cmd_Input >= LKLU_Cmd_Axis[LeLU_Int_Index]) &&
          (LeLU_Cmd_Input < LKLU_Cmd_Axis[LeLU_Int_Index + 1]) &&
          (LeLU_b_LookUpPt1Found == false))
      {
        LeLU_b_LookUpPt1Found = true;
        *LeLU_Int_Index_i = LeLU_Int_Index;
        *LeLU_Int_Index_j = LeLU_Int_Index + 1;
        LeLU_Int_Denomenator = LKLU_Cmd_Axis[LeLU_Int_Index + 1] - LKLU_Cmd_Axis[LeLU_Int_Index];
        if (LeLU_Int_Denomenator != 0.0)
        {
          /* Protect for zero division */
          *LeLU_Cmd_InputScalar1Minus = (LKLU_Cmd_Axis[LeLU_Int_Index + 1] - LeLU_Cmd_Input) / (LeLU_Int_Denomenator);
        }
        else
        {
          *LeLU_Cmd_InputScalar1Minus = 1;
        }
        *LeLU_Cmd_InputScalar = 1 - *LeLU_Cmd_InputScalar1Minus;
      }
    }

    if (LeLU_b_LookUpPt1Found == false)
    {
      /* Defensive programming.  We really shouldn't reach here... */
      *LeLU_Int_Index_i = 0;
      *LeLU_Int_Index_j = 1;
      *LeLU_Cmd_InputScalar = 0;
      *LeLU_Cmd_InputScalar1Minus = 1 - *LeLU_Cmd_InputScalar;
    }
  }
}

/******************************************************************************
 * Function:     LookUp2D_Table
 *
 * Description:  Two dimension lookup table.  Based on the linear interpolation
 *               at the link below:
 *               https://en.wikipedia.org/wiki/Bilinear_interpolation
 ******************************************************************************/
double LookUp2D_Table(double const *LKeLU_Cmd_XAxis,
                      int LeLU_Cmd_XAxisSize,
                      double LeLU_Cmd_X_Input,
                      double const *LKeLU_Cmd_Y_Axis,
                      int LeLU_Cmd_Y_AxisSize,
                      double LeLU_Cmd_Y_Input,
                      double **LeLU_Cmd_TableData2D)
{
  int LeLU_Int_X_Index_i = 0;
  int LeLU_Int_X_Index_j = 0;
  double LeLU_Int_X_IndexScalar = 0.0;
  double LeLU_Int_X_IndexScalar1Minus = 0.0;
  int LeLU_Int_Y_Index_i = 0;
  int LeLU_Int_Y_Index_j = 0;
  double LeLU_Int_Y_IndexScalar = 0.0;
  double LeLU_Int_Y_IndexScalar1Minus = 0.0;
  double LeLU_Int_F_XiYi = 0.0;
  double LeLU_Int_F_XiYj = 0.0;
  double LeLU_Int_F_XjYi = 0.0;
  double LeLU_Int_F_XjYj = 0.0;
  double LeLU_Int_Output = 0.0;

  LookUp1D_Axis(&LKeLU_Cmd_XAxis[0],
                LeLU_Cmd_XAxisSize,
                &LeLU_Int_X_Index_i,
                &LeLU_Int_X_Index_j,
                LeLU_Cmd_X_Input,
                &LeLU_Int_X_IndexScalar,
                &LeLU_Int_X_IndexScalar1Minus);

  LookUp1D_Axis(&LKeLU_Cmd_Y_Axis[0],
                LeLU_Cmd_Y_AxisSize,
                &LeLU_Int_Y_Index_i,
                &LeLU_Int_Y_Index_j,
                LeLU_Cmd_Y_Input,
                &LeLU_Int_Y_IndexScalar,
                &LeLU_Int_Y_IndexScalar1Minus);

  LeLU_Int_F_XiYi = LeLU_Cmd_TableData2D[(int)LeLU_Int_X_Index_i][(int)LeLU_Int_Y_Index_i];
  LeLU_Int_F_XiYj = LeLU_Cmd_TableData2D[(int)LeLU_Int_X_Index_i][(int)LeLU_Int_Y_Index_j];
  LeLU_Int_F_XjYi = LeLU_Cmd_TableData2D[(int)LeLU_Int_X_Index_j][(int)LeLU_Int_Y_Index_i];
  LeLU_Int_F_XjYj = LeLU_Cmd_TableData2D[(int)LeLU_Int_X_Index_j][(int)LeLU_Int_Y_Index_j];

  LeLU_Int_Output = LeLU_Int_F_XiYi * LeLU_Int_X_IndexScalar1Minus * LeLU_Int_Y_IndexScalar1Minus +
                    LeLU_Int_F_XjYi * LeLU_Int_X_IndexScalar * LeLU_Int_Y_IndexScalar1Minus +
                    LeLU_Int_F_XiYj * LeLU_Int_X_IndexScalar1Minus * LeLU_Int_Y_IndexScalar +
                    LeLU_Int_F_XjYj * LeLU_Int_X_IndexScalar * LeLU_Int_Y_IndexScalar;

  return (LeLU_Int_Output);
}

/******************************************************************************
 * Function:     DesiredAutonLocation2
 *
 * Description:  Determine the desired X/Y location based on the current time.
 ******************************************************************************/
bool DesiredAutonLocation2(double LeLU_s_AutonTime,
                           T_ADAS_ActiveFeature LeADAS_e_ActiveFeature,
                           frc::DriverStation::Alliance LeLC_e_AllianceColor,
                           double *LeLU_Cmd_L_X_Location,
                           double *LeLU_Cmd_L_Y_Location,
                           double *LeLU_Cmd_Deg_Angle,
                           double *LeLU_Cmd_TimeRemaining)
{
  double LeLU_l_X_Loc = 0.0;
  double LeLU_l_Y_Loc = 0.0;
  double LeLU_Deg_Ang = 0.0;
  double LeLU_t_TimeRemaining = 0.0;
  int    LeLU_Int_X_AxisSize = 0;
  int    LeLU_Int_X_CalArraySize = 0;
  int    LeLU_Int_Y_AxisSize = 0;
  int    LeLU_Int_Y_CalArraySize = 0;
  int    LeLU_Int_Ang_AxisSize = 0;
  int    LeLU_Int_Ang_CalArraySize = 0;
  int    LeLU_Int_t_AxisSize = 0;
  int    LeLU_Int_t_CalArraySize = 0;
  bool   LeLU_b_timeTableDONE = false;
  double LeLU_k_RedMirrorDirectionFlip = 1;

  switch (LeADAS_e_ActiveFeature)
    {
      case E_ADAS_DM_PathFollower1:
        LeLU_Int_X_AxisSize = (int)(sizeof(KnADAS_t_BlueP1) / sizeof(KaADAS_l_BlueP1_X[0]));
        LeLU_Int_X_CalArraySize = (int)(sizeof(KaADAS_l_BlueP1_X) / sizeof(KaADAS_l_BlueP1_X[0]));
    
        LeLU_Int_Y_AxisSize = (int)(sizeof(KnADAS_t_BlueP1) / sizeof(KaADAS_l_BlueP1_Y[0]));
        LeLU_Int_Y_CalArraySize = (int)(sizeof(KaADAS_l_BlueP1_Y) / sizeof(KaADAS_l_BlueP1_Y[0]));
        
        LeLU_Int_Ang_AxisSize = (int)(sizeof(KnADAS_t_BlueP1) / sizeof(KaADAS_Deg_BlueP1[0]));
        LeLU_Int_Ang_CalArraySize = (int)(sizeof(KaADAS_Deg_BlueP1) / sizeof(KaADAS_Deg_BlueP1[0]));
    
        LeLU_Int_t_AxisSize = (int)(sizeof(KnADAS_t_BlueP1) / sizeof(KaADAS_t_BlueP1Remaining[0]));
        LeLU_Int_t_CalArraySize = (int)(sizeof(KaADAS_t_BlueP1Remaining) / sizeof(KaADAS_t_BlueP1Remaining[0]));
    
        LeLU_l_X_Loc = LookUp1D_Table(&KnADAS_t_BlueP1[0],
                                      &KaADAS_l_BlueP1_X[0],
                                      LeLU_Int_X_AxisSize,
                                      LeLU_Int_X_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_l_Y_Loc = LookUp1D_Table(&KnADAS_t_BlueP1[0],
                                      &KaADAS_l_BlueP1_Y[0],
                                      LeLU_Int_Y_AxisSize,
                                      LeLU_Int_Y_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_Deg_Ang = LookUp1D_Table(&KnADAS_t_BlueP1[0],
                                      &KaADAS_Deg_BlueP1[0],
                                      LeLU_Int_Ang_AxisSize,
                                      LeLU_Int_Ang_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_t_TimeRemaining = LookUp1D_Table(&KnADAS_t_BlueP1[0],
                                              &KaADAS_t_BlueP1Remaining[0],
                                              LeLU_Int_t_AxisSize,
                                              LeLU_Int_t_CalArraySize,
                                              LeLU_s_AutonTime);
    
        if (LeLU_s_AutonTime >= KnADAS_t_BlueP1[LeLU_Int_X_AxisSize - 1])
        {
          LeLU_b_timeTableDONE = true;
        }
      break;
      case E_ADAS_DM_PathFollower2:
        LeLU_Int_X_AxisSize = (int)(sizeof(KnADAS_t_BlueP2) / sizeof(KaADAS_l_BlueP2_X[0]));
        LeLU_Int_X_CalArraySize = (int)(sizeof(KaADAS_l_BlueP2_X) / sizeof(KaADAS_l_BlueP2_X[0]));
    
        LeLU_Int_Y_AxisSize = (int)(sizeof(KnADAS_t_BlueP2) / sizeof(KaADAS_l_BlueP2_Y[0]));
        LeLU_Int_Y_CalArraySize = (int)(sizeof(KaADAS_l_BlueP2_Y) / sizeof(KaADAS_l_BlueP2_Y[0]));
        
        LeLU_Int_Ang_AxisSize = (int)(sizeof(KnADAS_t_BlueP2) / sizeof(KaADAS_Deg_BlueP2[0]));
        LeLU_Int_Ang_CalArraySize = (int)(sizeof(KaADAS_Deg_BlueP2) / sizeof(KaADAS_Deg_BlueP2[0]));
    
        LeLU_Int_t_AxisSize = (int)(sizeof(KnADAS_t_BlueP2) / sizeof(KaADAS_t_BlueP2Remaining[0]));
        LeLU_Int_t_CalArraySize = (int)(sizeof(KaADAS_t_BlueP2Remaining) / sizeof(KaADAS_t_BlueP2Remaining[0]));
    
        LeLU_l_X_Loc = LookUp1D_Table(&KnADAS_t_BlueP2[0],
                                      &KaADAS_l_BlueP2_X[0],
                                      LeLU_Int_X_AxisSize,
                                      LeLU_Int_X_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_l_Y_Loc = LookUp1D_Table(&KnADAS_t_BlueP2[0],
                                      &KaADAS_l_BlueP2_Y[0],
                                      LeLU_Int_Y_AxisSize,
                                      LeLU_Int_Y_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_Deg_Ang = LookUp1D_Table(&KnADAS_t_BlueP2[0],
                                      &KaADAS_Deg_BlueP2[0],
                                      LeLU_Int_Ang_AxisSize,
                                      LeLU_Int_Ang_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_t_TimeRemaining = LookUp1D_Table(&KnADAS_t_BlueP2[0],
                                              &KaADAS_t_BlueP2Remaining[0],
                                              LeLU_Int_t_AxisSize,
                                              LeLU_Int_t_CalArraySize,
                                              LeLU_s_AutonTime);
    
        if (LeLU_s_AutonTime >= KnADAS_t_BlueP2[LeLU_Int_X_AxisSize - 1])
        {
          LeLU_b_timeTableDONE = true;
        }
      break;
      case E_ADAS_DM_PathFollower3:
        LeLU_Int_X_AxisSize = (int)(sizeof(KnADAS_t_BlueP3) / sizeof(KaADAS_l_BlueP3_X[0]));
        LeLU_Int_X_CalArraySize = (int)(sizeof(KaADAS_l_BlueP3_X) / sizeof(KaADAS_l_BlueP3_X[0]));
    
        LeLU_Int_Y_AxisSize = (int)(sizeof(KnADAS_t_BlueP3) / sizeof(KaADAS_l_BlueP3_Y[0]));
        LeLU_Int_Y_CalArraySize = (int)(sizeof(KaADAS_l_BlueP3_Y) / sizeof(KaADAS_l_BlueP3_Y[0]));
        
        LeLU_Int_Ang_AxisSize = (int)(sizeof(KnADAS_t_BlueP3) / sizeof(KaADAS_Deg_BlueP3[0]));
        LeLU_Int_Ang_CalArraySize = (int)(sizeof(KaADAS_Deg_BlueP3) / sizeof(KaADAS_Deg_BlueP3[0]));
    
        LeLU_Int_t_AxisSize = (int)(sizeof(KnADAS_t_BlueP3) / sizeof(KaADAS_t_BlueP3Remaining[0]));
        LeLU_Int_t_CalArraySize = (int)(sizeof(KaADAS_t_BlueP3Remaining) / sizeof(KaADAS_t_BlueP3Remaining[0]));
    
        LeLU_l_X_Loc = LookUp1D_Table(&KnADAS_t_BlueP3[0],
                                      &KaADAS_l_BlueP3_X[0],
                                      LeLU_Int_X_AxisSize,
                                      LeLU_Int_X_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_l_Y_Loc = LookUp1D_Table(&KnADAS_t_BlueP3[0],
                                      &KaADAS_l_BlueP3_Y[0],
                                      LeLU_Int_Y_AxisSize,
                                      LeLU_Int_Y_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_Deg_Ang = LookUp1D_Table(&KnADAS_t_BlueP3[0],
                                      &KaADAS_Deg_BlueP3[0],
                                      LeLU_Int_Ang_AxisSize,
                                      LeLU_Int_Ang_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_t_TimeRemaining = LookUp1D_Table(&KnADAS_t_BlueP3[0],
                                              &KaADAS_t_BlueP3Remaining[0],
                                              LeLU_Int_t_AxisSize,
                                              LeLU_Int_t_CalArraySize,
                                              LeLU_s_AutonTime);
    
        if (LeLU_s_AutonTime >= KnADAS_t_BlueP3[LeLU_Int_X_AxisSize - 1])
        {
          LeLU_b_timeTableDONE = true;
        }
      break;
      case E_ADAS_DM_PathFollower4:
        LeLU_Int_X_AxisSize = (int)(sizeof(KnADAS_t_BlueP4) / sizeof(KaADAS_l_BlueP4_X[0]));
        LeLU_Int_X_CalArraySize = (int)(sizeof(KaADAS_l_BlueP4_X) / sizeof(KaADAS_l_BlueP4_X[0]));
    
        LeLU_Int_Y_AxisSize = (int)(sizeof(KnADAS_t_BlueP4) / sizeof(KaADAS_l_BlueP4_Y[0]));
        LeLU_Int_Y_CalArraySize = (int)(sizeof(KaADAS_l_BlueP4_Y) / sizeof(KaADAS_l_BlueP4_Y[0]));
        
        LeLU_Int_Ang_AxisSize = (int)(sizeof(KnADAS_t_BlueP4) / sizeof(KaADAS_Deg_BlueP4[0]));
        LeLU_Int_Ang_CalArraySize = (int)(sizeof(KaADAS_Deg_BlueP4) / sizeof(KaADAS_Deg_BlueP4[0]));
    
        LeLU_Int_t_AxisSize = (int)(sizeof(KnADAS_t_BlueP4) / sizeof(KaADAS_t_BlueP4Remaining[0]));
        LeLU_Int_t_CalArraySize = (int)(sizeof(KaADAS_t_BlueP4Remaining) / sizeof(KaADAS_t_BlueP4Remaining[0]));
    
        LeLU_l_X_Loc = LookUp1D_Table(&KnADAS_t_BlueP4[0],
                                      &KaADAS_l_BlueP4_X[0],
                                      LeLU_Int_X_AxisSize,
                                      LeLU_Int_X_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_l_Y_Loc = LookUp1D_Table(&KnADAS_t_BlueP4[0],
                                      &KaADAS_l_BlueP4_Y[0],
                                      LeLU_Int_Y_AxisSize,
                                      LeLU_Int_Y_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_Deg_Ang = LookUp1D_Table(&KnADAS_t_BlueP4[0],
                                      &KaADAS_Deg_BlueP4[0],
                                      LeLU_Int_Ang_AxisSize,
                                      LeLU_Int_Ang_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_t_TimeRemaining = LookUp1D_Table(&KnADAS_t_BlueP4[0],
                                              &KaADAS_t_BlueP4Remaining[0],
                                              LeLU_Int_t_AxisSize,
                                              LeLU_Int_t_CalArraySize,
                                              LeLU_s_AutonTime);
    
        if (LeLU_s_AutonTime >= KnADAS_t_BlueP4[LeLU_Int_X_AxisSize - 1])
        {
          LeLU_b_timeTableDONE = true;
        }
      break;
      case E_ADAS_DM_PathFollowerFWD:
        LeLU_Int_X_AxisSize = (int)(sizeof(KnADAS_t_BlueFWD) / sizeof(KaADAS_l_BlueFWD_X[0]));
        LeLU_Int_X_CalArraySize = (int)(sizeof(KaADAS_l_BlueFWD_X) / sizeof(KaADAS_l_BlueFWD_X[0]));
    
        LeLU_Int_Y_AxisSize = (int)(sizeof(KnADAS_t_BlueFWD) / sizeof(KaADAS_l_BlueFWD_Y[0]));
        LeLU_Int_Y_CalArraySize = (int)(sizeof(KaADAS_l_BlueFWD_Y) / sizeof(KaADAS_l_BlueFWD_Y[0]));
        
        LeLU_Int_Ang_AxisSize = (int)(sizeof(KnADAS_t_BlueFWD) / sizeof(KaADAS_Deg_BlueFWD[0]));
        LeLU_Int_Ang_CalArraySize = (int)(sizeof(KaADAS_Deg_BlueFWD) / sizeof(KaADAS_Deg_BlueFWD[0]));
    
        LeLU_Int_t_AxisSize = (int)(sizeof(KnADAS_t_BlueFWD) / sizeof(KaADAS_t_BlueFWDRemaining[0]));
        LeLU_Int_t_CalArraySize = (int)(sizeof(KaADAS_t_BlueFWDRemaining) / sizeof(KaADAS_t_BlueFWDRemaining[0]));
    
        LeLU_l_X_Loc = LookUp1D_Table(&KnADAS_t_BlueFWD[0],
                                      &KaADAS_l_BlueFWD_X[0],
                                      LeLU_Int_X_AxisSize,
                                      LeLU_Int_X_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_l_Y_Loc = LookUp1D_Table(&KnADAS_t_BlueFWD[0],
                                      &KaADAS_l_BlueFWD_Y[0],
                                      LeLU_Int_Y_AxisSize,
                                      LeLU_Int_Y_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_Deg_Ang = LookUp1D_Table(&KnADAS_t_BlueFWD[0],
                                      &KaADAS_Deg_BlueFWD[0],
                                      LeLU_Int_Ang_AxisSize,
                                      LeLU_Int_Ang_CalArraySize,
                                      LeLU_s_AutonTime);
    
        LeLU_t_TimeRemaining = LookUp1D_Table(&KnADAS_t_BlueFWD[0],
                                              &KaADAS_t_BlueP4Remaining[0],
                                              LeLU_Int_t_AxisSize,
                                              LeLU_Int_t_CalArraySize,
                                              LeLU_s_AutonTime);
    
        if (LeLU_s_AutonTime >= KnADAS_t_BlueFWD[LeLU_Int_X_AxisSize - 1])
        {
          LeLU_b_timeTableDONE = true;
        }
      break;
      default:  
      break;
    }

  if (LeLC_e_AllianceColor == frc::DriverStation::Alliance::kRed)
    {
      // Need to flip X and Angle for Red side:
      LeLU_l_X_Loc *= -1;
      LeLU_Deg_Ang *= -1;
    }

  *LeLU_Cmd_L_X_Location = LeLU_l_X_Loc;
  *LeLU_Cmd_L_Y_Location = LeLU_l_Y_Loc;
  *LeLU_Cmd_Deg_Angle = LeLU_Deg_Ang;
  *LeLU_Cmd_TimeRemaining = LeLU_t_TimeRemaining;

  return (LeLU_b_timeTableDONE);
}

/******************************************************************************
 * Function:     ScaleJoystickAxis
 *
 * Description:  Function to scale the joystick input.
 *               Primarily used for smooth debouncing.
 ******************************************************************************/
double ScaleJoystickAxis(double LeLU_Cmd_JoystickAxis)
{
  double LeLU_RPM_DesiredDriveSpeed = 0.0; // may or may not be RPM
  int LeLU_Int_AxisSize = (int)(sizeof(K_SD_DesiredDriveSpeedAxis) / sizeof(K_SD_DesiredDriveSpeed[0]));
  int LaLU_CalArraySize = (int)(sizeof(K_SD_DesiredDriveSpeed) / sizeof(K_SD_DesiredDriveSpeed[0]));

  LeLU_RPM_DesiredDriveSpeed = LookUp1D_Table(&K_SD_DesiredDriveSpeedAxis[0],
                                              &K_SD_DesiredDriveSpeed[0],
                                              LeLU_Int_AxisSize,
                                              LaLU_CalArraySize,
                                              LeLU_Cmd_JoystickAxis);

  return LeLU_RPM_DesiredDriveSpeed;
}

/******************************************************************************
 * Function:     ScaleJoystickRotateAxis
 *
 * Description:  Function to scale the joystick input.
 *               Primarily used for smooth debouncing.
 ******************************************************************************/
double ScaleJoystickRotateAxis(double LeLU_Cmd_JoystickAxis)
{
  double LeLU_RPM_DesiredDriveSpeed = 0.0; // may or may not be RPM
  int LeLU_Int_AxisSize = (int)(sizeof(K_SD_DesiredRotateAxis) / sizeof(K_SD_DesiredRotate[0]));
  int LaLU_CalArraySize = (int)(sizeof(K_SD_DesiredRotate) / sizeof(K_SD_DesiredRotate[0]));

  LeLU_RPM_DesiredDriveSpeed = LookUp1D_Table(&K_SD_DesiredRotateAxis[0],
                                              &K_SD_DesiredRotate[0],
                                              LeLU_Int_AxisSize,
                                              LaLU_CalArraySize,
                                              LeLU_Cmd_JoystickAxis);

  return LeLU_RPM_DesiredDriveSpeed;
}

/******************************************************************************
 * Function:     ScaleAccelAxis
 *
 * Description:  Function to scale the joystick input.
 *               Primarily used for smooth debouncing.
 ******************************************************************************/
double ScaleAccelAxis(double LeLU_Cmd_JoystickAxis)
{
  double LeLU_K_AccelScaler = 0.0;
  int LeLU_i_AxisSize = (int)(sizeof(KnLU_k_SD_DesiredAccelAxis) / sizeof(KtLU_k_SD_DesiredAccel[0]));
  int LeLU_i_CalArraySize = (int)(sizeof(KtLU_k_SD_DesiredAccel) / sizeof(KtLU_k_SD_DesiredAccel[0]));

  LeLU_K_AccelScaler = LookUp1D_Table(&KnLU_k_SD_DesiredAccelAxis[0],
                                      &KtLU_k_SD_DesiredAccel[0],
                                       LeLU_i_AxisSize,
                                       LeLU_i_CalArraySize,
                                       LeLU_Cmd_JoystickAxis);

  return LeLU_K_AccelScaler;
}

/******************************************************************************
 * Function:     DesiredRotateSpeed
 *
 * Description:  Function to determine the speed at which to rotate the robot
 *               (for auto targeting and auto rotate).
 ******************************************************************************/
double DesiredRotateSpeed(double LeLU_Cmd_Error)
{
  double LeLU_RPM_DesiredRotateSpeed = 0.0; // may or may not be RPM
  int LeLU_Int_AxisSize = (int)(sizeof(K_DesiredRotateSpeedAxis) / sizeof(K_DesiredRotateSpeed[0]));
  int LaLU_CalArraySize = (int)(sizeof(K_DesiredRotateSpeed) / sizeof(K_DesiredRotateSpeed[0]));

  LeLU_RPM_DesiredRotateSpeed = LookUp1D_Table(&K_DesiredRotateSpeedAxis[0],
                                               &K_DesiredRotateSpeed[0],
                                               LeLU_Int_AxisSize,
                                               LaLU_CalArraySize,
                                               LeLU_Cmd_Error);

  return LeLU_RPM_DesiredRotateSpeed;
}

/******************************************************************************
 * Function:     DesiredAutoRotateSpeed
 *
 * Description:  Function to determine the speed at which to rotate the robot
 *               (for auto targeting and auto rotate).
 ******************************************************************************/
double DesiredAutoRotateSpeed(double LeLU_Cmd_Error)
{
  double LeLU_RPM_DesiredRotateSpeed = 0.0;
  int LeLU_Int_AxisSize = (int)(sizeof(K_DesiredAutoRotateSpeedAxis) / sizeof(K_DesiredAutoRotateSpeed[0]));
  int LaLU_CalArraySize = (int)(sizeof(K_DesiredAutoRotateSpeed) / sizeof(K_DesiredAutoRotateSpeed[0]));

  LeLU_RPM_DesiredRotateSpeed = LookUp1D_Table(&K_DesiredAutoRotateSpeedAxis[0],
                                               &K_DesiredAutoRotateSpeed[0],
                                               LeLU_Int_AxisSize,
                                               LaLU_CalArraySize,
                                               LeLU_Cmd_Error);

  return LeLU_RPM_DesiredRotateSpeed;
}
