/*
  Encoders.cpp

  Created on: Jan 3, 2020
  Author: 5561

  Converted to 2024 code: January 17, 2024
 */

#include "Amp.hpp"
#include "rev/CANSparkMax.h"
#include "Const.hpp"
#include "SpeakerCntrl.hpp"
#include <frc/smartdashboard/SmartDashboard.h>

double VaENC_Deg_WheelAngleConverted[E_RobotCornerSz]; // This is the wheel angle coming from the angle Encoder and processed to only be from 0 - 180
double VaENC_Deg_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in degrees
double VaENC_Rad_WheelAngleFwd[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a forward direction, in radians
double VaENC_Deg_WheelAngleRev[E_RobotCornerSz]; // This is the wheel angle as if the wheel were going to be driven in a reverse direction

double VaENC_InS_WheelVelocity[E_RobotCornerSz]; // Velocity of drive wheels, in in/sec
double VaENC_In_WheelDeltaDistance[E_RobotCornerSz]; // Distance wheel moved, loop to loop, in inches
double VaENC_Cnt_WheelDeltaDistanceCurr[E_RobotCornerSz]; // Current distance wheel moved, loop to loop, in Counts
double VaENC_Cnt_WheelDeltaDistancePrev[E_RobotCornerSz]; // Prev distance wheel moved, loop to loop, in Counts

/******************************************************************************
 * Function:     EncodersInitSwerve
 * 
 * Description:  Initialize all of the applicable encoders for our Swerve drive.
 ******************************************************************************/
void EncodersInitSwerve(rev::SparkRelativeEncoder m_encoderFrontRightSteer,
                        rev::SparkRelativeEncoder m_encoderFrontLeftSteer,
                        rev::SparkRelativeEncoder m_encoderRearRightSteer,
                        rev::SparkRelativeEncoder m_encoderRearLeftSteer,
                        rev::SparkRelativeEncoder m_encoderFrontRightDrive,
                        rev::SparkRelativeEncoder m_encoderFrontLeftDrive,
                        rev::SparkRelativeEncoder m_encoderRearRightDrive,
                        rev::SparkRelativeEncoder m_encoderRearLeftDrive)
  {
    T_RobotCorner LeENC_e_Index;

    for (LeENC_e_Index = E_FrontLeft;
         LeENC_e_Index < E_RobotCornerSz;
         LeENC_e_Index = T_RobotCorner(int(LeENC_e_Index) + 1))
      {
        VaENC_Deg_WheelAngleFwd[LeENC_e_Index] = 0;
        VaENC_Rad_WheelAngleFwd[LeENC_e_Index] = 0;
        VaENC_InS_WheelVelocity[LeENC_e_Index] = 0;
        VaENC_In_WheelDeltaDistance[LeENC_e_Index] = 0;
        VaENC_Cnt_WheelDeltaDistanceCurr[LeENC_e_Index] = 0;
        VaENC_Cnt_WheelDeltaDistancePrev[LeENC_e_Index] = 0;
      }

    m_encoderFrontRightSteer.SetPosition(0);
    m_encoderFrontLeftSteer.SetPosition(0);
    m_encoderRearRightSteer.SetPosition(0);
    m_encoderRearLeftSteer.SetPosition(0);

    m_encoderFrontRightDrive.SetPosition(0);
    m_encoderFrontLeftDrive.SetPosition(0);
    m_encoderRearRightDrive.SetPosition(0);
    m_encoderRearLeftDrive.SetPosition(0);




  }

/******************************************************************************
 * Function:     Encoders_Drive_CompBot
 *
 * Description:  Run all of the encoder decoding logic that makes Swerve work.
 ******************************************************************************/
void Encoders_Drive_CompBot(units::degree_t                       LeENC_Cnt_EncoderWheelAngleFrontLeftRaw,
                            units::degree_t                       LeENC_Cnt_EncoderWheelAngleFrontRightRaw,
                            units::degree_t                       LeENC_Cnt_EncoderWheelAngleRearLeftRaw,
                            units::degree_t                       LeENC_Cnt_EncoderWheelAngleRearRightRaw,
                            rev::SparkRelativeEncoder m_encoderFrontLeftDrive,
                            rev::SparkRelativeEncoder m_encoderFrontRightDrive,
                            rev::SparkRelativeEncoder m_encoderRearLeftDrive,
                            rev::SparkRelativeEncoder m_encoderRearRightDrive)
  {
  T_RobotCorner LeENC_e_Index;

  


  VaENC_Deg_WheelAngleConverted[E_FrontLeft]  = std::fmod((LeENC_Cnt_EncoderWheelAngleFrontLeftRaw.value()), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_FrontLeft];
  VaENC_Deg_WheelAngleConverted[E_FrontRight] = std::fmod((LeENC_Cnt_EncoderWheelAngleFrontRightRaw.value()), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_FrontRight];
  VaENC_Deg_WheelAngleConverted[E_RearLeft]   = std::fmod((LeENC_Cnt_EncoderWheelAngleRearLeftRaw.value()), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_RearLeft];
  VaENC_Deg_WheelAngleConverted[E_RearRight]  = std::fmod((LeENC_Cnt_EncoderWheelAngleRearRightRaw.value()), 360) - KeENC_Deg_SD_WheelOffsetAngle[E_RearRight];

  // frc::SmartDashboard::PutNumber("WA FL", VaENC_Deg_WheelAngleConverted[E_FrontLeft]);
  // frc::SmartDashboard::PutNumber("WA FR", VaENC_Deg_WheelAngleConverted[E_FrontRight]);
  // frc::SmartDashboard::PutNumber("WA RL", VaENC_Deg_WheelAngleConverted[E_RearLeft]);
  // frc::SmartDashboard::PutNumber("WA RR", VaENC_Deg_WheelAngleConverted[E_RearRight]);


  // frc::SmartDashboard::PutNumber("WS FL", m_encoderFrontLeftDrive.GetVelocity());
  // frc::SmartDashboard::PutNumber("WS FR", m_encoderFrontRightDrive.GetVelocity());
  // frc::SmartDashboard::PutNumber("WS RL", m_encoderRearLeftDrive.GetVelocity());
  // frc::SmartDashboard::PutNumber("WS RR", m_encoderRearRightDrive.GetVelocity());



  

  // frc::SmartDashboard::PutNumber("raw WA FL", LeENC_Cnt_EncoderWheelAngleFrontLeftRaw.value());
  // frc::SmartDashboard::PutNumber("Raw WA FR", LeENC_Cnt_EncoderWheelAngleFrontRightRaw.value());
  // frc::SmartDashboard::PutNumber("Raw WA RL", LeENC_Cnt_EncoderWheelAngleRearLeftRaw.value());
  // frc::SmartDashboard::PutNumber("raw WA RR", LeENC_Cnt_EncoderWheelAngleRearRightRaw.value());


  VaENC_Cnt_WheelDeltaDistanceCurr[E_FrontLeft]  = m_encoderFrontLeftDrive.GetPosition();
  VaENC_Cnt_WheelDeltaDistanceCurr[E_FrontRight] = m_encoderFrontRightDrive.GetPosition();
  VaENC_Cnt_WheelDeltaDistanceCurr[E_RearRight]  = m_encoderRearRightDrive.GetPosition();
  VaENC_Cnt_WheelDeltaDistanceCurr[E_RearLeft]   = m_encoderRearLeftDrive.GetPosition();

  for (LeENC_e_Index = E_FrontLeft;
       LeENC_e_Index < E_RobotCornerSz;
       LeENC_e_Index = T_RobotCorner(int(LeENC_e_Index) + 1))
    {
    VaENC_Deg_WheelAngleFwd[LeENC_e_Index] = VaENC_Deg_WheelAngleConverted[LeENC_e_Index];

    if (VaENC_Deg_WheelAngleFwd[LeENC_e_Index] > 180)
      {
      VaENC_Deg_WheelAngleFwd[LeENC_e_Index] -= 360;
      }
    else if (VaENC_Deg_WheelAngleFwd[LeENC_e_Index] < -180)
      {
      VaENC_Deg_WheelAngleFwd[LeENC_e_Index] += 360;
      }

    /* Now we need to find the equivalent angle as if the wheel were going to be driven in the opposite direction, i.e. in reverse */
    if (VaENC_Deg_WheelAngleFwd[LeENC_e_Index] >= 0)
      {
      VaENC_Deg_WheelAngleRev[LeENC_e_Index] = VaENC_Deg_WheelAngleFwd[LeENC_e_Index] - 180;
      }
    else
      {
      VaENC_Deg_WheelAngleRev[LeENC_e_Index] = VaENC_Deg_WheelAngleFwd[LeENC_e_Index] + 180;
      }
    /* Create a copy of the Angle Fwd, but in radians */
    VaENC_Rad_WheelAngleFwd[LeENC_e_Index] = VaENC_Deg_WheelAngleFwd[LeENC_e_Index] * (C_PI/180);

    VaENC_In_WheelDeltaDistance[LeENC_e_Index]  = ((((VaENC_Cnt_WheelDeltaDistanceCurr[LeENC_e_Index]  - VaENC_Cnt_WheelDeltaDistancePrev[LeENC_e_Index])/  KeENC_k_ReductionRatio)) * KeENC_In_WheelCircumfrence );
    VaENC_Cnt_WheelDeltaDistancePrev[LeENC_e_Index]  = VaENC_Cnt_WheelDeltaDistanceCurr[LeENC_e_Index];
    }

  VaENC_InS_WheelVelocity[E_FrontLeft]  = ((m_encoderFrontLeftDrive.GetVelocity()  / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_FrontRight] = ((m_encoderFrontRightDrive.GetVelocity() / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_RearRight]  = ((m_encoderRearRightDrive.GetVelocity()  / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  VaENC_InS_WheelVelocity[E_RearLeft]   = ((m_encoderRearLeftDrive.GetVelocity()   / KeENC_k_ReductionRatio) / 60) * KeENC_In_WheelCircumfrence;
  }


/******************************************************************************
 * Function:     Encoders_DJ_Amp_INT
 *
 * Description:  Read the encoders from the Amp mech
 ******************************************************************************/
void Encoders_DJ_Amp_INT( rev::SparkMaxRelativeEncoder m_WristEncoder,
                       double                       LeENC_Deg_Elevator,
                       T_MotorControlType           LeENC_e_IntakeCmnd,
                       bool                         LeENC_b_WristReverseLimit)
  {
  bool LeENC_b_IntakeExtended = false;
  bool LeENC_b_ObjectDetected = false;

  VsAmp_s_Sensors.Deg_Wrist = m_WristEncoder.GetPosition() * KeENC_Deg_Wrist;

  VsAmp_s_Sensors.In_Elevator = LeENC_Deg_Elevator * KeENC_k_ElevatorEncoderScaler;

  if (LeENC_e_IntakeCmnd == E_MotorExtend)
    {
    LeENC_b_IntakeExtended = true;
    }

  /* Switches are wired to the wrist motor controller.  Switches are intended to detect object in the gripper... */
  if (LeENC_b_WristReverseLimit == false)
    {
      LeENC_b_ObjectDetected = true;
    }

  VsAmp_s_Sensors.b_Amp_ObjDetected = LeENC_b_ObjectDetected;

  frc::SmartDashboard::PutNumber("Wrist",          VsAmp_s_Sensors.Deg_Wrist);
  frc::SmartDashboard::PutNumber("LinearSlide",    VsAmp_s_Sensors.In_Elevator);
  }
