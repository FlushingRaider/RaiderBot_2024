/*
  Encoders.hpp

  Created on: Feb 25, 2020

  Author: 5561

  Updates:
  2022-02-15: Cleaned up file

  Coverted to 2024 code: January 17,2024
*/

extern double VaENC_Deg_WheelAngleFwd[E_RobotCornerSz];
extern double VaENC_Rad_WheelAngleFwd[E_RobotCornerSz]; 
extern double VaENC_Deg_WheelAngleRev[E_RobotCornerSz];
extern double VaENC_Deg_WheelAngleConverted[E_RobotCornerSz];
extern double VaENC_In_WheelDeltaDistance[E_RobotCornerSz];

void Encoders_Drive_CompBot(double                       LeENC_Cnt_EncoderWheelAngleFrontLeftRaw,
                             double                       LeENC_Cnt_EncoderWheelAngleFrontRightRaw,
                             double                       LeENC_Cnt_EncoderWheelAngleRearLeftRaw,
                             double                       LeENC_Cnt_EncoderWheelAngleRearRightRaw,
                             rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                             rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                             rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive,
                             rev::SparkMaxRelativeEncoder m_encoderRearRightDrive);

void EncodersInitSwerve(rev::SparkMaxRelativeEncoder m_encoderFrontRightSteer,
                        rev::SparkMaxRelativeEncoder m_encoderFrontLeftSteer,
                        rev::SparkMaxRelativeEncoder m_encoderRearRightSteer,
                        rev::SparkMaxRelativeEncoder m_encoderRearLeftSteer,
                        rev::SparkMaxRelativeEncoder m_encoderFrontRightDrive,
                        rev::SparkMaxRelativeEncoder m_encoderFrontLeftDrive,
                        rev::SparkMaxRelativeEncoder m_encoderRearRightDrive,
                        rev::SparkMaxRelativeEncoder m_encoderRearLeftDrive);

