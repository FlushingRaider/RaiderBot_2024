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

void Encoders_Drive_CompBot(units::degree_t                 LeENC_Cnt_EncoderWheelAngleFrontLeftRaw,
                             units::degree_t                LeENC_Cnt_EncoderWheelAngleFrontRightRaw,
                             units::degree_t                LeENC_Cnt_EncoderWheelAngleRearLeftRaw,
                             units::degree_t                LeENC_Cnt_EncoderWheelAngleRearRightRaw,
                             rev::SparkRelativeEncoder m_encoderFrontLeftDrive,
                             rev::SparkRelativeEncoder m_encoderFrontRightDrive,
                             rev::SparkRelativeEncoder m_encoderRearLeftDrive,
                             rev::SparkRelativeEncoder m_encoderRearRightDrive);

void EncodersInitSwerve(rev::SparkRelativeEncoder m_encoderFrontRightSteer,
                        rev::SparkRelativeEncoder m_encoderFrontLeftSteer,
                        rev::SparkRelativeEncoder m_encoderRearRightSteer,
                        rev::SparkRelativeEncoder m_encoderRearLeftSteer,
                        rev::SparkRelativeEncoder m_encoderFrontRightDrive,
                        rev::SparkRelativeEncoder m_encoderFrontLeftDrive,
                        rev::SparkRelativeEncoder m_encoderRearRightDrive,
                        rev::SparkRelativeEncoder m_encoderRearLeftDrive);

void Encoders_AMP_SPK_CLMR_Init( rev::SparkMaxRelativeEncoder m_encoderElevator,
                                 rev::SparkMaxRelativeEncoder m_encoderClimberLeft,
                                 rev::SparkMaxRelativeEncoder m_encoderClimberRight,
                                 rev::SparkMaxRelativeEncoder m_encoderWrist,
                                 rev::SparkMaxRelativeEncoder m_encoderIntake,
                                 rev::SparkMaxRelativeEncoder m_encoderUnderbelly,
                                 rev::SparkMaxRelativeEncoder m_encoderIAssist,
                                 rev::SparkMaxRelativeEncoder m_encoderShooter1,
                                 rev::SparkMaxRelativeEncoder m_encoderShooter2);

void Encoders_AMP_ResetWrist(rev::SparkMaxRelativeEncoder m_encoderWrist,
                             bool LeENC_b_ResetReq);

void Encoders_AMP_SPK_CLMR_Run( bool                         LeENC_b_AMP_IntakeLimit,
                                bool                         LeENC_b_AMP_ElevatorLimit,
                                bool                         LeENC_b_SPK_IntakeLimit,
                                rev::SparkMaxRelativeEncoder m_encoderElevator,
                                rev::SparkMaxRelativeEncoder m_encoderClimberLeft,
                                rev::SparkMaxRelativeEncoder m_encoderClimberRight,
                                rev::SparkMaxRelativeEncoder m_encoderWrist,
                                rev::SparkMaxRelativeEncoder m_encoderIntake,
                                rev::SparkMaxRelativeEncoder m_encoderUnderbelly,
                                rev::SparkMaxRelativeEncoder m_encoderIAssist,
                                rev::SparkMaxRelativeEncoder m_encoderShooter1,
                                rev::SparkMaxRelativeEncoder m_encoderShooter2);