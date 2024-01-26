/*
Team 5561 Shuffleboard PID code

Writen by - Chris & Wes 2024

*/

#include <frc/shuffleboard/Shuffleboard.h>
#include "Shuffleboard.hpp"
#include <networktables/NetworkTable.h>

Shuffleboard_PID shuffleboard_FrontleftPID;
Shuffleboard_PID shuffleboard_SteerFrontleftPID;

void shuffleboard_init(){
frc::ShuffleboardTab& Front_Left = frc::Shuffleboard::GetTab("Front Left");
nt::GenericEntry* P_FrontLeft_entry = Front_Left.Add("P", 0.0).GetEntry();
nt::GenericEntry* I_FrontLeft_entry = Front_Left.Add("I", 0.0).GetEntry();
nt::GenericEntry* D_FrontLeft_entry = Front_Left.Add("D", 0.0).GetEntry();
nt::GenericEntry* P_LL_FrontLeft_entry = Front_Left.Add("P LL", 0.0).GetEntry();
nt::GenericEntry* P_UL_FrontLeft_entry = Front_Left.Add("P UL", 0.0).GetEntry();
nt::GenericEntry* I_LL_FrontLeft_entry = Front_Left.Add("I LL", 0.0).GetEntry();
nt::GenericEntry* I_UL_FrontLeft_entry = Front_Left.Add("I UL", 0.0).GetEntry();
nt::GenericEntry* D_LL_FrontLeft_entry = Front_Left.Add("D LL", 0.0).GetEntry();
nt::GenericEntry* D_UL_FrontLeft_entry = Front_Left.Add("D UL", 0.0).GetEntry();
nt::GenericEntry* UL_FrontLeft_entry = Front_Left.Add("UL", 0.0).GetEntry();
nt::GenericEntry* LL_FrontLeft_entry = Front_Left.Add("LL", 0.0).GetEntry();
nt::GenericEntry* Iz_FrontLeft_entry = Front_Left.Add("Iz", 0.0).GetEntry();
nt::GenericEntry* FF_FrontLeft_entry = Front_Left.Add("FF", 0.0).GetEntry();
 
frc::ShuffleboardTab& Front_Left_steer = frc::Shuffleboard::GetTab("Front Left Steer");
nt::GenericEntry* P_FrontLeftSteer_entry = Front_Left_steer.Add("P", 0.0).GetEntry(); 
nt::GenericEntry* I_FrontLeftSteer_entry = Front_Left_steer.Add("I", 0.0).GetEntry(); 
nt::GenericEntry* D_FrontLeftSteer_entry = Front_Left_steer.Add("D", 0.0).GetEntry(); 
nt::GenericEntry* P_LL_FrontLeftSteer_entry = Front_Left_steer.Add("P LL", 0.0).GetEntry(); 
nt::GenericEntry* P_UL_FrontLeftSteer_entry = Front_Left_steer.Add("P UL", 0.0).GetEntry(); 
nt::GenericEntry* I_LL_FrontLeftSteer_entry = Front_Left_steer.Add("I LL", 0.0).GetEntry(); 
nt::GenericEntry* I_UL_FrontLeftSteer_entry = Front_Left_steer.Add("I UL", 0.0).GetEntry(); 
nt::GenericEntry* D_LL_FrontLeftSteer_entry = Front_Left_steer.Add("D LL", 0.0).GetEntry(); 
nt::GenericEntry* D_UL_FrontLeftSteer_entry = Front_Left_steer.Add("D UL", 0.0).GetEntry(); 
nt::GenericEntry* LL_FrontLeftSteer_entry = Front_Left_steer.Add("LL", 0.0).GetEntry(); 
nt::GenericEntry* UL_FrontLeftSteer_entry = Front_Left_steer.Add("UL", 0.0).GetEntry(); 
nt::GenericEntry* Iz_FrontLeftSteer_entry = Front_Left_steer.Add("Iz", 0.0).GetEntry(); 
nt::GenericEntry* FF_FrontLeftSteer_entry = Front_Left_steer.Add("FF", 0.0).GetEntry(); 
 
 
shuffleboard_FrontleftPID.P = P_FrontLeft_entry->GetDouble(0.0);
shuffleboard_FrontleftPID.I = I_FrontLeft_entry->GetDouble(0.0);
shuffleboard_FrontleftPID.D = D_FrontLeft_entry->GetDouble(0.0);
shuffleboard_FrontleftPID.P_UL = P_UL_FrontLeft_entry->GetDouble(0.0);
shuffleboard_FrontleftPID.P_LL = P_LL_FrontLeft_entry->GetDouble(0.0);
shuffleboard_FrontleftPID.I_UL = I_UL_FrontLeft_entry->GetDouble(0.0);
shuffleboard_FrontleftPID.I_LL = I_LL_FrontLeft_entry->GetDouble(0.0);
shuffleboard_FrontleftPID.D_UL = D_UL_FrontLeft_entry->GetDouble(0.0);
shuffleboard_FrontleftPID.D_LL = D_LL_FrontLeft_entry->GetDouble(0.0);
shuffleboard_FrontleftPID.UL = UL_FrontLeft_entry->GetDouble(0.0);
shuffleboard_FrontleftPID.LL = LL_FrontLeft_entry->GetDouble(0.0);
shuffleboard_FrontleftPID.Iz = Iz_FrontLeft_entry->GetDouble(0.0);
shuffleboard_FrontleftPID.FF = FF_FrontLeft_entry->GetDouble(0.0);

shuffleboard_SteerFrontleftPID.P = P_FrontLeftSteer_entry->GetDouble(0.0);
shuffleboard_SteerFrontleftPID.I = I_FrontLeftSteer_entry->GetDouble(0.0);
shuffleboard_SteerFrontleftPID.D = D_FrontLeftSteer_entry->GetDouble(0.0);
shuffleboard_SteerFrontleftPID.P_UL = P_UL_FrontLeftSteer_entry->GetDouble(0.0);
shuffleboard_SteerFrontleftPID.P_LL = P_LL_FrontLeftSteer_entry->GetDouble(0.0);
shuffleboard_SteerFrontleftPID.I_UL = I_UL_FrontLeftSteer_entry->GetDouble(0.0);
shuffleboard_SteerFrontleftPID.I_LL = I_LL_FrontLeftSteer_entry->GetDouble(0.0);
shuffleboard_SteerFrontleftPID.D_UL = D_UL_FrontLeftSteer_entry->GetDouble(0.0);
shuffleboard_SteerFrontleftPID.D_LL = D_LL_FrontLeftSteer_entry->GetDouble(0.0);
shuffleboard_SteerFrontleftPID.UL = UL_FrontLeftSteer_entry->GetDouble(0.0);
shuffleboard_SteerFrontleftPID.LL = LL_FrontLeftSteer_entry->GetDouble(0.0);
shuffleboard_SteerFrontleftPID.Iz = Iz_FrontLeftSteer_entry->GetDouble(0.0);
shuffleboard_SteerFrontleftPID.FF = FF_FrontLeftSteer_entry->GetDouble(0.0);
}