#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonUtils.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/geometry/Pose3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <vector>

#include "Const.hpp"
#include "Odometry.hpp"
#include "Vision.hpp"
#define VisionOn
#ifdef VisionOn
frc::AprilTagFieldLayout L_Vis_Layout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

// cameras are defined as shared pointers like so:
photon::PhotonCamera L_FrontCam = photon::PhotonCamera("FrontCam");

// photon::PhotonCamera L_LeftCam = photon::PhotonCamera("LeftCam");

photon::PhotonCamera L_RightCam = photon::PhotonCamera("RightCam");

// define a Transform3d value for the camera position on the bot
frc::Transform3d L_robotToFrontCam =
    frc::Transform3d(frc::Translation3d(0_in, 0_in, 8.5_in),
                     frc::Rotation3d(0_deg, 0_deg, 0_deg));
// TODO - setup Right and Left cam locations, once mounts are better defined
frc::Transform3d L_robotToLeftCam =
    frc::Transform3d(frc::Translation3d(0_in, 0_in, 9_in),
                     frc::Rotation3d(0_deg, 0_deg, 0_deg));

frc::Transform3d L_robotToRightCam =
    frc::Transform3d(frc::Translation3d(0_in, 0_in, 9_in),
                     frc::Rotation3d(0_deg, 0_deg, 0_deg));

std::optional<photon::EstimatedRobotPose> L_Vis_CurrentCamPose;

photon::PhotonPoseEstimator FrontPoseEstimator = photon::PhotonPoseEstimator{
    L_Vis_Layout,
    photon::LOWEST_AMBIGUITY,
    std::move(L_FrontCam),
    L_robotToFrontCam};

photon::PhotonPoseEstimator RightPoseEstimator = photon::PhotonPoseEstimator{
    L_Vis_Layout,
    photon::LOWEST_AMBIGUITY,
    std::move(L_RightCam),
    L_robotToRightCam};

// cameras is a vector of pairs of (camera obj, camera position)
std::vector<photon::PhotonPoseEstimator> Estimators{
    FrontPoseEstimator,
    RightPoseEstimator};

const frc::Pose3d BlankPose = frc::Pose3d(0.0_m, 0.0_m, 0.0_m, frc::Rotation3d(0.0_rad, 0.0_rad, 0.0_rad));

// stores (found pose, )
std::vector<std::pair<frc::Pose3d, double>> L_VisCamResults = {};

double Ve_Vis_VisionCenteredCounter = 0.0;
bool Le_Vis_VisionCentered = false;
double VeVis_deg_VisionYaw = 0.0;
bool *VeVis_CenteringEnable = false;


/******************************************************************************
 * Function:     VisionInit
 *
 * Description: initializes some values for vision's operation.
 ******************************************************************************/
void VisionInit()
{
    // setup our results to have the same number of spots as we have estimators
    L_VisCamResults.assign(Estimators.size(), std::make_pair(BlankPose, 255.0));
}

// a debug value to see how many checks have passed

void VisionRun()
{
    int Debug_tests_passed = 0;
    double L_outputX = 0.0;
    double L_outputY = 0.0;

    // time how long it's been since we had a good centering;
    if (Le_Vis_VisionCentered)
    {
        Ve_Vis_VisionCenteredCounter += C_ExeTime;
    }

    if (Ve_Vis_VisionCenteredCounter >= KeVIS_t_VisionTimeout)
    {
        Ve_Vis_VisionCenteredCounter = 0.0;
        Le_Vis_VisionCentered = false;
        Debug_tests_passed = 0;
    }

    frc::SmartDashboard::PutNumber("vision centered counter", Ve_Vis_VisionCenteredCounter);

    // frc::SmartDashboard::PutNumber("slot #", L_VisCamResults.size());
    // frc::SmartDashboard::PutNumber("cam #", Estimators.size());
    // frc::SmartDashboard::PutBoolean("emtpy lists", Estimators.empty());

    // loop through our list of cameras
    for (unsigned L_CamIndex = 0; L_CamIndex < Estimators.size(); ++L_CamIndex)
    {
        L_Vis_CurrentCamPose = Estimators[L_CamIndex].Update();
        frc::SmartDashboard::PutNumber("index", L_CamIndex);

        // check that cam has a target
        if (L_Vis_CurrentCamPose.has_value())
        {
            L_VisCamResults[L_CamIndex].first = L_Vis_CurrentCamPose.value().estimatedPose;
            L_VisCamResults[L_CamIndex].second = Estimators[L_CamIndex].GetCamera().get()->GetLatestResult().GetBestTarget().GetPoseAmbiguity();
        }
        else
        {
            // if we have no target still fill the results with a blank pose and a maximum ambiguity
            L_VisCamResults[L_CamIndex].first = BlankPose;
            L_VisCamResults[L_CamIndex].second = 255.0;
        }
    }
    frc::SmartDashboard::PutNumber("cam 0 ambiguity", L_VisCamResults[0].second);
    frc::SmartDashboard::PutNumber("cam 1 ambiguity", L_VisCamResults[1].second);

    frc::SmartDashboard::PutNumber("cam 0 x", L_VisCamResults[0].first.Translation().X().value());
    frc::SmartDashboard::PutNumber("cam 1 x", L_VisCamResults[1].first.Translation().X().value());

    int L_bestCam = 0;

    // loop through our newly found results
    for (unsigned camResultIndex = 0; camResultIndex < L_VisCamResults.size(); camResultIndex++)
    {
        if (L_VisCamResults[camResultIndex].second <= L_VisCamResults[L_bestCam].second)
        {
            L_bestCam = camResultIndex;
        }
    }

    frc::SmartDashboard::PutNumber("Best cam", L_bestCam);

    VeVis_deg_VisionYaw = L_VisCamResults[L_bestCam].first.Rotation().Z().value() * 180;

    L_outputX = L_VisCamResults[L_bestCam].first.Translation().X().value(); // this returns in meters
    L_outputX *= C_MeterToIn;                                               // convert to inches
    L_outputY = L_VisCamResults[L_bestCam].first.Translation().Y().value(); // this returns in meters
    L_outputY *= C_MeterToIn;

    frc::SmartDashboard::PutNumber("vision out x", L_outputX);

    frc::SmartDashboard::PutNumber("vision out y", L_outputY);
    frc::SmartDashboard::PutNumber("vision out yaw", VeVis_deg_VisionYaw);
    // we need to wait at least 2 seconds before centering, we're probably fine in those 2 seconds
    // or maybe we haven't centered at all
    if ((Ve_Vis_VisionCenteredCounter >= 2.0) || (Le_Vis_VisionCentered == false))
    {
        Debug_tests_passed++;
        if (L_VisCamResults[L_bestCam].second <= KeVIS_AmbiguityThreshold)
        {
            Debug_tests_passed++;

            if ((L_outputX > 0.0) && (L_outputY > 0.0) && (L_outputX < KeVIS_in_MaxX) // sanity check theat we are in the bounds of the field
                && (L_outputY < KeVIS_in_MaxY))
            {
                Debug_tests_passed++;

                // NOTE - at maximum speed we get a delta X of about 2.0, chose half that
                //  we probably have to be a little below max speed to trust cam updating
                if ((fabs(VeODO_In_DeltaX) < KeVis_dIn_DeltaThreshold) && (fabs(VeODO_In_DeltaY) < KeVis_dIn_DeltaThreshold))
                {
                    Debug_tests_passed++;

                    // even if everything passes we can manually say no
                    if (VeVis_CenteringEnable)
                    {
                        Debug_tests_passed++;

                        OdometryInitToArgs(L_outputX, L_outputY);

                        Le_Vis_VisionCentered = true;
                    }
                }
            }
        }
    }

    // frc::SmartDashboard::PutNumber("Debug: Vision tests passed", Debug_tests_passed);
    frc::SmartDashboard::PutBoolean("vision centred", Le_Vis_VisionCentered);
}
#endif