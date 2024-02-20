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

frc::AprilTagFieldLayout L_Vis_Layout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

// cameras is a vector of pairs of (camera obj, camera position)
std::vector<std::pair<std::shared_ptr<photon::PhotonCamera>, frc::Transform3d>> cameras;

// cameras are defined as shared pointers like so:
std::shared_ptr<photon::PhotonCamera> L_FrontCam = std::make_shared<photon::PhotonCamera>("FrontCam");

std::shared_ptr<photon::PhotonCamera> L_LeftCam = std::make_shared<photon::PhotonCamera>("LeftCam");

std::shared_ptr<photon::PhotonCamera> L_RightCam = std::make_shared<photon::PhotonCamera>("RightCam");

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

photon::PhotonPipelineResult L_Vis_CurrentCamResult;

std::optional<photon::EstimatedRobotPose> L_Vis_CurrentCamPose;

photon::PhotonPoseEstimator FrontPoseEstimator{
    L_Vis_Layout,
    photon::LOWEST_AMBIGUITY,
    photon::PhotonCamera{"Front"},
    cameras[0].second};
photon::PhotonPoseEstimator LeftPoseEstimator{
    L_Vis_Layout,
    photon::LOWEST_AMBIGUITY,
    photon::PhotonCamera{"Left"},
    cameras[1].second};
photon::PhotonPoseEstimator RightPoseEstimator{
    L_Vis_Layout,
    photon::LOWEST_AMBIGUITY,
    photon::PhotonCamera{"Right"},
    cameras[2].second};

std::vector<std::pair<frc::Pose3d, double>> L_VisCamResults;

double Ve_Vis_VisionCenteredCounter = 0.0;
bool Le_Vis_VisionCentered = false;

void VisionInit()
{
    // order is front, left, right (0, 1, 2)
    cameras.push_back(std::make_pair(L_FrontCam, L_robotToFrontCam));
    cameras.push_back(std::make_pair(L_LeftCam, L_robotToLeftCam));
    cameras.push_back(std::make_pair(L_RightCam, L_robotToRightCam));

    /*TODO - ask photon devs the difference between estimator that requires
    * frc::AprilTagFieldLayout aprilTags,
                               PoseStrategy strategy, PhotonCamera&& camera,
                               frc::Transform3d robotToCamera)

    * and estimator that requires:
    *
    * frc::AprilTagFieldLayout aprilTags,
                               PoseStrategy strategy,
                               frc::Transform3d robotToCamera
    */
}

// a debug value to see how many checks have passed
int Debug_tests_passed = 0;

void VisionRun(bool L_DisableCentering)
{

    int L_bestCam = 0;

    double L_outputX = 0.0;
    double L_outputY = 0.0;

    // time how long it's been since we had a good centering;
    if (Le_Vis_VisionCentered)
    {
        Ve_Vis_VisionCenteredCounter += 0.02;
    }

    if (Ve_Vis_VisionCenteredCounter >= 6.0)
    {
        Le_Vis_VisionCentered = false;
        Debug_tests_passed = 0;
    }

    // loop through our list of cameras
    for (unsigned L_CamIndex = 0; L_CamIndex < cameras.size(); L_CamIndex++)
    {
        // grab the latest result from this cam
        L_Vis_CurrentCamResult = cameras[L_CamIndex].first.get()->GetLatestResult();

        // check that cam has a target
        if (L_Vis_CurrentCamResult.HasTargets())
        {
            // select the proper pose estimator
            switch (L_CamIndex)
            {
            case 0:
                L_Vis_CurrentCamPose = FrontPoseEstimator.Update();
                break;
            case 1:
                L_Vis_CurrentCamPose = LeftPoseEstimator.Update();
                break;
            case 2:
                L_Vis_CurrentCamPose = RightPoseEstimator.Update();
                break;
            }

            // if we got a value
            if (L_Vis_CurrentCamPose.has_value())
            {
                // store it in our result vector
                L_VisCamResults[L_CamIndex].first = L_Vis_CurrentCamPose.value().estimatedPose;
                // store the ambiguity too
                L_VisCamResults[L_CamIndex].second = L_Vis_CurrentCamResult.GetBestTarget().poseAmbiguity;
            };
        }
    }

    // loop through our newly found results
    for (unsigned camResult = 0; camResult < L_VisCamResults.size(); camResult++)
    {
        // if cam 1 is lower ambiguity than cam 0, make that the best
        // if cam 2 is lower than cam 1 make that the best, etc
        if (L_VisCamResults[camResult + 1].second <= L_VisCamResults[camResult].second)
        {
            L_bestCam = camResult + 1;
        }
    }

    // we need to wait at least 2 seconds before centering, we're probably fine in those 2 seconds
    // or maybe we haven't centered at all
    if ((Ve_Vis_VisionCenteredCounter >= 2.0) || (Le_Vis_VisionCentered == false))
    {
        Debug_tests_passed++;
        if (L_VisCamResults[L_bestCam].second >= KeAmbiguityThreshold)
        {
            Debug_tests_passed++;

            if ((L_outputX > 0.0) && (L_outputY > 0.0) && (L_outputX < KeMaxX) // sanity check theat we are in the bounds of the field
                && (L_outputY < KeMaxY))
            {
                Debug_tests_passed++;

                // NOTE - at maximum speed we get a delta X of about 2.0, chose half that
                //  we probably have to be a little below max speed to trust cam updating
                if ((fabs(VeODO_In_DeltaX) < 1.0) && (fabs(VeODO_In_DeltaY) < 1.0))
                {
                    Debug_tests_passed++;

                    // even if everything passes we can manually say no
                    if (L_DisableCentering)
                    {
                        Debug_tests_passed++;

                        L_outputX = L_VisCamResults[L_bestCam].first.Translation().X().value(); // this returns in meters
                        L_outputX *= C_MeterToIn;
                        L_outputY = L_VisCamResults[L_bestCam].first.Translation().Y().value(); // this returns in meters
                        L_outputY *= C_MeterToIn;

                        OdometryInitToArgs(L_outputX, L_outputY);
                        Le_Vis_VisionCentered = true;
                    }
                }
            }
        }
    }

    frc::SmartDashboard::PutNumber("Debug: Vision tests passed", Debug_tests_passed);
}