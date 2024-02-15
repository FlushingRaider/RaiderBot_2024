#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonUtils.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/geometry/Pose3d.h>

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
    photon::PhotonPoseEstimator FrontPoseEstimator{
        L_Vis_Layout,
        photon::LOWEST_AMBIGUITY,
        cameras[0].second};
    photon::PhotonPoseEstimator LeftPoseEstimator{
        L_Vis_Layout,
        photon::LOWEST_AMBIGUITY,
        cameras[1].second};
    photon::PhotonPoseEstimator RightPoseEstimator{
        L_Vis_Layout,
        photon::LOWEST_AMBIGUITY,
        cameras[2].second};
}
