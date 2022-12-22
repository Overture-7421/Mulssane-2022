// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionManager.h"

VisionManager::VisionManager(SwerveChassis *chassis):chassis(chassis){

};

// This method will be called once per scheduler run
void VisionManager::Periodic() {
    photonlib::PhotonPipelineResult cameraResult = camera.GetLatestResult();

    if (cameraResult.HasTargets()) {
        photonlib::PhotonTrackedTarget target = cameraResult.GetBestTarget();
        auto poseVector = visionPoseEntry.GetDoubleArray({ 0,0,0,0,0,0 });

        //frc::Transform2d targetPose = target.GetCameraRelativePose();
        frc::Transform2d targetPose = { {units::meter_t(poseVector[0]), units::meter_t(poseVector[1])},QuaternionToEuler(poseVector) };

        frc::Transform2d fieldToCamera =
            fieldToTarget + targetPose.Inverse();
        frc::Transform2d fieldToRobot =
            fieldToCamera + cameraToRobot.Inverse();
        frc::Pose2d visionPose = { fieldToRobot.Translation(), fieldToRobot.Rotation() };
        chassis->addVisionMeasurement(visionPose, cameraResult.GetLatency());

        frc::SmartDashboard::PutNumber("Vision-X", visionPose.X().value());
        frc::SmartDashboard::PutNumber("Vision-Y", visionPose.Y().value());
        frc::SmartDashboard::PutNumber("Vision-Degrees", visionPose.Rotation().Degrees().value());
    }

}

frc::Rotation2d VisionManager::QuaternionToEuler(std::vector<double> quaternion) {

    const auto x = quaternion[4];
    const auto y = quaternion[5];
    const auto z = quaternion[6];
    const auto w = quaternion[3];

    // // roll (x-axis rotation)
    // double sinr_cosp = 2 * (w * x + y * z);
    // double cosr_cosp = 1 - 2 * (x * x + y * y);
    // angles[2] = std::atan2(sinr_cosp, cosr_cosp);

    // // pitch (y-axis rotation)
    // double sinp = 2 * (w * y - z * x);
    // if (std::abs(sinp) >= 1)
    //     angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    // else
    //     angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return { units::radian_t(yaw) };

}
