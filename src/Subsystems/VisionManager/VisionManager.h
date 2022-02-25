// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPipelineResult.h>
#include <frc/Notifier.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/SubsystemBase.h>
#include <Eigen/SVD>
#include "Subsystems/Chassis/Chassis.h"
class VisionManager : public frc2::SubsystemBase {
 public:
 struct Circle{
    frc::Translation2d midpoint;
    units::meter_t radius;
  };

  VisionManager(Chassis* chassis);

  const frc::Pose2d& getTargetPose();

  frc::Rotation2d getRotationToTarget();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  bool updateCircleFit(const photonlib::PhotonPipelineResult& result);
  std::vector<std::pair<double, double>> sortPoints(const std::vector<std::pair<double, double>>& points);
  std::optional<frc::Translation2d> cameraToTargetTranslation(std::pair<double, double> corner, const units::meter_t targetHeight);
  std::optional<Circle> solveLeastSquaresCircle(const std::vector<frc::Translation2d>& circlePoints);
  bool solveLeastSquaresCircle(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> &points, Eigen::Vector2d &midpoint, double &radius);
  
        
  const std::string cameraName = "TheOverCamara";
  photonlib::PhotonPipelineResult lastResult;
  photonlib::PhotonCamera camera {cameraName};
  frc::Pose2d visionPose;
  frc::Notifier visionNotifier;
  const int minTargetCount = 2;
  const units::meter_t targetHeight = 2.631923_m; //From Field's CAD
  const frc::Pose2d fieldToTarget {8.033_m, 4.108_m, {0_deg}};
  Chassis* chassis;

};

namespace CameraConstants {
  const frc::Rotation2d pitch {34_deg};
  const units::meter_t height = 0.5_m;
  const double vpw = 2.0 * std::tan((59.6 / 2.0) * M_PI / 180.0);
  const double vph = 2.0 * std::tan((49.7 / 2.0) * M_PI / 180.0);
  const std::pair<double, double> resolution {640, 480};
  const frc::Transform2d cameraToRobot;
}
