// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Santiago Quintana Moreno 

#pragma once

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPipelineResult.h>
#include <frc/Notifier.h>
#include <frc/geometry/Pose2d.h>
#include <frc/DigitalOutput.h>
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

  units::meter_t getDistanceToTarget();

  bool isChassisAligned(units::degree_t tolerance = 2_deg);

  void setLeds(bool set);
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
  
        
  const std::string cameraName = "overcamara";
  photonlib::PhotonPipelineResult lastResult;
  photonlib::PhotonCamera camera {cameraName};
  frc::Pose2d visionPose;
  frc::Notifier visionNotifier;
  const int minTargetCount = 2;
  const units::meter_t targetHeight = 2.631923_m; //From Field's CAD
  const frc::Pose2d fieldToTarget {8.25_m, 4.07_m, {0_deg}};
  //const frc::Pose2d fieldToTarget {0_m, 0_m, {0_deg}};

  const frc::Transform2d shooterToRobot {{0_m, 0_m}, 180_deg};
  frc::DigitalOutput ledRelay {2};
  Chassis* chassis;

};

namespace CameraConstants {
  const frc::Rotation2d pitch {35_deg};
  const units::meter_t height = 30.11_in;
  const double vpw = 2.0 * units::math::tan(58.5_deg / 2.0);
  const double vph = 2.0 *  units::math::tan(45.6_deg / 2.0);
  const std::pair<double, double> resolution {640, 480};
  const frc::Transform2d cameraToRobot {{-12.5_in, 16.1_in}, 180_deg};
}
