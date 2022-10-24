// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionManager.h"

#include <iostream>
#include <numeric>

VisionManager::VisionManager(Chassis* chassis)
    : visionNotifier([this] {
        photonlib::PhotonPipelineResult result = camera.GetLatestResult();
        if (result != lastResult) {
          bool res = updateCircleFit(result);
          frc::SmartDashboard::PutBoolean("VisionManager/CircleFitted", res);
          if (res) {
            units::second_t targetTimestamp =
                frc::Timer::GetFPGATimestamp() - result.GetLatency();
            this->chassis->addVisionMeasurement(visionPose,
                                                targetTimestamp.value());
            lastResult = result;
          }
        }
      }) {
  this->chassis = chassis;

  setLeds(true);
  visionNotifier.StartPeriodic(20_ms);
}

const frc::Pose2d& VisionManager::getTargetPose() { return fieldToTarget; }

frc::Rotation2d VisionManager::getRotationToTarget() {
  const auto poseDiff =
      fieldToTarget.Translation() - chassis->getPose().Translation();

  return frc::Rotation2d(poseDiff.X().value(), poseDiff.Y().value()) + shooterToRobot.Rotation();
}

units::meter_t VisionManager::getDistanceToTarget() {
  return chassis->getPose().Translation().Distance(fieldToTarget.Translation());
}

bool VisionManager::isChassisAligned(units::degree_t tolerance) {
  return units::math::abs(chassis->getPose().Rotation().Degrees() -
                          getRotationToTarget().Degrees()) < tolerance;
}

void VisionManager::setLeds(bool set) {
  camera.SetLEDMode(set ? photonlib::kOn : photonlib::kOff);
}

// This method will be called once per scheduler run
void VisionManager::Periodic() {
  frc::SmartDashboard::PutNumber("VisionManager/Distance",
                                 getDistanceToTarget().value());
  frc::SmartDashboard::PutNumber("VisionManager/Heading",
                                 getRotationToTarget().Degrees().value());
  frc::SmartDashboard::PutBoolean("VisionManager/ChassisAligned",
                                  isChassisAligned());

  ledRelay.Set(!(camera.GetLEDMode() == photonlib::kOn));
}

bool VisionManager::updateCircleFit(
    const photonlib::PhotonPipelineResult& result) {
  frc::SmartDashboard::PutNumber("VisionManager/TargetCount",
                                 result.GetTargets().size());

  if (!result.HasTargets() ||
      (int)result.GetTargets().size() < minTargetCount ||
      camera.GetLEDMode() != photonlib::LEDMode::kOn) {
    return false;
  }

  const auto targets = result.GetTargets();
  std::vector<frc::Translation2d> outputTranslations;

  for (auto targetPointer = targets.begin(); targetPointer != targets.end();
       targetPointer++) {
    auto& target = *targetPointer;
    std::vector<std::pair<double, double>> targetPoints, topTargets,
        bottomTargets;

    for (const auto& point : target.GetCorners()) {
      targetPoints.emplace_back(point);
    }

    if (targetPoints.size() != 4) {
      std::cout << "TargePoints is different to 4!!! "
                << (int)targetPoints.size() << "\n";
      continue;
    }

    targetPoints = sortPoints(targetPoints);

    std::string sortedPoints;

    for (const auto point : targetPoints) {
      sortedPoints +=
          "[" + fmt::format("{} , {}", point.first, point.second) + "]";
    }
    frc::SmartDashboard::PutString("VisionManager/SortedPoints", sortedPoints);

    std::copy_n(targetPoints.begin(), 2,
                std::back_inserter(topTargets));  // First 2 targets
    std::copy_n(targetPoints.rbegin(), 2,
                std::back_inserter(bottomTargets));  // Last two points

    for (auto corner = topTargets.begin(); corner != topTargets.end();
         corner++) {
      auto ret = cameraToTargetTranslation(*corner, targetHeight);
      if (ret.has_value()) {
        outputTranslations.push_back(std::move(ret.value()));
      }
    }

    for (auto corner = bottomTargets.begin(); corner != bottomTargets.end();
         corner++) {
      auto ret = cameraToTargetTranslation(*corner, targetHeight - 2_in);
      if (ret.has_value()) {
        outputTranslations.push_back(std::move(ret.value()));
      }
    }
  }

  auto circleRet = solveLeastSquaresCircle(outputTranslations);
  if (!circleRet.has_value()) {
    std::cout << "Could not fit circle\n";
    return false;
  }

  Circle circle = circleRet.value();

  frc::Translation2d circleTranslation = circle.midpoint;
  frc::SmartDashboard::PutString(
      "VisionManager/TraslationToCircle",
      fmt::format("X:{} Y:{}", circleTranslation.X(), circleTranslation.Y()));

  frc::Rotation2d robotRotation = chassis->getPose().Rotation();
  frc::Rotation2d cameraRotation =
      robotRotation.RotateBy(CameraConstants::cameraToRobot.Rotation());
  frc::Transform2d fieldToTargetRotated{fieldToTarget.Translation(),
                                        cameraRotation};

  frc::Transform2d fieldToCamera =
      fieldToTargetRotated + frc::Transform2d(-circleTranslation, {});
  frc::Transform2d fieldToRobot =
      fieldToCamera + CameraConstants::cameraToRobot.Inverse();
  visionPose = {fieldToRobot.Translation(), fieldToRobot.Rotation()};
  return true;
}

std::vector<std::pair<double, double>> VisionManager::sortPoints(
    const std::vector<std::pair<double, double>>& points) {
  // Get sum of all points
  auto targetCenter = std::accumulate(
      points.begin(), points.end(), std::pair(0.0, 0.0),
      [](const auto& p1, const auto& p2) {
        return std::pair(p1.first + p2.first, p1.second + p2.second);
      });
  // Average center
  targetCenter.first /= points.size();
  targetCenter.second /= points.size();

  // Find top corners
  int topLeftIndex = -1;
  int topRightIndex = -1;

  double minPosRads = M_PI;
  double minNegRads = M_PI;
  for (int i = 0; i < (int)points.size(); i++) {
    std::pair<double, double> corner = points[i];

    double angleRad = (frc::Rotation2d(corner.first - targetCenter.first,
                                       targetCenter.second - corner.second) -
                       frc::Rotation2d(90_deg))
                          .Radians()
                          .value();
    if (angleRad > 0) {
      if (angleRad < minPosRads) {
        minPosRads = angleRad;
        topLeftIndex = i;
      }
    } else {
      if (std::abs(angleRad) < minNegRads) {
        minNegRads = std::abs(angleRad);
        topRightIndex = i;
      }
    }
  }

  // Find lower corners
  int lowerIndex1 = -1;
  int lowerIndex2 = -1;
  for (int i = 0; i < (int)points.size(); i++) {
    bool alreadySaved = false;
    if (topLeftIndex != -1) {
      if (topLeftIndex == i) {
        alreadySaved = true;
      }
    }
    if (topRightIndex != -1) {
      if (topRightIndex == i) {
        alreadySaved = true;
      }
    }
    if (!alreadySaved) {
      if (lowerIndex1 == -1) {
        lowerIndex1 = i;
      } else {
        lowerIndex2 = i;
      }
    }
  }

  // Combine final list
  std::vector<std::pair<double, double>> newCorners;
  if (topLeftIndex != -1) {
    newCorners.emplace_back(points[topLeftIndex]);
  }

  if (topRightIndex != -1) {
    newCorners.emplace_back(points[topRightIndex]);
  }

  if (lowerIndex1 != -1) {
    newCorners.emplace_back(points[lowerIndex1]);
  }

  if (lowerIndex2 != -1) {
    newCorners.emplace_back(points[lowerIndex2]);
  }

  return newCorners;
}

std::optional<frc::Translation2d> VisionManager::cameraToTargetTranslation(
    std::pair<double, double> corner, const units::meter_t targetHeight) {
  frc::SmartDashboard::PutString(
      "cameraToTargetTranslation_params",
      fmt::format("{} {} {} {} {}", CameraConstants::vpw, CameraConstants::vph,
                  CameraConstants::height,
                  CameraConstants::pitch.Degrees().value(), targetHeight));

  double yPixels = corner.first;
  double zPixels = corner.second;

  // Robot frame of reference
  double nY = -((yPixels - (CameraConstants::resolution.first / 2)) /
                (CameraConstants::resolution.first / 2));
  double nZ = -((zPixels - (CameraConstants::resolution.second / 2)) /
                (CameraConstants::resolution.second / 2));

  frc::Translation2d xzPlaneTranslation =
      frc::Translation2d(1.0_m, units::meter_t(CameraConstants::vph / 2.0 * nZ))
          .RotateBy(CameraConstants::pitch);
  double x = xzPlaneTranslation.X().value();
  double y = CameraConstants::vpw / 2.0 * nY;
  double z = xzPlaneTranslation.Y().value();

  double differentialHeight =
      (CameraConstants::height - this->targetHeight).value();
  if ((z < 0.0) == (differentialHeight > 0.0)) {
    double scaling = differentialHeight / -z;
    double distance = std::hypot(x, y) * scaling;
    frc::Rotation2d angle = frc::Rotation2d(x, y);
    return frc::Translation2d(units::meter_t(distance * angle.Cos()),
                              units::meter_t(distance * angle.Sin()));
  }
  return {};
}

std::optional<VisionManager::Circle> VisionManager::solveLeastSquaresCircle(
    const std::vector<frc::Translation2d>& circlePoints) {
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>
      eigenPoints;
  Eigen::Vector2d midpoint;
  Circle circle;

  std::transform(circlePoints.begin(), circlePoints.end(),
                 std::back_inserter(eigenPoints),
                 [](const frc::Translation2d& point) {
                   return Eigen::Vector2d(point.X().value(), point.Y().value());
                 });

  size_t length = eigenPoints.size();
  double x1;
  double x2;
  double x3;
  Eigen::MatrixXd AFill(3, length);
  Eigen::MatrixXd A(length, 3);
  Eigen::VectorXd AFirst(length);
  Eigen::VectorXd ASec(length);
  Eigen::VectorXd AFirstSquared(length);
  Eigen::VectorXd ASecSquared(length);
  Eigen::VectorXd ASquaredRes(length);
  Eigen::VectorXd b(length);
  Eigen::VectorXd c(3);
  bool ok = true;

  if (length > 1) {
    for (size_t i = 0; i < length; i++) {
      AFill(0, i) = eigenPoints[i](0);
      AFill(1, i) = eigenPoints[i](1);
      AFill(2, i) = 1;
    }

    A = AFill.transpose();

    for (size_t i = 0; i < length; i++) {
      AFirst(i) = A(i, 0);
      ASec(i) = A(i, 1);
    }

    for (size_t i = 0; i < length; i++) {
      AFirstSquared(i) = AFirst(i) * AFirst(i);
      ASecSquared(i) = ASec(i) * ASec(i);
    }

    b = AFirstSquared + ASecSquared;

    c = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    x1 = c(0);
    midpoint(0) = x1 * 0.5;
    x2 = c(1);
    midpoint(1) = x2 * 0.5;
    x3 = c(2);
    circle.radius = units::meter_t(sqrt((x1 * x1 + x2 * x2) / 4 + x3));
  } else {
    ok = false;
  }

  if (!ok) return std::nullopt;
  circle.midpoint = frc::Translation2d(units::meter_t(midpoint.x()),
                                       units::meter_t(midpoint.y()));

  return circle;
}
