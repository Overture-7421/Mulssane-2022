// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "LinearInterpolator.h"

#include <fmt/format.h>

#include <algorithm>
#include <iostream>

LinearInterpolator::LinearInterpolator(const DataTable& dataTable) {
  this->dataTable = dataTable;
  if (this->dataTable.size() < 2) {
    throw std::invalid_argument(
        "LinearInterpolator given less than two points");
  }
}

double LinearInterpolator::getY(double x) {
  const auto& points = getInterpolationPoints(x);
  const auto& startPoint = points[0];
  const auto& endPoint = points[1];

  double change = ((endPoint->second - startPoint->second) /
                   (endPoint->first - startPoint->first)) *
                  (x - startPoint->first);
  return startPoint->second + change;
}

std::array<DataTable::iterator, 2> LinearInterpolator::getInterpolationPoints(
    double value) {
  auto it = std::upper_bound(
      dataTable.begin(), dataTable.end(), value,
      [](double a, const std::pair<double, double>& b) { return a < b.first; });

  if (dataTable.begin() == it) {
    return {it, it + 1};
  }

  if (dataTable.end() == it) {
    return {it - 2, it - 1};
  };

  return {it - 1, it};
}
