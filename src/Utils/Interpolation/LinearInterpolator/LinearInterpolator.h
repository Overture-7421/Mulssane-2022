// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <array>
#include <vector>
typedef std::vector<std::pair<double, double>> DataTable;

class LinearInterpolator {
 public:
  LinearInterpolator(const DataTable& dataTable);
  double getY(double x);

 private:
  std::array<DataTable::iterator, 2> getInterpolationPoints(double value);
  DataTable dataTable;
};
