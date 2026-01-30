#pragma once

#include "../../lib/Eigen/Dense"

using Scalar = double;
using Vec3   = Eigen::Matrix<Scalar, 3, 1>;
using Quat   = Eigen::Quaternion<Scalar>;
using Mat3   = Eigen::Matrix<Scalar, 3, 3>;

#define M_PI 3.14159265358979323846