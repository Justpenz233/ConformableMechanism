/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2015, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jia Pan */

#ifndef HPP_FCL_TRANSFORM_H
#define HPP_FCL_TRANSFORM_H

#include "hpp/fcl/fwd.hh"
#include "hpp/fcl/data_types.h"

namespace hpp {
namespace fcl {

typedef Eigen::Quaternion<FCL_REAL> Quaternion3f;
typedef Eigen::Quaternion<FCL_REAL> Quatf;

static inline std::ostream& operator<<(std::ostream& o, const Quatf& q) {
  o << "(" << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << ")";
  return o;
}

/// @brief Simple transform class used locally by InterpMotion
class HPP_FCL_DLLAPI Transform3f {
  /// @brief Matrix cache
  Matrix3f R;

  /// @brief Translation vector
  Vec3f T;

 public:
  /// @brief Default transform is no movement
  Transform3f() {
    setIdentity();  // set matrix_set true
  }

  static Transform3f Identity() { return Transform3f(); }

  /// @brief Construct transform from rotation and translation
  template <typename Matrixx3Like, typename Vector3Like>
  Transform3f(const Eigen::MatrixBase<Matrixx3Like>& R_,
              const Eigen::MatrixBase<Vector3Like>& T_)
      : R(R_), T(T_) {}

  /// @brief Construct transform from rotation and translation
  template <typename Vector3Like>
  Transform3f(const Quatf& q_, const Eigen::MatrixBase<Vector3Like>& T_)
      : R(q_.toRotationMatrix()), T(T_) {}

  /// @brief Construct transform from rotation
  Transform3f(const Matrix3f& R_) : R(R_), T(Vec3f::Zero()) {}

  /// @brief Construct transform from rotation
  Transform3f(const Quatf& q_) : R(q_), T(Vec3f::Zero()) {}

  /// @brief Construct transform from translation
  Transform3f(const Vec3f& T_) : R(Matrix3f::Identity()), T(T_) {}

  /// @brief Construct transform from other transform
  Transform3f(const Transform3f& tf) : R(tf.R), T(tf.T) {}

  /// @brief operator =
  Transform3f& operator=(const Transform3f& tf) {
    R = tf.R;
    T = tf.T;
    return *this;
  }

  /// @brief get translation
  inline const Vec3f& getTranslation() const { return T; }

  /// @brief get translation
  inline const Vec3f& translation() const { return T; }

  /// @brief get translation
  inline Vec3f& translation() { return T; }

  /// @brief get rotation
  inline const Matrix3f& getRotation() const { return R; }

  /// @brief get rotation
  inline const Matrix3f& rotation() const { return R; }

  /// @brief get rotation
  inline Matrix3f& rotation() { return R; }

  /// @brief get quaternion
  inline Quatf getQuatRotation() const { return Quatf(R); }

  /// @brief set transform from rotation and translation
  template <typename Matrix3Like, typename Vector3Like>
  inline void setTransform(const Eigen::MatrixBase<Matrix3Like>& R_,
                           const Eigen::MatrixBase<Vector3Like>& T_) {
    R.noalias() = R_;
    T.noalias() = T_;
  }

  /// @brief set transform from rotation and translation
  inline void setTransform(const Quatf& q_, const Vec3f& T_) {
    R = q_.toRotationMatrix();
    T = T_;
  }

  /// @brief set transform from rotation
  template <typename Derived>
  inline void setRotation(const Eigen::MatrixBase<Derived>& R_) {
    R.noalias() = R_;
  }

  /// @brief set transform from translation
  template <typename Derived>
  inline void setTranslation(const Eigen::MatrixBase<Derived>& T_) {
    T.noalias() = T_;
  }

  /// @brief set transform from rotation
  inline void setQuatRotation(const Quatf& q_) { R = q_.toRotationMatrix(); }

  /// @brief transform a given vector by the transform
  template <typename Derived>
  inline Vec3f transform(const Eigen::MatrixBase<Derived>& v) const {
    return R * v + T;
  }

  /// @brief transform a given vector by the inverse of the transform
  template <typename Derived>
  inline Vec3f inverseTransform(const Eigen::MatrixBase<Derived>& v) const {
    return R.transpose() * (v - T);
  }

  /// @brief inverse transform
  inline Transform3f& inverseInPlace() {
    R.transposeInPlace();
    T = -R * T;
    return *this;
  }

  /// @brief inverse transform
  inline Transform3f inverse() {
    return Transform3f(R.transpose(), -R.transpose() * T);
  }

  /// @brief inverse the transform and multiply with another
  inline Transform3f inverseTimes(const Transform3f& other) const {
    return Transform3f(R.transpose() * other.R, R.transpose() * (other.T - T));
  }

  /// @brief multiply with another transform
  inline const Transform3f& operator*=(const Transform3f& other) {
    T += R * other.T;
    R *= other.R;
    return *this;
  }

  /// @brief multiply with another transform
  inline Transform3f operator*(const Transform3f& other) const {
    return Transform3f(R * other.R, R * other.T + T);
  }

  /// @brief check whether the transform is identity
  inline bool isIdentity(
      const FCL_REAL& prec =
          Eigen::NumTraits<FCL_REAL>::dummy_precision()) const {
    return R.isIdentity(prec) && T.isZero(prec);
  }

  /// @brief set the transform to be identity transform
  inline void setIdentity() {
    R.setIdentity();
    T.setZero();
  }

  /// @brief return a random transform
  static Transform3f Random() { return Transform3f().setRandom(); }

  /// @brief set the transform to a random transform
  Transform3f& setRandom();

  bool operator==(const Transform3f& other) const {
    return (R == other.getRotation()) && (T == other.getTranslation());
  }

  bool operator!=(const Transform3f& other) const { return !(*this == other); }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename Derived>
inline Quatf fromAxisAngle(const Eigen::MatrixBase<Derived>& axis,
                           FCL_REAL angle) {
  return Quatf(Eigen::AngleAxis<FCL_REAL>(angle, axis));
}

/// @brief Uniformly random quaternion sphere.
/// Code taken from Pinocchio (https://github.com/stack-of-tasks/pinocchio).
inline Quatf uniformRandomQuaternion() {
  // Rotational part
  const FCL_REAL u1 = (FCL_REAL)rand() / RAND_MAX;
  const FCL_REAL u2 = (FCL_REAL)rand() / RAND_MAX;
  const FCL_REAL u3 = (FCL_REAL)rand() / RAND_MAX;

  const FCL_REAL mult1 = std::sqrt(FCL_REAL(1.0) - u1);
  const FCL_REAL mult2 = std::sqrt(u1);

  static const FCL_REAL PI_value = static_cast<FCL_REAL>(EIGEN_PI);
  FCL_REAL s2 = std::sin(2 * PI_value * u2);
  FCL_REAL c2 = std::cos(2 * PI_value * u2);
  FCL_REAL s3 = std::sin(2 * PI_value * u3);
  FCL_REAL c3 = std::cos(2 * PI_value * u3);

  Quatf q;
  q.w() = mult1 * s2;
  q.x() = mult1 * c2;
  q.y() = mult2 * s3;
  q.z() = mult2 * c3;
  return q;
}

inline Transform3f& Transform3f::setRandom() {
  const Quatf q = uniformRandomQuaternion();
  this->rotation() = q.matrix();
  this->translation().setRandom();

  return *this;
}

/// @brief Construct othonormal basis from vector.
/// The z-axis is the normalized input vector.
inline Matrix3f constructOrthonormalBasisFromVector(const Vec3f& vec) {
  Matrix3f basis = Matrix3f::Zero();
  basis.col(2) = vec.normalized();
  basis.col(1) = -vec.unitOrthogonal();
  basis.col(0) = basis.col(1).cross(vec);
  return basis;
}

}  // namespace fcl
}  // namespace hpp

#endif
