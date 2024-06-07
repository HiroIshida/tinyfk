#include "urdf_model/pose.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <tinyfk.hpp>

struct EigenTransform {
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;

  EigenTransform operator*(const EigenTransform &other) const {
    Eigen::Quaterniond &&rot = this->rotation * other.rotation;
    Eigen::Vector3d &&trans =
        this->rotation * other.translation + this->translation;
    return EigenTransform{trans, rot};
  }

  Eigen::Vector3d operator*(const Eigen::Vector3d &other) const {
    return this->rotation * other + this->translation;
  }

  EigenTransform inverse() const {
    EigenTransform result;
    result.rotation = this->rotation.inverse();
    result.translation = -(result.rotation * this->translation);
    return result;
  }

  EigenTransform inverse_in_place() {
    this->rotation = this->rotation.inverse();
    this->translation = -(this->rotation * this->translation);
    return *this;
  }

  tinyfk::Transform toUrdf() const {
    tinyfk::Transform pose;
    tinyfk::Rotation rot{this->rotation.x(), this->rotation.y(),
                         this->rotation.z(), this->rotation.w()};
    tinyfk::Vector3 vec{this->translation.x(), this->translation.y(),
                        this->translation.z()};
    pose.rotation = rot;
    pose.position = vec;
    return pose;
  }

  friend std::ostream &operator<<(std::ostream &os, const EigenTransform &t) {
    os << "Translation: " << t.translation.transpose() << std::endl;
    os << "Rotation: " << t.rotation.coeffs().transpose() << std::endl;
    return os;
  }
};

EigenTransform urdf_to_eigen(const tinyfk::Transform &pose) {
  Eigen::Vector3d translation{pose.position.x, pose.position.y,
                              pose.position.z};
  Eigen::Quaterniond rotation{pose.rotation.w, pose.rotation.x, pose.rotation.y,
                              pose.rotation.z};
  return EigenTransform{translation, rotation};
}

int main() {
  auto rot1 =
      Eigen::Quaterniond{Eigen::AngleAxisd{0.3, Eigen::Vector3d::UnitX()}};
  auto rot2 =
      Eigen::Quaterniond{Eigen::AngleAxisd{0.6, Eigen::Vector3d::UnitY()}};
  auto t1 = EigenTransform{Eigen::Vector3d{0.1, 0, 0}, rot1};
  auto t2 = EigenTransform{Eigen::Vector3d{0, 0.2, 0.3}, rot2};
  auto tt1 = t1.toUrdf();
  auto tt2 = t2.toUrdf();

  // use eigen affine transform
  Eigen::Affine3d ttt1;
  ttt1.translation() = t1.translation;
  ttt1.linear() = t1.rotation.toRotationMatrix();
  Eigen::Affine3d ttt2;
  ttt2.translation() = t2.translation;
  ttt2.linear() = t2.rotation.toRotationMatrix();

  // apply transform t2 to t1 10000 times
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 100000000; i++) {
    t1 = std::move(t1 * t2);
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "Eigen Transform: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms" << std::endl;
  std::cout << t1 << std::endl;

  // apply transform tt2 to tt1 10000 times
  start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 100000000; i++) {
    tt1 = std::move(urdf::pose_transform(tt1, tt2));
  }
  end = std::chrono::high_resolution_clock::now();
  std::cout << "tinyfk Transform: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms" << std::endl;
  std::cout << urdf_to_eigen(tt1) << std::endl;

  // apply transform ttt2 to ttt1 10000 times
  start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 100000000; i++) {
    ttt1 = std::move(ttt1 * ttt2);
  }
  end = std::chrono::high_resolution_clock::now();
  std::cout << "Affine Transform: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms" << std::endl;
  std::cout << "Translation: " << ttt1.translation().transpose() << std::endl;
  std::cout << "Rotation: "
            << Eigen::Quaterniond{ttt1.linear()}.coeffs().transpose()
            << std::endl;
}
