#include "tinyfk.hpp"
#include <regex>
#include <stdexcept>

namespace tinyfk {

void KinematicModel::load_collision_objects() {
  // urdf_path_ is None
  for (auto &link : links_) {
    std::cout << "hoge" << std::endl;
    if (link->collision != nullptr) {
      auto &origin = link->collision->origin;
      auto geometry = link->collision->geometry;
      Eigen::Vector3d position;
      position << origin.position.x, origin.position.y, origin.position.z;
      std::string msg = "parsing " + link->name + ": ";

      if (position.norm() > 1e-6) {
        msg += "TODO: unsupported collision object position";
        std::cout << msg << std::endl;
        continue;
      }
      Eigen::Vector4d rotation;
      rotation << origin.rotation.x, origin.rotation.y, origin.rotation.z,
          origin.rotation.w;
      if (rotation(3) != 1.0) {
        throw std::runtime_error("TODO: unsupported collision object rotation");
      }
      if (geometry->type == urdf::Geometry::BOX) {
        auto box = std::dynamic_pointer_cast<urdf::Box>(geometry);
        std::cout << msg + "TODO: box is not supported" << std::endl;
        continue;
      } else if (geometry->type == urdf::Geometry::CYLINDER) {
        auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
        std::cout << msg + "TODO: cylinder is not supported" << std::endl;
        continue;
      } else if (geometry->type == urdf::Geometry::SPHERE) {
        auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(geometry);
        std::cout << msg + "TODO: sphere is not supported" << std::endl;
        continue;
      } else if (geometry->type == urdf::Geometry::MESH) {
        if (!urdf_path_.has_value()) {
          throw std::runtime_error(
              "urdf_path must be provided for collision object loading");
        }
        auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geometry);
        std::cout << msg + "parse mesh file " + mesh->filename;
        std::regex regexPattern(R"(meshes/[^/]+\.STL)");
        std::smatch matches;
        if (std::regex_search(mesh->filename, matches, regexPattern)) {
          std::cout << "Found: " << matches[0] << std::endl;
        } else {
          throw std::runtime_error(msg + "unsupported mesh file format");
        }
        std::string mesh_fullpath = urdf_path_.value().parent_path() /
                                    std::filesystem::path(matches[0]);
        std::cout << mesh_fullpath << std::endl;
      } else {
        throw std::runtime_error("unsupported geometry type");
      }
    }
  }
}

} // namespace tinyfk
