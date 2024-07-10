#include "tinyfk.hpp"
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/mesh_loader/assimp.h>
#include <regex>
#include <stdexcept>

namespace tinyfk {

std::shared_ptr<hpp::fcl::BVHModel<hpp::fcl::OBBRSS>>
loadMesh(const std::string &filePath) {
  Assimp::Importer importer;
  const aiScene *scene = importer.ReadFile(filePath, aiProcess_Triangulate);
  if (!scene) {
    std::cerr << "Error: " << importer.GetErrorString() << std::endl;
    return nullptr;
  }

  auto model = std::make_shared<hpp::fcl::BVHModel<hpp::fcl::OBBRSS>>();
  for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
    const aiMesh *mesh = scene->mMeshes[i];

    std::vector<hpp::fcl::Triangle> triangles;
    std::vector<hpp::fcl::Vec3f> vertices;

    for (unsigned int j = 0; j < mesh->mNumVertices; ++j) {
      aiVector3D vertex = mesh->mVertices[j];
      vertices.emplace_back(vertex.x, vertex.y, vertex.z);
    }

    for (unsigned int j = 0; j < mesh->mNumFaces; ++j) {
      const aiFace &face = mesh->mFaces[j];
      if (face.mNumIndices != 3)
        continue; // Skip non-triangle faces
      triangles.emplace_back(face.mIndices[0], face.mIndices[1],
                             face.mIndices[2]);
    }

    model->beginModel();
    model->addSubModel(vertices, triangles);
    model->endModel();
  }

  model->computeLocalAABB();
  return model;
}

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
        auto mesh_model = loadMesh(mesh_fullpath);
      } else {
        throw std::runtime_error("unsupported geometry type");
      }
    }
  }
}

} // namespace tinyfk
