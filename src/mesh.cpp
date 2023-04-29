#include "mesh.hpp"
#include <array>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace tinyfk {

struct Trimesh {
  std::vector<fcl::Vector3f> vertices;
  std::vector<fcl::Triangle> faces;
};

Trimesh merge_trimeshes(const std::vector<Trimesh> &trimeshes) {
  Trimesh trimesh_new;
  size_t idx_head = 0;
  for (const auto &trimesh : trimeshes) {
    for (const auto &v : trimesh.vertices) {
      trimesh_new.vertices.push_back(v);
    }
    for (const auto &f : trimesh.faces) {
      trimesh_new.faces.emplace_back(f[0] + idx_head, f[1] + idx_head,
                                     f[2] + idx_head);
    }
    idx_head += trimesh.vertices.size();
  }
  return trimesh_new;
}

Trimesh load_trimesh(const std::string &file_path) {
  Assimp::Importer importer;
  const aiScene *scene = importer.ReadFile(file_path, 0);

  if (scene == nullptr) {
    throw std::runtime_error("scene is nullptr");
  }
  if (scene->mNumMeshes == 0) {
    throw std::runtime_error("scene has 0 mesh");
  }

  std::vector<Trimesh> trimeshes;

  for (size_t i = 0; i < scene->mNumMeshes; ++i) {
    const auto mesh = scene->mMeshes[i];

    auto trimesh = Trimesh();

    for (size_t j = 0; j < mesh->mNumVertices; ++j) {
      double x = mesh->mVertices[j].x;
      double y = mesh->mVertices[j].y;
      double z = mesh->mVertices[j].z;
      trimesh.vertices.emplace_back(x, y, z);
    }

    for (size_t j = 0; j < mesh->mNumFaces; ++j) {
      size_t idx1 = mesh->mFaces[j].mIndices[0] + 1;
      size_t idx2 = mesh->mFaces[j].mIndices[1] + 1;
      size_t idx3 = mesh->mFaces[j].mIndices[2] + 1;
      trimesh.faces.emplace_back(idx1, idx2, idx3);
    }
    trimeshes.push_back(trimesh);
  }

  if (trimeshes.size() == 1) {
    return trimeshes.at(0);
  }

  const auto trimesh_merged = merge_trimeshes(trimeshes);
  return trimesh_merged;
}

std::string resolve_file_path(const std::string &urdf_file_path,
                              const std::string &mesh_file_path) {
  bool start_with_package = (mesh_file_path.rfind("package://", 0) == 0);
  std::string raw_path = mesh_file_path;
  if (start_with_package) {
    raw_path.erase(0, 10);
  }

  std::string base_directory =
      urdf_file_path.substr(0, urdf_file_path.find_last_of("/\\"));

  std::vector<std::string> prepath_candidates = {"./", "../", "../../"};
  for (auto &prepath : prepath_candidates) {
    auto fullpath = base_directory + "/" + prepath + raw_path;
    std::ifstream test_exist(fullpath);
    if (test_exist) {
      return fullpath;
    }
  }
  throw std::runtime_error("cannot resolve file path");
}

std::shared_ptr<fcl::BVHModel<fcl::OBBRSSf>>
load_fcl_geometry(const std::string &urdf_path, const std::string &mesh_path) {
  const auto mesh = load_trimesh(resolve_file_path(urdf_path, mesh_path));
  std::shared_ptr<fcl::BVHModel<fcl::OBBRSSf>> geom =
      std::make_shared<fcl::BVHModel<fcl::OBBRSSf>>();
  geom->beginModel();
  geom->addSubModel(mesh.vertices, mesh.faces);
  geom->endModel();
  return geom;
}

} // namespace tinyfk
