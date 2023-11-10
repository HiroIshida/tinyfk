#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>

namespace tinyfk {

std::shared_ptr<fcl::BVHModel<fcl::OBBRSSf>>
load_fcl_geometry(const std::string &urdf_path, const std::string &mesh_path);

}
