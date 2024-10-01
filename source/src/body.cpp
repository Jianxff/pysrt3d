// SPDX-License-Identifier: MIT
// Copyright (c) 2021 Manuel Stoiber, German Aerospace Center (DLR)

#include <srt3d/body.h>

// #define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader/tiny_obj_loader.h>

namespace srt3d {

Body::Body(const std::string &name, const std::filesystem::path &geometry_path,
           float geometry_unit_in_meter, bool geometry_counterclockwise,
           bool geometry_enable_culling, const Transform3fA &geometry2body_pose)
    : name_{name},
      geometry_path_{geometry_path},
      geometry_unit_in_meter_{geometry_unit_in_meter},
      geometry_counterclockwise_{geometry_counterclockwise},
      geometry_enable_culling_{geometry_enable_culling},
      geometry2body_pose_{geometry2body_pose}
{
  geometry2world_pose_ = geometry2body_pose;
  world2geometry_pose_ = geometry2world_pose_.inverse();
  calculate_maximum_body_diameter();
}

void Body::set_name(const std::string &name) { name_ = name; }

void Body::set_geometry_path(const std::filesystem::path &geometry_path) {
  geometry_path_ = geometry_path;
}

void Body::set_geometry_unit_in_meter(float geometry_unit_in_meter) {
  geometry_unit_in_meter_ = geometry_unit_in_meter;
}

void Body::set_geometry_counterclockwise(bool geometry_counterclockwise) {
  geometry_counterclockwise_ = geometry_counterclockwise;
}

void Body::set_geometry_enable_culling(bool geometry_enable_culling) {
  geometry_enable_culling_ = geometry_enable_culling;
}

// void Body::set_maximum_body_diameter(float maximum_body_diameter) {
//   maximum_body_diameter_ = maximum_body_diameter;
// }

void Body::set_geometry2body_pose(const Transform3fA &geometry2body_pose) {
  geometry2body_pose_ = geometry2body_pose;
  geometry2world_pose_ = body2world_pose_ * geometry2body_pose_;
  world2geometry_pose_ = geometry2world_pose_.inverse();
}

// bool Body::set_occlusion_id(int occlusion_id) {
//   if (occlusion_id > 7) {
//     std::cout << "Invalid value for occlusion id. Has to be <= 7." << std::endl;
//     return false;
//   }
//   occlusion_id_ = occlusion_id;
//   return true;
// }

void Body::set_body2world_pose(const Transform3fA &body2world_pose) {
  body2world_pose_ = body2world_pose;
  world2body_pose_ = body2world_pose_.inverse();
  geometry2world_pose_ = body2world_pose_ * geometry2body_pose_;
  world2geometry_pose_ = geometry2world_pose_.inverse();
}

void Body::set_world2body_pose(const Transform3fA &world2body_pose) {
  world2body_pose_ = world2body_pose;
  body2world_pose_ = world2body_pose_.inverse();
  geometry2world_pose_ = body2world_pose_ * geometry2body_pose_;
  world2geometry_pose_ = geometry2world_pose_.inverse();
}

const std::string &Body::name() const { return name_; }

int Body::occlusion_id() const { return occlusion_id_; }

const std::filesystem::path &Body::geometry_path() const {
  return geometry_path_;
}

float Body::geometry_unit_in_meter() const { return geometry_unit_in_meter_; }

bool Body::geometry_counterclockwise() const {
  return geometry_counterclockwise_;
}

bool Body::geometry_enable_culling() const { return geometry_enable_culling_; }

float Body::maximum_body_diameter() const { return maximum_body_diameter_; }

const Transform3fA &Body::geometry2body_pose() const {
  return geometry2body_pose_;
}

const Transform3fA &Body::body2world_pose() const { return body2world_pose_; }

const Transform3fA &Body::world2body_pose() const { return world2body_pose_; }

const Transform3fA &Body::geometry2world_pose() const {
  return geometry2world_pose_;
}

const Transform3fA &Body::world2geometry_pose() const {
  return world2geometry_pose_;
}

void Body::calculate_maximum_body_diameter() {
  // load mesh data
  tinyobj::attrib_t attributes;
  std::vector<Eigen::Vector3f> vertices;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warning;
  std::string error;

  if (!tinyobj::LoadObj(&attributes, &shapes, &materials, &warning, &error,
                        geometry_path_.string().c_str(), nullptr, true,
                        false)) {
    std::cerr << "TinyObjLoader failed to load data from " << geometry_path_
              << std::endl;
    return;
  }
  if (!error.empty()) std::cerr << error << std::endl;

  vertices.resize(attributes.vertices.size() / 3);
  memcpy(vertices.data(), attributes.vertices.data(),
         sizeof(float) * attributes.vertices.size());
  if (geometry_unit_in_meter_ != 1.0f) {
    for (auto &vertex : vertices) {
      vertex *= geometry_unit_in_meter_;
    }
  }

  // calculate maximum body diameter
  float max_radius = 0.0f;
  for (const auto &vertex : vertices) {
    max_radius = std::max(max_radius, (geometry2body_pose_ * vertex).norm());
  }
  maximum_body_diameter_ = 2.2f * max_radius;
  // std::cout << "Maximum body diameter: " << maximum_body_diameter_ << std::endl;

}

}  // namespace srt3d
