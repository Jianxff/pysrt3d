#pragma once 

#include <srt3d/body.h>
#include <srt3d/common.h>
#include <srt3d/normal_viewer.h>
#include <srt3d/occlusion_renderer.h>
#include <srt3d/region_modality.h>
#include <srt3d/renderer_geometry.h>
#include <srt3d/tracker.h>

#include "virtual_camera.hpp"

#include <Eigen/Geometry>
#include <filesystem>
#include <unordered_map>
#include <memory>
#include <string>

#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

// convert numpy array to cv::Mat
cv::Mat convert_numpy_cvmat(const py::array_t<uint8_t>& input, bool gray_scale = false);
// calculate pose uv
cv::Point2i calculate_pose_uv(const Eigen::Matrix4f& pose, const Eigen::Matrix3f& K);


class RegionModel{
public:
    // constructor
    RegionModel(const std::string& name,
        const std::string& geometry_path, 
        const std::string& meta_path = "",
        const float unit_in_meter = 1.0, 
        const float shpere_radius = 0.8,
        const bool from_opengl = false,
        const Eigen::Matrix4f& geometry2body = Eigen::Matrix4f::Identity(),
        const float threshold_on_init = 0.8,
        const float threshold_on_track = 0.5,
        const float kl_threshold = 1.0,
        const bool debug_visualize = false
    );
    // setup tracker
    void Setup();

    // variables
    const std::string name_;
    std::shared_ptr<srt3d::Body> body_ptr_;
    std::shared_ptr<srt3d::Model> model_ptr_;
    std::shared_ptr<srt3d::RegionModality> region_modality_ptr_ = nullptr;
    float threshold_on_init_;
    float threshold_on_track_;
    float kl_threshold_ = 1.0;
    bool debug_visualize_;

    // states
    float conf_ = 0.0;
    Eigen::Matrix4f last_pose_ = Eigen::Matrix4f::Identity();
    int state_ = 1;
    cv::Point2i uv_;

    // methods
    // void set_target_rotation(const Eigen::Matrix3f& rot);
    void set_kl_threshold(const float kl_threshold);
    void reset_pose(const Eigen::Matrix4f& pose = Eigen::Matrix4f::Identity());
    const Eigen::Matrix4f pose();
    const Eigen::Matrix4f pose_gl();
    bool validate();
    float valid_percentage();

}; // class Model


class RegionTracker{
public:
    RegionTracker(const int imwidth, const int imheight, 
            const Eigen::Matrix3f& K = Eigen::Matrix3f::Zero(),
            const float fx = 0.0, const float fy = 0.0,
            const float cx = 0.0, const float cy = 0.0,
            const int corr_update_iter = 7, const int pose_update_iter = 2);
    // add region model
    void add_model(const std::shared_ptr<RegionModel>& region_model);
    // track
    void track(const py::array_t<uint8_t>& image, std::optional<py::array_t<uint8_t>> mask);
    // setup
    void setup();
    // delete
    void terminate();

    int imwidth_;
    int imheight_;
    int focal_;
    Eigen::Matrix3f K_;

    std::vector<std::shared_ptr<RegionModel>> models_;
    std::shared_ptr<srt3d::Tracker> tracker_ptr_;
    std::shared_ptr<srt3d::VirtualCamera> camera_ptr_;

}; // class tracker


class RegionRenderer{
public:
    RegionRenderer(const std::shared_ptr<RegionTracker>& region_tracker_ptr);
    py::array_t<uint8_t> render();

    std::shared_ptr<RegionTracker> region_tracker_;
    std::shared_ptr<srt3d::RendererGeometry> renderer_geometry_ptr_;
    std::shared_ptr<srt3d::NormalViewer> viewer_ptr_;
    std::shared_ptr<srt3d::VirtualCamera> camera_ptr_;
}; // class renderer


