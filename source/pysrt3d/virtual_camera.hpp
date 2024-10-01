#pragma once

#include <srt3d/camera.h>
#include <srt3d/common.h>
#include <opencv2/opencv.hpp>
#include <queue>

namespace srt3d {

class VirtualCamera : public Camera{
public:
    VirtualCamera(std::string name) : Camera(name) {}
    ~VirtualCamera() {}

    void set_intrinsics(const srt3d::Intrinsics& intrinsics) {
        intrinsics_ = intrinsics;
    }

    bool SetUp() override {
        set_up_ = true;
        return true;
    }

    void PushImage(cv::Mat& image) {
        if(image.cols == 0 || image.rows == 0 || image.empty()) {
            std::cerr << "[warn] got empty image" << std::endl;
            return;
        }
        image_list_.push(image.clone());
    }

    void set_realtime(bool flag) {
        force_realtime_ = flag;
    }

    bool UpdateImage() override {
        if (image_list_.empty()) {
            return !image_.empty();
        }
        
        image_ = image_list_.front();
        image_list_.pop();
        // drop frames
        if(force_realtime_) {
            while(image_list_.size() > 0) {
                image_ = image_list_.front();
                image_list_.pop();
            }
        }

        return true;
    }

protected:
    std::queue<cv::Mat> image_list_;
    bool force_realtime_ = false;

}; // class Virtual Camera


} // namespace srt3d