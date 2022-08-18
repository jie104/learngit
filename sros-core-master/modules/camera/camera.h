//
// Created by duan on 19-10-10.
//

#ifndef PGV_CAMERA_H
#define PGV_CAMERA_H

#include <linux/videodev2.h>
#include <boost/thread/shared_mutex.hpp>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include "camera_device_manage.hpp"
#include "core/circle_optimizer_set.hpp"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

class V4L2Camera {
 public:
    struct Buffer {
        void *start;
        size_t length;
    };

    V4L2Camera();

    ~V4L2Camera();

    //todo:接口修改完成后，需要将这三个函数改成私有化函数
    bool open_device(int index);
    bool open_device(std::string filename);
    bool close_device();

    bool initCamera();

    bool initCapture();
    bool uninitCapture();

    bool isOpened();
    bool isGrabed();

    static bool getAllSVC100Devices(std::map<std::string, Device::CameraDevice> &svc100_devices,int& totally_count);

    bool openParticularSVC100(const std::string &camera_sn);

    /**
     *
     * @param camera_sn
     * @param state ,相机是正常关闭、还是掉线；正常关闭后可以读取参数、掉线后不能读取参数
     * @return
     */
    bool closeParticularSVC100(const std::string &camera_sn, const Device::CameraState &state);

    bool readFrame(cv::Mat &frame, int64_t &frame_stamp, bool need_decode = true);

    void decode(cv::Mat &frame);

    bool OpenAndReadImage(int index, cv::Mat &frame);

    bool setImageSize(int width, int height);
    bool setGrabFps(int fps);

    double getImageProp(int property);

    bool setAutoExposure();

    /**
     * 单位为us, 对于SVC100相机设置没有作用
     * @param exposure
     * @return
     */
    bool setExposure(int exposure);

    bool setSaturation(int saturation);
    int getSaturation();

 private:
    bool requestBuffers(int number = 2);
    bool createBuffers();
    bool releaseBuffers();

    bool reset();

    bool turnOnStream();
    bool turnOffStream();

    bool setPixelFormat();
    bool printPixelFormat();

    bool setFps();
    bool printFps();

 private:
    // v4l2 struct
    v4l2_streamparm setfps;
    v4l2_capability cap;      // querying capabilities
    v4l2_fmtdesc fmtdesc;     // image format description
    v4l2_format fmt, fmtack;  // v4l2 format
    v4l2_requestbuffers req;
    //    v4l2_buffer buf;
    v4l2_control ctrl;
    v4l2_queryctrl setting;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    int width_ = 640;
    int height_ = 480;
    int format = V4L2_PIX_FMT_MJPEG;
    int depth_ = 3;
    int fps_ = 100;

    int fd_;
    //    uint8_t *buffer_;
    struct Buffer *buffers_;

    bool is_opened_ = false;
    bool is_grabed_ = false;

    std::string dev_name_;
    int buffer_size_ = 2;

    static Device::CameraManagedPtr camera_device_ptr;

    circle::CircleOptimizerArray<int64_t> delta_timestamp_;
    //    mutable boost::shared_mutex camera_v4l2_mutex_;
};

typedef std::shared_ptr<V4L2Camera> v4l2_camera_ptr;

#endif  // PGV_CAMERA_H
