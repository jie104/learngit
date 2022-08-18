//
// Created by duan on 19-10-10.
//

#include "azure_camera.h"
#include <core/util/time.h>
#include <errno.h>
#include <fcntl.h>
#include <glog/logging.h>
#include <malloc.h>  //for calloc
#include <stdlib.h>  //for malloc
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <iostream>

namespace camera {

StdCamera::StdCamera() {}

StdCamera::~StdCamera() {}

bool StdCamera::open(int index) { return false; }

bool StdCamera::open(std::string filename) { return false; }

bool StdCamera::close() { return false; }

AzureCamera::AzureCamera():min_stamps(50) {}

AzureCamera::~AzureCamera() { close_device(); }

static void errno_exit(const char *s) {
  fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
  //    exit(EXIT_FAILURE);
}

bool AzureCamera::open_device(int index) {
  std::string dev_name =
      "/dev/video" +
      std::to_string(
          index); // "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._down-video-index0";

  return open_device(dev_name);
}

bool AzureCamera::open_device(std::string name) {
  //    boost::lock_guard<boost::shared_mutex> lock(camera_v4l2_mutex_);

  is_opened_ = false;

  // open device
  if ((fd_ = open(name.c_str(), O_RDWR /*| O_NONBLOCK*/, 0)) ==
      -1) { //阻塞式打开
    LOG(INFO) << "Open camera device error!!!";
    errno_exit("open_device: ");
    //        exit(-1);
    return false;
  }
  LOG(INFO) << "Device " << name << "Open is OK ";

  dev_name_ = name;
  is_opened_ = true;

  return true;
}

bool AzureCamera::close_device() {
  //    boost::lock_guard<boost::shared_mutex> lock(camera_v4l2_mutex_);

  if (isGrabed()) {
    uninitCapture();
  }

  if (isOpened()) {
    if (close(fd_) == -1) {
      errno_exit("close: ");
    } else {
      LOG(INFO) << "Camera closed success!!!";
    }
  } else {
    LOG(INFO) << "Camera not Open!!";
  }

  is_opened_ = false;
  is_grabed_ = false;

  return true;
}

void AzureCamera::decode(cv::Mat &frame) {
  cv::Mat dst;
  cv::imdecode(frame, cv::ImreadModes::IMREAD_GRAYSCALE, &dst);
  dst.copyTo(frame);
}

bool AzureCamera::initCamera() {
  if (!isOpened()) {
    LOG(INFO) << "Camera is not open!!";
    return false;
  }

  if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) == -1) {
    LOG(INFO) << "unable Querying Capabilities!!";
    return false;
  } else {
    LOG(INFO) << "Driver Caps:" << cap.driver;
    LOG(INFO) << "Driver:" << cap.card;
    LOG(INFO) << "Bus:" << cap.bus_info;
    LOG(INFO) << "Version:" << cap.version;
    LOG(INFO) << "Capabilities:" << cap.capabilities;
  }

  if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE) {
    LOG(INFO) << "Camera device :" << dev_name_ << " support capture";
  }
  if ((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING) {
    LOG(INFO) << "Camera device :" << dev_name_ << " support streaming";
  }

  // get image description
  fmtdesc.index = 0;
  fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  while (ioctl(fd_, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
    LOG(INFO) << "Support format is: " << fmtdesc.description;
    fmtdesc.index++;
  }

//  if(!setExposure(500))//TODO 设置手动曝光
//      return false;

  if (!setPixelFormat())
    return false;
  //    printPixelFormat();

  if (!setFps())
    return false;
  //    printFps();

  //    if(!setAutoExposure())
  //        return false;

  LOG(INFO) << "Camera Initialize Success!!";

  return true;
}

bool AzureCamera::initCapture() {
  if (!isOpened()) {
    LOG(INFO) << "Camera is not open and can not grab image!!";
    return false;
  }

  is_grabed_ = false;

  if (!requestBuffers())
    return false;

  if (!createBuffers()) {
    releaseBuffers();
    return false;
  }

  if (!turnOnStream())
    return false;

  is_grabed_ = true;

  return true;
}

bool AzureCamera::setImageSize(int width, int height) {
  width_ = width < 0 ? 640 : width;
  height_ = height < 0 ? 480 : height;

  return setPixelFormat();
}

bool AzureCamera::setGrabFps(int fps) {
  fps_ = fps;

  return setFps();
}

double AzureCamera::getImageProp(int property) { return 0.0; }

bool AzureCamera::setPixelFormat() {
  // set image format
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width_;
  fmt.fmt.pix.height = height_;
  fmt.fmt.pix.pixelformat = format;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (ioctl(fd_, VIDIOC_S_FMT, &fmt) == -1) {
    LOG(INFO) << "Setting Pixel format error!!!";
    errno_exit("VIDIOC_S_FMT");
    return false;
  } else {
    LOG(INFO) << "Setting image size is ok, width is " << width_
              << ", height is " << height_;
  }

  return true;
}

bool AzureCamera::printPixelFormat() {
  if (ioctl(fd_, VIDIOC_G_FMT, &fmt) == -1) {
    LOG(INFO) << "Getting pixel format is error!!!";
    return false;
  } else {
    LOG(INFO) << "fmt type is :" << fmt.type;
    LOG(INFO) << "fmt pixel format is :"
              << (fmt.fmt.pix.pixelformat & 0xFF,
                  (fmt.fmt.pix.pixelformat >> 8) & 0xFF,
                  (fmt.fmt.pix.pixelformat >> 16) & 0xFF,
                  (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
    LOG(INFO) << "fmt width is :" << fmt.fmt.pix.width;
    LOG(INFO) << "fmt height is :" << fmt.fmt.pix.height;
    LOG(INFO) << "fmt filed is :" << fmt.fmt.pix.field;
  }

  return true;
}

bool AzureCamera::setFps() {
  if (!isOpened())
    return false;

  setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  setfps.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
  setfps.parm.capture.timeperframe.numerator = 1;      //分子
  setfps.parm.capture.timeperframe.denominator = fps_; //分母

  if (ioctl(fd_, VIDIOC_S_PARM, &setfps) == -1) {
    LOG(INFO) << " Setting Fps error!!!";
    errno_exit("Setting Fps: ");
    return false;
  } else {
    LOG(INFO) << "fps setting is ok, fps is " << fps_;
  }

  return true;
}

bool AzureCamera::printFps() {
  if (ioctl(fd_, VIDIOC_G_PARM, &setfps) == -1) {
    LOG(INFO) << " Getting Fps error!!!";
    return false;
  } else {
    LOG(INFO) << "FPS is: " << setfps.parm.capture.timeperframe.denominator;
    LOG(INFO) << "FPS numerator is: "
              << setfps.parm.capture.timeperframe.numerator;
  }

  return true;
}

bool AzureCamera::isOpened() { return is_opened_; }

bool AzureCamera::isGrabed() { return is_grabed_; }

bool AzureCamera::readFrame(cv::Mat &frame, int64_t &frame_stamp, bool need_decode) {
  //    boost::lock_guard<boost::shared_mutex> lock(camera_v4l2_mutex_);

  if (!isOpened()) {
    LOG(INFO) << "VIDIOC_DQBUF ERROR!!!, camera is not open!!";
    return false;
  }

  fd_set fds;
  struct timeval tv;
  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  /* Timeout. */
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  int r = select(fd_ + 1, &fds, NULL, NULL, &tv);
  if (r == 0) {
    errno_exit("Select timeout 5s. ");
    return false;
  } else if (r == -1 && EINTR != errno) {
    errno_exit("Select: ");
    return false;
  }

  struct v4l2_buffer buf;
  CLEAR(buf);
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd_, VIDIOC_DQBUF, &buf) == -1) {
    LOG(INFO) << "VIDIOC_DQBUF ERROR!!!";
    errno_exit("VIDIOC_DQBUF: ");

    return false;
  }
  auto curr_time = sros::core::util::get_time_in_us();
  int64_t curr_stamp = buf.timestamp.tv_usec + buf.timestamp.tv_sec * 1e6;//当前读取值比camera落后一个周期,10ms
  auto delta_stamp = curr_time - curr_stamp;
  min_stamps.push_back(delta_stamp);
  auto min_delta_stamp = min_stamps.getMinValue();
  frame_stamp = curr_stamp + min_delta_stamp;
   if (abs((uint64_t )((uint64_t )curr_time - (uint64_t )frame_stamp) > (uint64_t )5e4)) {
      LOG(INFO) << "stamp is too large! will use curr time as stamp!";
      min_stamps.reset();
      frame_stamp = curr_time;
  }
  //    cv::Mat frame_buf = cv::Mat(1, buf.bytesused, CV_8UC3, (void*)buffers_[buf.index].start);
  cv::Mat frame_buf =
      cv::Mat(1, buf.bytesused, CV_8U, (void *)buffers_[buf.index].start);

  if (frame_buf.empty()) {
    LOG(INFO) << "v4l2 can not grab image, mat frame buf is empty!!";
    return false;
  } else {
    frame_buf.copyTo(frame);
  }

  if (need_decode && !frame.empty()) {
    decode(frame);
  }

  //TODO 根据成像效果调节曝光时间,以适配红外摄像头
    cv::Mat test_img = frame.clone();
    decode(test_img);
    cv::Mat gray_img;
    if(test_img.channels()==3){
        cv::cvtColor(test_img,gray_img,cv::COLOR_BGR2GRAY);
    } else{
        gray_img = test_img.clone();
    }
//    cv::imwrite("/sros/azure_imgtest.jpg",test_img);
    int meanGrayValue = cv::mean(gray_img)[0];
//    LOG(INFO)<<"Image Mean Gray Value: "<<meanGrayValue;

    double magnification;//更新策略
    if(meanGrayValue<100){
        magnification = 1.25;
    } else if(meanGrayValue<140){
        magnification = 1;
    } else{
        magnification = 0.75;
    }

    exposure = int(exposure*magnification);
    //过大导致无法实时
    if(exposure>max_exposure){
        exposure = max_exposure;
        LOG(WARNING)<<"exposure is max value : "<< max_exposure;
    }
    else if(exposure<5){
        exposure = 5;
    }

    if(!setExposure(exposure))
        return false;


  if (ioctl(fd_, VIDIOC_QBUF, &buf) == -1) {
    LOG(INFO) << "VIDIOC_QBUF ERROR!!!";
    errno_exit("VIDIOC_QBUF: ");
    return false;
  }

  return true;
}

bool AzureCamera::OpenAndReadImage(int index, cv::Mat &frame) {
  if (!open_device(index)) {
    return false;
  }

  if (!initCamera()) {
    return false;
  }

  if (!initCapture()) {
    return false;
  }

  int64_t frame_stamp;
  if (!readFrame(frame, frame_stamp))
    return false;

  return true;
}

bool AzureCamera::requestBuffers(int number) {
  req.count = number;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd_, VIDIOC_REQBUFS, &req) == -1) {
    LOG(INFO) << "Requesting Buffer error!!!";
    errno_exit("VIDIOC_REQBUFS: ");
    return false;
  }

  return true;
}

bool AzureCamera::createBuffers() {
  // mmap for buffer
  buffers_ = (Buffer *)calloc(req.count, sizeof(Buffer));
  if (!buffers_) {
    LOG(INFO) << "Out of memory!!!";
    return false;
  }

  uint32_t n_buffers = 0;
  for (n_buffers = 0; n_buffers < req.count; n_buffers++) {
    struct v4l2_buffer buf;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;

    if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1) {
      LOG(INFO) << " Querying Buffer error!!!";
      return false;
    }
    buffers_[n_buffers].length = buf.length;

    buffers_[n_buffers].start =
        (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                        fd_, buf.m.offset);

    if (buffers_[n_buffers].start == MAP_FAILED) {
      LOG(INFO) << "Buffer Map error!!!";
      return false;
    }

    // queue
    if (ioctl(fd_, VIDIOC_QBUF, &buf)) {
      LOG(INFO) << "queue buffer error!!!";
      return false;
    }
  }

  return true;
}

bool AzureCamera::releaseBuffers() {
  if (!isOpened())
    return false;

  for (int n_buffers = 0; n_buffers < req.count; n_buffers++) {
    if (buffers_[n_buffers].start) {
      if (munmap(buffers_[n_buffers].start, buffers_[n_buffers].length) == -1) {
        LOG(INFO) << "munmap error!!!";
        errno_exit("munmap: ");
        //                return false;
      } else {
        buffers_[n_buffers].start = 0;
      }
    }
  }

  free(buffers_);

  return false;
}

bool AzureCamera::turnOnStream() {
  // start stream
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
    LOG(INFO) << "Stream buffer error!!!";
    errno_exit("VIDIOC_STREAMON: ");
    return false;
  }

  return true;
}

bool AzureCamera::turnOffStream() {
  // off stream
  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_STREAMOFF, &type) == -1) {
    LOG(INFO) << "Stream buffer error!!!";
    errno_exit("VIDIOC_STREAMOFF: ");
    return false;
  }

  return true;
}

bool AzureCamera::reset() {
  turnOffStream();
  releaseBuffers();

  return initCapture();
}

bool AzureCamera::uninitCapture() {
  turnOffStream();
  releaseBuffers();
  requestBuffers(0);

  return true;
}

bool AzureCamera::setExposure(int exposure) {
  // 设置手动曝光模式
  ctrl.id = V4L2_CID_EXPOSURE_AUTO;
  ctrl.value = V4L2_EXPOSURE_MANUAL;
  if (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == -1) {
    LOG(INFO) << "Set exposure error!!";
    errno_exit("VIDIOC_S_CTRL: V4L2_EXPOSURE_MANUAL, ");
    return false;
  }

  //设置曝光值，单位us
  ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  ctrl.value = exposure;
  if (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == -1) {
    LOG(INFO) << "Set exposure error!!";
    errno_exit("VIDIOC_S_CTRL: V4L2_EXPOSURE_MANUAL, ");
    return false;
  }

  if (ioctl(fd_, VIDIOC_G_CTRL, &ctrl) == -1) {
  } else {
//    LOG(INFO) << "exposure is : " << ctrl.value;
  }

  return true;
}

bool AzureCamera::setAutoExposure() {
  //    setting.id = V4L2_CID_EXPOSURE;
  //    if(ioctl(fd_, VIDIOC_QUERYCTRL,&setting) == -1){
  //        errno_exit("VIDIOC_QUERYCTRL: ");
  //        return false;
  //    }
  //
  //    LOG(INFO) << setting.id;
  //    LOG(INFO) << setting.type;

  ctrl.id = V4L2_CID_EXPOSURE_AUTO;
  ctrl.value = V4L2_EXPOSURE_APERTURE_PRIORITY;

  if (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == -1) {
    LOG(INFO) << "Set exposure error!!";
    errno_exit("VIDIOC_S_CTRL: V4L2_CID_EXPOSURE_AUTO,  ");
    return false;
  }

  LOG(INFO) << "exposure mode is :" << ctrl.value;
  ctrl.value = V4L2_EXPOSURE_MANUAL;

  return true;
}

bool AzureCamera::setSaturation(int saturation) {
  ctrl.id = V4L2_CID_SATURATION;
  ctrl.value = saturation;
  if (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == -1) {
    LOG(INFO) << "Set saturation error!!";
    errno_exit("VIDIOC_G_CTRL: V4L2_CID_SATURATION,  ");
    return false;
  }

  LOG(INFO) << "saturation setting is ok!!!" << ctrl.value;

  return true;
}

int AzureCamera::getSaturation() {
  ctrl.id = V4L2_CID_SATURATION;
  if (ioctl(fd_, VIDIOC_G_CTRL, &ctrl) == -1) {
    LOG(INFO) << "Set saturation error!!";
    errno_exit("VIDIOC_G_CTRL: V4L2_CID_SATURATION,  ");
    return -1;
  }

  return ctrl.value;
}
}