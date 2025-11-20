#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <errno.h>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

bool protonect_shutdown = false;
int v4l2_color = -1;
int v4l2_depth = -1;

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

bool init_v4l2_device(const char *device, int width, int height,int* stream,int pixformat)
{
  *stream = open(device, O_WRONLY);
  if (*stream < 0)
  {
    std::cerr << "Error opening v4l2loopback device: " << strerror(errno) << std::endl;
    return false;
  }

  struct v4l2_format v4l2_fmt;
  memset(&v4l2_fmt, 0, sizeof(v4l2_fmt));
  v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  v4l2_fmt.fmt.pix.width = width;
  v4l2_fmt.fmt.pix.height = height;
  v4l2_fmt.fmt.pix.pixelformat = pixformat;//V4L2_PIX_FMT_BGR24;
  v4l2_fmt.fmt.pix.sizeimage = width * height * 3;
  v4l2_fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (ioctl(*stream, VIDIOC_S_FMT, &v4l2_fmt) < 0)
  {
    std::cerr << "Error setting v4l2 format: " << strerror(errno) << std::endl;
    close(*stream);
    return -1;
  }

  return true;
}

void write_frame_to_v4l2(unsigned char *rgb_data, int width, int height,int* stream)
{
  if (*stream < 0)
    return;

  cv::Mat frame(height, width, CV_8UC4, rgb_data);
  cv::Mat bgr_frame;
  cv::cvtColor(frame, bgr_frame, cv::COLOR_BGRA2BGR);

  write(*stream, bgr_frame.data, width * height * 3);
}

void write_frame_to_depth(unsigned char *rgb_data, int width, int height,int* stream)
{
  if (*stream < 0)
    return;

  cv::Mat frame(height, width, CV_32SC1, rgb_data); //CV_16UC1
  cv::Mat bgr_frame;
  cv::cvtColor(frame, bgr_frame, cv::COLOR_GRAY2BGR );

  write(*stream, bgr_frame.data, width * height * 3);
}

int main(int argc, char *argv[])frame.data
{
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = nullptr;
  libfreenect2::PacketPipeline *pipeline = nullptr;

  
  if (!init_v4l2_device("/dev/video10", 1920, 1080,&v4l2_color,V4L2_PIX_FMT_BGR24) < 0)
  {
    std::cerr << "Failed to initialize v4l2loopback device" << std::endl;
    return -1;
  }

  if (!init_v4l2_device("/dev/video11", 512, 424,&v4l2_depth,V4L2_PIX_FMT_BGR24) < 0) //V4L2_PIX_FMT_Y16
  {
    std::cerr << "Failed to initialize v4l2loopback-depth device" << std::endl;
    return -1;
  }

  if (freenect2.enumerateDevices() == 0)
  {
    std::cout << "No device connected!" << std::endl;
    return -1;
  }

  std::string serial = freenect2.getDefaultDeviceSerialNumber();

  dev = freenect2.openDevice(serial);

  if (dev == nullptr)
  {
    std::cout << "Failure opening device!" << std::endl;
    return -1;
  }

  signal(SIGINT, sigint_handler);
   int types = libfreenect2::Frame::Color | libfreenect2::Frame::Depth | libfreenect2::Frame::Ir;
  libfreenect2::SyncMultiFrameListener listener(types);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);

  /// [registration setup]
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
  /// [registration setup]

  if (!dev->start())
    return -1;

  std::cout << "Device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "Device firmware: " << dev->getFirmwareVersion() << std::endl;

  while (!protonect_shutdown)
  {
    if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 seconds
    {
      std::cout << "Timeout!" << std::endl;
      return -1;
    }
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    /// [registration]
      registration->apply(rgb, depth, &undistorted, &registered);
    /// [registration]
    
    write_frame_to_v4l2(rgb->data, rgb->width, rgb->height,&v4l2_color);
    write_frame_to_v4l2(depth->data, depth->width, depth->height,&v4l2_depth);




    listener.release(frames);
  }

  dev->stop();
  dev->close();
  close(v4l2_color);
  close(v4l2_depth);
  return 0;
}
