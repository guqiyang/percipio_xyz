//this example shows how to trigger device and caputre image by software
//the difference from continous mode is :
//1.config device to trigger mode
//2.for software trigger  capture data : using  FramePackageGetSync function
//3.for hardware trigger  : using FramePackageGet function to get frame 
//

#define _CRT_SECURE_NO_WARNINGS

#ifdef _WIN32
#include <time.h>
#include <windows.h>
#else
#include <sys/time.h>
#endif

#include <percipio_camport.h>
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include "depth_render.h"
#include <fstream>

static cv::Mat left, right, depth, point_cloud;
static DepthRender render;
static int fps_counter = 0;
static int frame_total = 0;
static clock_t fps_tm = 0;
static volatile bool continous_saving = false;
static std::ofstream ofs;


void process_frames(void *);
int get_fps();
void convert_buffer(percipio::ImageBuffer *pbuf, cv::Mat &img);
int init_device(percipio::DepthCameraDevice &port);



int main(int argc, char** argv) {
  percipio::DepthCameraDevice port;
  //init render
  render.range_mode = DepthRender::COLOR_RANGE_DYNAMIC;
  render.color_type = DepthRender::COLORTYPE_BLUERED;
  render.invalid_label = 0;
  render.Init();
  //init device
  int ret = init_device(port);
  if (ret != 0) {
    return ret;
  }

  //display a empty window for receiving key input
  cv::imshow("depth", cv::Mat::zeros(100, 100, CV_8UC1));
  fps_tm = clock();
  fps_counter = 0;
  continous_saving = false;
  while (true) {
    process_frames(&port);
    int k = cv::waitKey(1);
    if (k == 'q' || k == 1048689) {
      break;
    }
  }//while
  port.CloseDevice();
  //release all buffers
  left.release();
  right.release();
  depth.release();
  point_cloud.release();
  render.Uninit();
  return 0;
}

int init_device(percipio::DepthCameraDevice &port) {
  percipio::SetLogLevel(percipio::LOG_LEVEL_INFO);

  int ver = percipio::LibVersion();
  printf("camport api version is %d\n", ver);

  int num = port.GetDeviceNum();
  int idx = 1;
  if (num > 1) {
    printf("current device number:%d\n please select one:", num);
    scanf("%d", &idx);
  }
  int ret = port.OpenDevice(idx);
  if (percipio::CAMSTATUS_SUCCESS != ret) {
    printf("open device failed\n");
    return -1;
  }

  int wait_time;
  port.GetProperty(percipio::PROP_WAIT_NEXTFRAME_TIMEOUT, (char *)&wait_time, sizeof(int));
  printf("get property PROP_WAIT_NEXTFRAME_TIMEOUT %d\n", wait_time);

  int reti = port.SetProperty_Bool(percipio::PROP_TRIGGER_MODE, true);
  if (reti < 0) {
    printf("set PROP_TRIGGER_MODE failed,error code:%d\n", reti);
    return -1;
  }

  reti = port.SetProperty_Int(percipio::PROP_WORKMODE, percipio::WORKMODE_IR_DEPTH);
  if (reti < 0) {
    printf("set mode failed,error code:%d\n", reti);
    return -1;
  }

  ofs.open("log.txt");
  return 0;
}

void process_frames(void *p) {
  percipio::DepthCameraDevice &port = *(percipio::DepthCameraDevice *)p;
  int res = port.FramePackageGet();
  if (res != percipio::CAMSTATUS_SUCCESS) {
    return;
  }
  percipio::ImageBuffer pimage;
  int ret = port.FrameGet(percipio::CAMDATA_LEFT, &pimage);
  if (percipio::CAMSTATUS_SUCCESS == ret) {
    convert_buffer(&pimage, left);
    cv::imshow("left", left);
  }
  ret = port.FrameGet(percipio::CAMDATA_RIGHT, &pimage);
  if (percipio::CAMSTATUS_SUCCESS == ret) {
    convert_buffer(&pimage, right);
    cv::imshow("right", right);
  }
  ret = port.FrameGet(percipio::CAMDATA_DEPTH, &pimage);
  if (percipio::CAMSTATUS_SUCCESS == ret) {
    int c = pimage.timestamp;
    int frame_index = pimage.frame_index;
    frame_total++;
    printf("%d arrived at \t%d \tindex = %d\n", frame_total, c, frame_index);
    if (ofs.is_open()) {
      ofs << c << std::endl;
    }
    convert_buffer(&pimage, depth);
    cv::Mat t;
    int fps = get_fps();
    render.Compute(depth, t);
    cv::imshow("depth", t);
    if (fps > 0) {
      unsigned short v = depth.ptr<unsigned short>(depth.rows / 2)[depth.cols / 2];
      //printf("fps:%d distance: %d\n", (int)fps, v);
    }
  }
  ret = port.FrameGet(percipio::CAMDATA_POINT3D, &pimage);
  if (percipio::CAMSTATUS_SUCCESS == ret) {
    convert_buffer(&pimage, point_cloud);
  }
  continous_saving = false;
}

#ifdef _WIN32
int get_fps() {
  const int kMaxCounter = 20;
  fps_counter++;
  if (fps_counter < kMaxCounter) {
    return -1;
  }
  int elapse = (clock() - fps_tm);
  int v = (int)(((float)fps_counter) / elapse * CLOCKS_PER_SEC);
  fps_tm = clock();

  fps_counter = 0;
  return v;
}
#else
int get_fps() {
  const int kMaxCounter = 20;
  struct timeval start;
  fps_counter++;
  if (fps_counter < kMaxCounter) {
    return -1;
  }

  gettimeofday(&start, NULL);
  int elapse = start.tv_sec * 1000 + start.tv_usec / 1000 - fps_tm;
  int v = (int)(((float)fps_counter) / elapse * 1000);
  gettimeofday(&start, NULL);
  fps_tm = start.tv_sec * 1000 + start.tv_usec / 1000;

  fps_counter = 0;
  return v;
}
#endif

void convert_buffer(percipio::ImageBuffer *pbuf, cv::Mat &img) {
  switch (pbuf->type) {
  case percipio::ImageBuffer::PIX_8C1:
    img.create(pbuf->height, pbuf->width, CV_8UC1);
    break;
  case percipio::ImageBuffer::PIX_16C1:
    img.create(pbuf->height, pbuf->width, CV_16UC1);
    break;
  case percipio::ImageBuffer::PIX_8C3:
    img.create(pbuf->height, pbuf->width, CV_8UC3);
    break;
  case percipio::ImageBuffer::PIX_32FC3:
    img.create(pbuf->height, pbuf->width, CV_32FC3);
    break;
  default:
    img.release();

    return;
  }
  memcpy(img.data, pbuf->data, pbuf->width * pbuf->height * pbuf->get_pixel_size());
}
