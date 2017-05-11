// camport_test.cpp : Defines the entry point for the console application.
//

#define _CRT_SECURE_NO_WARNINGS

#ifdef _WIN32
#include <time.h>
#else
#include <sys/time.h>
#endif

#include "percipio_camport.h"
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include "depth_render.h"
#include "slambase.h"

static cv::Mat left, right, depth, point_cloud;
static DepthRender render;
static int fps_counter = 0;
static clock_t fps_tm = 0;
static CameraCompute camera_mine;
void process_frames(percipio::DepthCameraDevice &port);
int init_device(percipio::DepthCameraDevice &port);
void save_frame_to_file();
int get_fps();
void copy_buffer(percipio::ImageBuffer *pbuf, cv::Mat &img);

int main(int argc, char** argv) {
  percipio::DepthCameraDevice port(percipio::MODEL_DPB04GN);    //create instance of class DepthCameraDevice 
  //DepthRender can convert depth image into rgb color image for display
  render.range_mode = DepthRender::COLOR_RANGE_DYNAMIC;         //there is two types of render modes
  render.color_type = DepthRender::COLORTYPE_BLUERED;           //set color type
  render.invalid_label = 0;
  render.Init();                                                //render initializes
  int res = init_device(port);                                  //init device  
  if (res != 0) {
    return res;
  }
  //display a window for receiving key input
  cv::imshow("depth", cv::Mat::zeros(100, 100, CV_8UC1));
  fps_tm = clock();
  fps_counter = 0;
  while (true) {
    //fetch frames
    if (port.FramePackageGet() == percipio::CAMSTATUS_SUCCESS) {
      process_frames(port);
    }
    int k = cv::waitKey(1);
    if (k == 'q' || k == 1048689) {
      break;
    }
    if (k == 's' || k == 1048691) {
      save_frame_to_file();
    }
  }//while
  port.CloseDevice();
  //release opencv image buffers
  left.release();
  right.release();
  depth.release();
  point_cloud.release();
  render.Uninit();
  return 0;
}


void frame_arrived_callback(void *user_data) {
  // call port.FramePackageGet to update internal buffer
  // call port.FrameGet to get frame data here
  // To avoid performance problem ,time consuming task in callback function is not recommended.

  //note:when you want to use callback,you need to stop calling FramePackageGet in main loop
}


int init_device(percipio::DepthCameraDevice &port) {
  percipio::SetLogLevel(percipio::LOG_LEVEL_INFO);              //set log level for libcamm
  port.SetCallbackUserData(NULL);                               //set data for call back function
  //uncomment below if you need callback
  //port.SetFrameReadyCallback(frame_arrived_callback);

  int ver = percipio::LibVersion();
  printf("camport api version is %d\n", ver);

  int num = port.GetDeviceNum();
  int id = 1;
  if (num > 1){
    printf("device num = %d\n select one device(id start from 1):", num);
    scanf("%d", &id);
  }
  int ret = port.OpenDevice(id);
  if (percipio::CAMSTATUS_SUCCESS != ret) {
    printf("open device failed\n");
    return -1;
  }

  int wait_time;
  port.GetProperty(percipio::PROP_WAIT_NEXTFRAME_TIMEOUT, (char *)&wait_time, sizeof(int));
  printf("get property PROP_WAIT_NEXTFRAME_TIMEOUT %d\n", wait_time);


  //int reti = port.SetProperty_Int(percipio::PROP_WORKMODE, percipio::WORKMODE_DEPTH);         //set work mode, depth mode here, output depth data
  //int reti = port.SetProperty_Int(percipio::PROP_WORKMODE, percipio::WORKMODE_IR);          //uncomment if need ir mode, output left and right ir data 
  int reti = port.SetProperty_Int(percipio::PROP_WORKMODE, percipio::WORKMODE_IR_DEPTH);    //uncomment if need ir_depth mode, output left ir, right ir and depth data,
  port.SetPointCloudOutput(1);
  if (reti < 0) {
    printf("set mode failed,error code:%d\n", reti);
    return -1;
  }

  return 0;
}

void process_frames(percipio::DepthCameraDevice &port) {
  percipio::ImageBuffer pimage;
  PointCloud::Ptr cloud_tmp;
  int ret = port.FrameGet(percipio::CAMDATA_LEFT, &pimage);                                //get left ir data if exist
  if (percipio::CAMSTATUS_SUCCESS == ret) {
    copy_buffer(&pimage, left);
    cv::imshow("left", left);
  }
  ret = port.FrameGet(percipio::CAMDATA_RIGHT, &pimage);                                   //get right ir data if exist
  if (percipio::CAMSTATUS_SUCCESS == ret) {
    copy_buffer(&pimage, right);
    cv::imshow("right", right);
  }
  ret = port.FrameGet(percipio::CAMDATA_DEPTH, &pimage);                                   //get depth data if exist 
  if (percipio::CAMSTATUS_SUCCESS == ret) {
    copy_buffer(&pimage, depth);
    cv::Mat t;
    int fps = get_fps();
    render.Compute(depth, t);                                                              //render raw depth data
    cv::imshow("depth", t);
    if (fps > 0) {
      unsigned short v = depth.ptr<unsigned short>(depth.rows / 2)[depth.cols / 2];
      printf("fps:%d distance: %d\n", (int)fps, v);
    }
  }
  ret = port.FrameGet(percipio::CAMDATA_POINT3D, &pimage);                                 //get point cloud data if exist
  if (percipio::CAMSTATUS_SUCCESS == ret) {
    copy_buffer(&pimage, point_cloud);
    cv::imshow("pcloud", point_cloud);
    cloud_tmp = camera_mine.image2PointCloud(point_cloud, depth);
    pcl::io::savePCDFile( "./pointcloud.pcd", *cloud_tmp);
    // 清除数据并退出
    cloud_tmp->points.clear();
  }
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

void save_frame_to_file() {
  static int idx = 0;
  char buff[100];
  if (!left.empty()) {
    sprintf(buff, "%d-left.png", idx);
    cv::imwrite(buff, left);
  }
  if (!right.empty()) {
    sprintf(buff, "%d-right.png", idx);
    cv::imwrite(buff, right);
  }
  if (!depth.empty()) {
    sprintf(buff, "%d-depth.txt", idx);
    std::ofstream ofs(buff);
    ofs << depth;
    ofs.close();
  }
  if (!point_cloud.empty()) {
    sprintf(buff, "%d-points.txt", idx);
    std::ofstream ofs(buff);
    percipio::Vect3f *ptr = point_cloud.ptr<percipio::Vect3f>();
    for (int i = 0; i < point_cloud.size().area(); i++) {
      ofs << ptr->x << "," << ptr->y << "," << ptr->z << std::endl;
      ptr++;
    }
    ofs.close();
  }
  idx++;
}

void copy_buffer(percipio::ImageBuffer *pbuf, cv::Mat &img) {
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
