// camport_test.cpp : Defines the entry point for the console application.
//

#define _CRT_SECURE_NO_WARNINGS

#ifdef _WIN32
#include <time.h>
#include <windows.h>
#define MSLEEP(n)    Sleep(n)
#else
#include <sys/time.h>
#include <unistd.h>
#define MSLEEP(n)    usleep(n*1000)
#endif

#include <sstream>
#include "percipio_camport.h"
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include "depth_render.h"

#ifdef HAVE_PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#endif

/// For easier use
class CamIntristicWrapper {
private:
  float scalex, scaley;           /**< sensor downsampling scale factor */
  float inv_fx, x0,     cx;       /**< mat row 1 */
  float x1,     inv_fy, cy;       /**< mat row 2 */
  float x2,     x3,     x4;       /**< mat row 3 */
  float scaleSize;                /**< calculated */

public:
  CamIntristicWrapper(const percipio::CamIntristic &_i, const int imageW, const int imageH)
    : scalex((float)_i.width / imageW),
      scaley((float)_i.height / imageH),
      inv_fx(1.f / _i.data[0]), x0(_i.data[1]),             cx(_i.data[2]),
      x1(_i.data[3]),           inv_fy(1.f / _i.data[4]),   cy(_i.data[5]),
      x2(_i.data[6]),           x3(_i.data[7]),             x4(_i.data[8]),
      scaleSize(scalex * scaley)
  {
        
  }

  ~CamIntristicWrapper()
  {
        
  }

  cv::Point3f depthToWorld(const cv::Point3f &depth)
  {
    cv::Point3f world;
    world.x = (depth.x * this->scalex - this->cx) * depth.z * this->inv_fx;
    world.y = (depth.y * this->scaley - this->cy) * depth.z * this->inv_fy;
    world.z = depth.z;

    return world;
        
  }

  int depthImgToPointCloud(const cv::Mat &depthImg, cv::Mat &pointCloud, const cv::Rect &roi = cv::Rect())
  {
    if(depthImg.type() != CV_16UC1 && depthImg.type() != CV_32FC1){
      printf("depthImgToPointCloud wrong type of depthImg, have to be CV_16U or CV_32F.\n");
      return -1;
              
    }

    cv::Rect rect(roi);
    if(roi == cv::Rect()){
      rect = cv::Rect(0, 0, depthImg.cols, depthImg.rows);
              
    }

    pointCloud.create(rect.height, rect.width, CV_32FC3);
    int k = 0;
    cv::Point3f* dst = (cv::Point3f*)pointCloud.data;
    if(depthImg.type() == CV_16UC1){
      for(int r = 0; r < rect.height; r++){
        uint16_t*    src = (uint16_t*)depthImg.ptr(r+rect.y, rect.x);
        for(int c = 0; c < rect.width; c++){
          dst[k] = depthToWorld(cv::Point3f(c+rect.x, r+rect.y, src[c]));
          k++;
                          
        }
                    
      }
              
    }else if(depthImg.type() == CV_32FC1){
      for(int r = 0; r < rect.height; r++){
        float*       src = (float*)depthImg.ptr(r+rect.y, rect.x);
        for(int c = 0; c < rect.width; c++){
          dst[k] = depthToWorld(cv::Point3f(c+rect.x, r+rect.y, src[c]));
          k++;
                          
        }
                    
      }
              
    }
    return 0;
        
  }


};

#ifdef HAVE_PCL
int pcshow(const cv::Mat &pointCloud, const std::string &windowName)
{
  if(!(pointCloud.type() == CV_32FC3)){
    printf("pcshow: pointCloud should be (type=CV_32FC3)\n");
    return -1;
        
  }

  static std::map<std::string, pcl::visualization::CloudViewer*> gViewerMap;

  std::map<std::string, pcl::visualization::CloudViewer*>::iterator it = gViewerMap.find(windowName);

  if(gViewerMap.end() == it)
    {
      pcl::visualization::CloudViewer* viewer = new pcl::visualization::CloudViewer(windowName);
      std::pair<std::map<std::string, pcl::visualization::CloudViewer*>::iterator, bool> ret;
      ret = gViewerMap.insert(std::pair<std::string, pcl::visualization::CloudViewer*>(windowName, viewer));
      if(!ret.second){
        printf("pcshow: insert viewer %s failed.\n", windowName.c_str());
        return -1;
                
      }
      it = ret.first;
          
    }

  // PCL display
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->resize(0);
  for(int i = 0; i < pointCloud.rows * pointCloud.cols; i++){
    float* data = (float*)pointCloud.data;
    cloud->push_back(pcl::PointXYZ(data[i*3 + 0], data[i*3 + 1], data[i*3 + 2]));
        
  }

  it->second->showCloud(cloud);

  return 0;

}
#endif

static volatile bool exitmain = false;

void CopyBuffer(percipio::ImageBuffer *pbuf, cv::Mat &img);

static inline std::string toString(int i){
  std::ostringstream oss;
  oss << i;
  return oss.str();

}

struct dev_context {
  percipio::DepthCameraDevice* port;
  DepthRender*                 render;
  CamIntristicWrapper*         ciw;
  std::string                  devID;
  int                          frameIndex;

};

void frame_arrived_callback(void *user_data) {
  // call port.FramePackageGet to update internal buffer
  // call port.FrameGet to get frame data here
  // To avoid performance problem ,time consuming task in callback function is not recommended.
  if(exitmain) return;

  dev_context* p = (dev_context*)user_data;
  if(!p->port){
    printf("p[%p] port is NULL\n", p);
    return;
      
  }

  if (p->port->FramePackageGet() == percipio::CAMSTATUS_SUCCESS)
    {
      percipio::ImageBuffer pimage;
      int ret = p->port->FrameGet(percipio::CAMDATA_DEPTH, &pimage);
      if (percipio::CAMSTATUS_SUCCESS == ret)
        {
          cv::Mat _depth, t;
          CopyBuffer(&pimage, _depth);
          p->render->Compute(_depth, t);
          cv::imshow(p->devID+"_depth", t);
          int k = cv::waitKey(1);
          if (k == 'q' || k == 1048689)
            {
              exitmain = true;
                    
            }
          printf("frame %d center distance: %d\n", p->frameIndex++, _depth.ptr<unsigned short>(_depth.rows / 2)[_depth.cols / 2]);

          // ----------- show point cloud -----------------
          cv::Mat pointCloud;
          if(p->ciw->depthImgToPointCloud(_depth, pointCloud))
            // if(p->ciw->depthImgToPointCloud(_depth, pointCloud, cv::Rect(0, 200, _depth.cols, 10)))
            {
              printf("pcshow: Convert depth image to point cloud failed.\n");
              return;
                    
            }
          #ifdef HAVE_PCL
          pcshow(pointCloud, "PointCloud");
          #endif
          // ----------------------------------------------
              
        }
        
    }

}

int main(int argc, char** argv)
{
  //--- get system info
  percipio::SetLogLevel(percipio::LOG_LEVEL_INFO);
  int ver = percipio::LibVersion();
  printf("Sdk version is %d\n", ver);

  percipio::DepthCameraDevice port0(percipio::MODEL_DPB04GN);

  //--- get devices info
  int dev_num = port0.GetDeviceNum();
  if(dev_num == 0){
    printf("No device connected.\n");
    return -1;
      
  } else if (dev_num > 10) {
    printf("Not support devices more than 10.\n");
    return -1;
      
  } else if (dev_num > 1) {
    printf("NOTE: this test only use one device!\n");
      
  }

  int dev_s[10];
  if(port0.GetDeviceList(dev_s)){
    printf("failed to get dev lists.\n");
    return -1;
      
  }

  DepthRender render;
  render.range_mode = DepthRender::COLOR_RANGE_DYNAMIC;
  render.color_type = DepthRender::COLORTYPE_BLUERED;
  render.invalid_label = 0;
  render.Init();

  dev_context c;
  c.port = &port0;
  c.render = &render;
  c.devID = toString(0);
  c.frameIndex = 0;

  c.port->SetCallbackUserData(&c);
  c.port->SetFrameReadyCallback(frame_arrived_callback);

  int ret = c.port->OpenDevice();
  if (percipio::CAMSTATUS_SUCCESS != ret) {
    printf("open device failed\n");
    return -1;
      
  }

  //------------------------------------------------------
  //--- get cam intristic after OpenDevice()
  //------------------------------------------------------
  percipio::CamIntristic intristic;
  ret = c.port->GetProperty(percipio::PROP_CALIB_DEPTH_INTRISTIC, &intristic, sizeof(intristic));
  if(ret != sizeof(intristic)){
    printf("get camera intristic param failed. error code:%d\n", ret);
    return -1;
      
  }
  CamIntristicWrapper ciw(intristic, 640, 480);
  c.ciw = &ciw;

  //--- start devices
  ret = c.port->SetProperty_Int(percipio::PROP_WORKMODE, percipio::WORKMODE_DEPTH);
  //ret = port0.SetProperty_Int(percipio::PROP_WORKMODE, percipio::WORKMODE_IR);
  //ret = port0.SetProperty_Int(percipio::PROP_WORKMODE, percipio::WORKMODE_IR_DEPTH);
  if (ret < 0) {
    printf("set mode failed,error code:%d\n", ret);
    return -1;
      
  }

  //display a empty window for receiving key input
  // cv::Mat left;
  // cv::imshow("left", cv::Mat::zeros(100, 100, CV_8UC1));

  while (!exitmain) {
    MSLEEP(100);
      
  }

  c.port->CloseDevice();

  return 0;

}

void CopyBuffer(percipio::ImageBuffer *pbuf, cv::Mat &img) {
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
 
