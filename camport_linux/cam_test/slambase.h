//standard
#include<fstream>
#include<vector>

//opencv
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

//PCL
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>

//typedef
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class CameraCompute{

 private:
  double cx,cy,fx,fy,scale;
 public:
  CameraCompute(){
    cx=cy=fx=fy=scale=0.0;
    setcameracalib();
  }
  //from image to point cloud
  PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth);
  //input: (u,v,d)
  cv::Point3f point2dTo3d(cv::Point3f& point);
  void setcameracalib();
};
