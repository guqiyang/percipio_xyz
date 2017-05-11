/*************************************************************************
    > File Name: src/slamBase.cpp
    > Author: xiang gao
    > Mail: gaoxiang12@mails.tsinghua.edu.cn
    > Implementation of slamBase.h
    > Created Time: 2015年07月18日 星期六 15时31分49秒
 ************************************************************************/

#include "slambase.h"

const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

PointCloud::Ptr CameraCompute::image2PointCloud( cv::Mat& rgb, cv::Mat& depth )
{
    PointCloud::Ptr cloud ( new PointCloud );

    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / scale;
            p.x = (n - cx) * p.z / fx;
            p.y = (m - cy) * p.z / fy;
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

cv::Point3f CameraCompute::point2dTo3d( cv::Point3f& point)
{
    cv::Point3f p; // 3D 点
    p.z = double( point.z ) / scale;
    p.x = ( point.x - cx) * p.z /fx;
    p.y = ( point.y - cy) * p.z /fy;
    return p;
}

void CameraCompute::setcameracalib(){
  cx = camera_cx;
  cy = camera_cy;
  fx = camera_fx;
  fy = camera_fy;
  scale = camera_factor;
}
