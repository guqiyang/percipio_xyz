/*
 * PCLRenderer.cpp
 *
 *  Created on: Sep 10, 2016
 *      Author: qxji
 */

#include "PCLRenderer.h"

PCLRenderer::PCLRenderer() :
   m_pPCLViewer(NULL),
   m_pPointCloud(new pcl::PointCloud<pcl::PointXYZ>),
   m_pColorPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>)
{

}

PCLRenderer::~PCLRenderer() {
   delete m_pPCLViewer;
}

//update point cloud by client
void PCLRenderer::show(int width, int height, int bpp, const unsigned char* data)
{
    float* points = (float*)data;
    size_t image_size = width*height;
   const char* win_name = "Percipio PointCloud";
   static bool bFirst = false;

   if (!m_pPCLViewer) {
      m_pPCLViewer = new pcl::visualization::CloudViewer(win_name);
      bFirst = true;
   }

    m_pPointCloud->resize(0);
    for(size_t i=0; i<image_size; ++i)
    {
       m_pPointCloud->push_back(pcl::PointXYZ(points[0], points[1], points[2]));
        points += 3;
    }

    m_pPCLViewer->showCloud(m_pPointCloud, win_name);

    if (bFirst) {
       m_pPCLViewer->runOnVisualizationThreadOnce(viewerOneOff);
       bFirst = false;
    }
}

void PCLRenderer::viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
    viewer.resetCamera();
    //viewer.addCoordinateSystem(1.0);
}
