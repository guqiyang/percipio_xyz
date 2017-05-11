/*
 * PCLRenderer.h
 *
 *  Created on: Sep 10, 2016
 *      Author: qxji
 */

#ifndef _PCLRENDERER_H_
#define _PCLRENDERER_H_

#include <pcl/visualization/cloud_viewer.h>

class PCLRenderer {
 public:
  PCLRenderer();
  virtual ~PCLRenderer();

  void show(int width, int height, int bpp, const unsigned char* data);

 private:
  pcl::visualization::CloudViewer* m_pPCLViewer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_pPointCloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pColorPointCloud;

  static void viewerOneOff(pcl::visualization::PCLVisualizer& viewer);
  void render();

};

#endif /* _PCLRENDERER_H_ */
