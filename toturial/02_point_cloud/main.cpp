
#include <percipio_camport.h>
#include <opencv2/opencv.hpp>
#include "PCLRenderer.h"
#include "Profiler.h"

int main(int argc, char** argv)
{
   int ret;
   percipio::DepthCameraDevice device;
   PCLRenderer render;

   //Open device
   ret = device.OpenDevice();
   if (ret != percipio::CAMSTATUS_SUCCESS) {
      printf("open device failed\n");
      return -1;
   }

   //Set work-mode to DEPTH
   ret = device.SetProperty_Int(percipio::PROP_WORKMODE, percipio::WORKMODE_DEPTH);
   if (ret < 0) {
       printf("set work-mode failed,error code:%d\n", ret);
       return -1;
   }
   //Enable point cloud output
   ret = device.SetPointCloudOutput(true);
   if (ret < 0) {
       printf("SetPointCloudOutput failed,error code:%d\n", ret);
       return -1;
   }

   /*
    * Loop to get depth data and display it
    * 'q' - quit
    */
   while (true) {
      //'q' to exit
      int k = cv::waitKey(1);
      if (k == 'q' || k == 1048689)
         break;

      //get package
      ret = device.FramePackageGet();
      if (ret != percipio::CAMSTATUS_SUCCESS) {
         printf("failed in FramePackageGet()\n");
         continue;
      }

      //get depth image
      percipio::ImageBuffer image;
      ret = device.FrameGet(percipio::CAMDATA_POINT3D, &image);
      if (ret != percipio::CAMSTATUS_SUCCESS) {
         printf("failed in FrameGet()\n");
         continue;
      }

      //fps
      Profiler::fps("Point cloud");

      //display
      render.show(image.width, image.height, image.get_pixel_size(), image.data);
   }

   //Close device
   device.CloseDevice();
   return 0;
}
