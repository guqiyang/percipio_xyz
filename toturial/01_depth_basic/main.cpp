#include <percipio_camport.h>
#include <opencv2/opencv.hpp>
#include "depth_render.h"

int main(int argc, char** argv)
{
  int ret;
  percipio::DepthCameraDevice device;
  DepthRender render;

  //Open device
  ret = device.OpenDevice();
  if (ret != percipio::CAMSTATUS_SUCCESS) {
    printf("open device failed\n");
    return -1;
       
  }

  //Init render for displaying depth image
  render.range_mode = DepthRender::COLOR_RANGE_DYNAMIC;
  render.color_type = DepthRender::COLORTYPE_BLUERED;
  render.invalid_label = 0;
  render.Init();

  //Set work-mode to DEPTH
  ret = device.SetProperty_Int(percipio::PROP_WORKMODE, percipio::WORKMODE_DEPTH);
  if (ret < 0) {
    printf("set work-mode failed,error code:%d\n", ret);
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
    ret = device.FrameGet(percipio::CAMDATA_DEPTH, &image);
    if (ret != percipio::CAMSTATUS_SUCCESS) {
      printf("failed in FrameGet()\n");
      continue;
            
    }

    //convert the 16-bit gray depth to color type
    cv::Mat depth;
    cv::Mat t(image.height, image.width, CV_16UC1, image.data);
    render.Compute(t, depth);
    cv::imshow("Percipio Depth", depth);
       
  }

  //Close device
  device.CloseDevice();
  render.Uninit();

  return 0;

}
