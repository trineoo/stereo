#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <iostream>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/ximgproc.hpp"

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include <string>

#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/stereo_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Point.h>
#include <custom_msgs/DetectedObjects.h>
#include <custom_msgs/Object.h>
#include <custom_msgs/NorthEastHeading.h>
#include <custom_msgs/gnssGGA.h>

#include <custom_libraries/convert_to_NED.hpp>

#include <fstream>

char path1[70] = "/home/stereo/STEREO_SYSTEM/GPS/clusteringCNNDepthNED.csv";

double heading_MA;



using namespace message_filters;

ros::Publisher detected_objects_pub;



float medianMat (cv::Mat image){
  double m=(image.rows*image.cols)/2;
  int bin=0;
  int med = -1;
  int histSize = 256;
  float range[] = { 0, 256 } ;
  const float* histRange = { range };
  bool uniform = true;
  bool accumulate = false;
  cv::Mat hist;
  cv::calcHist( &image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );

    for (int i=0; i<256 && ( med<0);i++){
        bin=bin+cvRound(hist.at<float>(i));
        if (bin>m && med<0)
            med=i;
    }

    return med;
}

double constrainAngle(double x){
    x = x - 90;
    x = fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}


void headingCallback(const custom_msgs::NorthEastHeading::ConstPtr& heading){
  heading_MA = heading->heading;

}


void callback(const stereo_msgs::DisparityImage::ConstPtr& disp, const darknet_ros_msgs::BoundingBoxes::ConstPtr& objects, const sensor_msgs::CameraInfo::ConstPtr& l_cam_info, const sensor_msgs::CameraInfo::ConstPtr& r_cam_info, const custom_msgs::gnssGGA::ConstPtr& pos){
    //std::ofstream myfile;
    //myfile.open(path1, std::ios::out | std::ios::app);
    custom_msgs::DetectedObjects detected_objects;

    // NED frame Piren
    double lat0_deg  = 63.4389029083;
    double long0_deg = 10.39908278;
    double altitude0 = 39.923;
    std::vector<double> position_MA = lla2ned(pos->latitude, pos->longitude, pos->altitude, lat0_deg, long0_deg, altitude0);

    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(l_cam_info, r_cam_info);

    const sensor_msgs::Image& dimage = disp->image;
    cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
    cv::Mat disp_u(dmat.size(), 0);
    dmat.convertTo(disp_u, disp_u.type());
    cv::normalize(disp_u, disp_u, 0, 255, cv::NORM_MINMAX );
    applyColorMap(disp_u, disp_u, cv::COLORMAP_JET);


    for(int i = 0; i < objects->bounding_boxes.size() ; i++){

      int width = objects->bounding_boxes[i].xmax - objects->bounding_boxes[i].xmin;
      int height = objects->bounding_boxes[i].ymax - objects->bounding_boxes[i].ymin;
      cv::Mat roi(dmat, cv::Rect(objects->bounding_boxes[i].xmin, objects->bounding_boxes[i].ymin, width, height));

      float med = medianMat(roi);

      float middle_x = objects->bounding_boxes[i].xmin + (width/2);
      float middle_y = objects->bounding_boxes[i].ymin + (height/2);
      cv::Point2d point_uv(middle_x, middle_y);
      cv::Point3d point_xyz;

      model.projectDisparityTo3d(point_uv, med, point_xyz);

      geometry_msgs::Point coord;
      coord.x = point_xyz.x/1000;
      coord.y = point_xyz.y/1000;
      coord.z = point_xyz.z/1000;

      cv::Point3d xmin_xyz, xmax_xyz;
      model.projectDisparityTo3d(cv::Point2d(objects->bounding_boxes[i].xmin, objects->bounding_boxes[i].ymin), dmat.at<float>(objects->bounding_boxes[i].ymin, objects->bounding_boxes[i].xmin),xmin_xyz);
      model.projectDisparityTo3d(cv::Point2d(objects->bounding_boxes[i].xmax, objects->bounding_boxes[i].ymin), dmat.at<float>(objects->bounding_boxes[i].ymin, objects->bounding_boxes[i].xmax),xmax_xyz);

      float obj_width = (xmax_xyz.x/1000) - (xmin_xyz.x/1000);

      std::vector<double> obj_NED = objectCoord2NED(coord.x, coord.y, coord.z, position_MA[0], position_MA[1], position_MA[2], heading_MA);

      custom_msgs::Object object;
      object.north = obj_NED[0];
      object.east  = obj_NED[1];
      object.down  = obj_NED[2];
      object.width = obj_width;
      object.Class = objects->bounding_boxes[i].Class;


      //float depth = sqrt(pow(obj_NED[0]-position_MA[0],2)+pow(obj_NED[1]-position_MA[1],2));
      //float depth1 = sqrt(pow(coord.x,2)+pow(coord.z,2));
      //myfile << objects->header.stamp.sec <<","<<coord_NED.x<<","<<coord_NED.y <<","<<heading_MA<<std::endl;
      //myfile << objects->header.stamp.sec <<","<<depth<<","<<depth1 <<std::endl;
      detected_objects.objects.push_back(object);

    }
    detected_objects.header = objects->header;
    detected_objects_pub.publish(detected_objects);
  //  myfile.close();
}



int main(int argc, char** argv){
  ros::init(argc, argv, "clustering_cnn");
  ros::NodeHandle nh;

  heading_MA = 0;

  //std::ofstream myfile;
  //myfile.open(path1, std::ios::out);
  //myfile <<"time"<<","<< "depth" <<std::endl;


  detected_objects_pub = nh.advertise<custom_msgs::DetectedObjects>("detected_objects", 10);

  message_filters::Subscriber<sensor_msgs::CameraInfo> l_camInfo_sub(nh, "/camera_array/left/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> r_camInfo_sub(nh, "/camera_array/right/camera_info", 1);
  message_filters::Subscriber<stereo_msgs::DisparityImage> disp_sub(nh, "/camera_array/disparity", 100);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> objects_sub(nh, "/darknet_ros/bounding_boxes", 1);
  message_filters::Subscriber<custom_msgs::gnssGGA> position_sub(nh, "/vectorVS330/fix", 1);
  //TimeSynchronizer<stereo_msgs::DisparityImage,darknet_ros_msgs::BoundingBoxes, sensor_msgs::CameraInfo,sensor_msgs::CameraInfo, custom_msgs::gnssGGA> sync(disp_sub, objects_sub, l_camInfo_sub, r_camInfo_sub, position_sub, 10);
  typedef sync_policies::ApproximateTime<stereo_msgs::DisparityImage,darknet_ros_msgs::BoundingBoxes, sensor_msgs::CameraInfo,sensor_msgs::CameraInfo, custom_msgs::gnssGGA> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), disp_sub, objects_sub, l_camInfo_sub, r_camInfo_sub, position_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

  ros::Subscriber heading_sub = nh.subscribe("/navigation/eta", 1000, headingCallback);

  ros::spin();
  return 0;

}
