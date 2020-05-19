

#include <ros/ros.h>
#include <iostream>
#include <math.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

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


void callback(const stereo_msgs::DisparityImage::ConstPtr& disp, const darknet_ros_msgs::BoundingBoxes::ConstPtr& objects, const sensor_msgs::CameraInfo::ConstPtr& l_cam_info, const sensor_msgs::CameraInfo::ConstPtr& r_cam_info){
    custom_msgs::DetectedObjects detected_objects;

    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(l_cam_info, r_cam_info);

    const sensor_msgs::Image& dimage = disp->image;
    const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);

    for(int i = 0; i < objects->bounding_boxes.size() ; i++){

      int width = objects->bounding_boxes[i].xmax - objects->bounding_boxes[i].xmin;
      int height = objects->bounding_boxes[i].ymax - objects->bounding_boxes[i].ymin;
      cv::Mat roi(dmat, cv::Rect(objects->bounding_boxes[i].xmin, objects->bounding_boxes[i].ymin, width, height));

      float med = medianMat(roi);
      //float depth = (disp->f * disp->T)/med;


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

      custom_msgs::Object object;
      object.coord = coord;
      object.width = obj_width;
      object.Class = objects->bounding_boxes[i].Class;

      detected_objects.objects.push_back(object);
    }
    detected_objects.header = objects->header;
    detected_objects_pub.publish(detected_objects);

}

/*
void callback(const stereo_msgs::DisparityImage::ConstPtr& disp, const darknet_ros_msgs::BoundingBoxes::ConstPtr& objects){
    //BoundingBox[] boxes = objects->bounding_boxes;
    const sensor_msgs::Image& dimage = disp->image;
    const cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
    //cv::Mat disparityMap;
    //cv::resize(dmat, disparityMap, cv::Size(608,608));

    //imwrite("disp22.bmp", dmat);
    //std::cout<<"height: "<<

    //cv::Mat image = cv::Mat::zeros(608, 608, CV_32F);
    cv::Mat image = cv::Mat::zeros(disp->image.height, disp->image.width, CV_32F);

    //const cv::Mat_<uint8_t> disparityMap = cv_bridge::toCvShare(disp, sensor_msgs::image_encodings::MONO8)->image;


    for(int i = 0; i < objects->bounding_boxes.size() ; i++){
      std::cout<<"xmin: "<< objects->bounding_boxes[i].xmin <<" xmax: "<< objects->bounding_boxes[i].xmax <<std::endl;
      std::cout<<"ymin: "<< objects->bounding_boxes[i].ymin <<" ymax: "<< objects->bounding_boxes[i].ymax <<std::endl;

      for (int r = objects->bounding_boxes[i].xmin ; r < objects->bounding_boxes[i].xmax ; r++ ){
        for (int c = objects->bounding_boxes[i].ymin ; c < objects->bounding_boxes[i].ymax ; c++ ){
          image.at<float>(c,r) = 200;
        }
      }
    }
    imwrite("disp.bmp", image);
}*/


int main(int argc, char** argv){
  ros::init(argc, argv, "clustering_NN");
  ros::NodeHandle nh;

  detected_objects_pub = nh.advertise<custom_msgs::DetectedObjects>("detected_objects", 10);

  //ros::Subscriber sub_objects = nh.subscribe("/darknet_ros/bounding_boxes", 1, objectsCallback);
  //ros::Subscriber sub_disp = nh.subscribe("/camera_array/disparity", 1, dispCallback);
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_camInfo_sub(nh, "/camera_array/left/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> r_camInfo_sub(nh, "/camera_array/right/camera_info", 1);
  message_filters::Subscriber<stereo_msgs::DisparityImage> disp_sub(nh, "/camera_array/disparity", 100);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> objects_sub(nh, "/darknet_ros/bounding_boxes", 1);
  TimeSynchronizer<stereo_msgs::DisparityImage,darknet_ros_msgs::BoundingBoxes, sensor_msgs::CameraInfo,sensor_msgs::CameraInfo> sync(disp_sub, objects_sub, l_camInfo_sub, r_camInfo_sub, 10);
  //typedef sync_policies::ApproximateTime<stereo_msgs::DisparityImage,darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
  //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), disp_sub, objects_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));




  //sync.registerCallback(mask_detect_callback)

  ros::spin();
  return 0;

}
