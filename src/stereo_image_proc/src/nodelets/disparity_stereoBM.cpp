#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_geometry/stereo_camera_model.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <stereo_image_proc/DisparityConfig.h>
#include <dynamic_reconfigure/server.h>

#include <stereo_image_proc/processor.h>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/ximgproc.hpp"
#include <string>

namespace stereo_image_proc {

using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters::sync_policies;
using namespace cv;
using namespace cv::ximgproc;

class DisparityNodelet : public nodelet::Nodelet
{
  boost::shared_ptr<image_transport::ImageTransport> it_;

  // Subscriptions
  image_transport::SubscriberFilter sub_l_image_, sub_r_image_;
  message_filters::Subscriber<CameraInfo> sub_l_info_, sub_r_info_;
  typedef ExactTime<Image, CameraInfo, Image, CameraInfo> ExactPolicy;
  typedef ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproximatePolicy;
  typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
  typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
  boost::shared_ptr<ExactSync> exact_sync_;
  boost::shared_ptr<ApproximateSync> approximate_sync_;
  // Publications
  boost::mutex connect_mutex_;
  ros::Publisher pub_disparity_;

  // Dynamic reconfigure
  // delete or add dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef stereo_image_proc::DisparityConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  // Processing state (note: only safe because we're single-threaded!)
  image_geometry::StereoCameraModel model_;
  stereo_image_proc::StereoProcessor block_matcher_; // contains scratch buffers for block matching

  virtual void onInit();

  void connectCb(); // Handles (un)subscribing when clients (un)subscribe

  void imageCb(const ImageConstPtr& l_image_msg, const CameraInfoConstPtr& l_info_msg,
               const ImageConstPtr& r_image_msg, const CameraInfoConstPtr& r_info_msg);

  //void configCb(Config &config, uint32_t level);
};

void DisparityNodelet::onInit()
{
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &private_nh = getPrivateNodeHandle();

  it_.reset(new image_transport::ImageTransport(nh));

  // Synchronize inputs. Topic subscriptions happen on demand in the connection
  // callback. Optionally do approximate synchronization.
  int queue_size;
  private_nh.param("queue_size", queue_size, 5);
  bool approx;
  private_nh.param("approximate_sync", approx, false);
  if (approx)
  {
    approximate_sync_.reset( new ApproximateSync(ApproximatePolicy(queue_size),
                                                 sub_l_image_, sub_l_info_,
                                                 sub_r_image_, sub_r_info_) );
    approximate_sync_->registerCallback(boost::bind(&DisparityNodelet::imageCb,
                                                    this, _1, _2, _3, _4));
  }
  else
  {
    exact_sync_.reset( new ExactSync(ExactPolicy(queue_size),
                                     sub_l_image_, sub_l_info_,
                                     sub_r_image_, sub_r_info_) );
    exact_sync_->registerCallback(boost::bind(&DisparityNodelet::imageCb,
                                              this, _1, _2, _3, _4));
  }

  // Set up dynamic reconfiguration
  //ReconfigureServer::CallbackType f = boost::bind(&DisparityNodelet::configCb,
                      //                            this, _1, _2);
  //reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  //reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  ros::SubscriberStatusCallback connect_cb = boost::bind(&DisparityNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_disparity_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_disparity_ = nh.advertise<DisparityImage>("disparity", 1, connect_cb, connect_cb);
}

// Handles (un)subscribing when clients (un)subscribe
void DisparityNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_disparity_.getNumSubscribers() == 0)
  {
    sub_l_image_.unsubscribe();
    sub_l_info_ .unsubscribe();
    sub_r_image_.unsubscribe();
    sub_r_info_ .unsubscribe();
  }
  else if (!sub_l_image_.getSubscriber())
  {
    ros::NodeHandle &nh = getNodeHandle();
    // Queue size 1 should be OK; the one that matters is the synchronizer queue size.
    /// @todo Allow remapping left, right?
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_l_image_.subscribe(*it_, "left/image_rect", 1, hints);
    sub_l_info_ .subscribe(nh,   "left/camera_info", 1);
    sub_r_image_.subscribe(*it_, "right/image_rect", 1, hints);
    sub_r_info_ .subscribe(nh,   "right/camera_info", 1);
  }
}

void DisparityNodelet::imageCb(const ImageConstPtr& l_image_msg,
                               const CameraInfoConstPtr& l_info_msg,
                               const ImageConstPtr& r_image_msg,
                               const CameraInfoConstPtr& r_info_msg)
{
  // Update the camera model
  model_.fromCameraInfo(l_info_msg, r_info_msg);

  // Allocate new disparity image message
  DisparityImagePtr disp_msg = boost::make_shared<DisparityImage>();
  disp_msg->header         = l_info_msg->header;
  disp_msg->image.header   = l_info_msg->header;

  // Compute window of (potentially) valid disparities
  /*int border   = block_matcher_.getCorrelationWindowSize() / 2;
  int left   = block_matcher_.getDisparityRange() + block_matcher_.getMinDisparity() + border - 1;
  int wtf = (block_matcher_.getMinDisparity() >= 0) ? border + block_matcher_.getMinDisparity() : std::max(border, -block_matcher_.getMinDisparity());
  int right  = disp_msg->image.width - 1 - wtf;
  int top    = border;
  int bottom = disp_msg->image.height - 1 - border; */

  int wsize=9;
  int min_disp = 0;
  int range_disp = 144;               //parameter of stereo matching
  bool no_downscale = true;       //force stereo matching on full-sized views to improve quality
  double vis_mult = 1.0;           //coefficient used to scale disparity map visualizations
  bool no_display = false;       // don't display results
  double lambda = 8000.0;       //8000.0       parameter of wls post-filtering
  double sigma  = 1.5;          //1.5, parameter of wls post-filtering
  String filter="wls_conf";

  // Create cv::Mat views onto all buffers
  const cv::Mat_<uint8_t> l_image = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
  const cv::Mat_<uint8_t> r_image = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)->image;

  disp_msg->valid_window.x_offset = range_disp + min_disp + wsize/2 -1;
  disp_msg->valid_window.y_offset = wsize/2;
  disp_msg->valid_window.width    = (l_image.size().width -1 - wsize/2 + min_disp) - (range_disp + min_disp + wsize/2 -1 ) ;// right - left;
  disp_msg->valid_window.height   = (l_image.size().height -1 - wsize/2) - wsize/2;//bottom - top;


  //Create cv::Mat for matching
  cv::Mat left_for_matcher, right_for_matcher;
  cv::Mat left_disp,right_disp;
  cv::Mat filtered_disp;

  Rect ROI;
  Ptr<DisparityWLSFilter> wls_filter;
  double matching_time;
  double solving_time = 0;

  if(filter=="wls_conf") // filtering with confidence (significantly better quality than wls_no_conf)
  {
        if(!no_downscale)
        {
            // downscale the views to speed-up the matching stage, as we will need to compute both left
            // and right disparity maps for confidence map computation
            range_disp/=2;
            if(range_disp%16!=0)
                range_disp += 16-(range_disp%16);
            resize(l_image ,left_for_matcher ,Size(),0.5,0.5, INTER_LINEAR_EXACT);
            resize(r_image,right_for_matcher,Size(),0.5,0.5, INTER_LINEAR_EXACT);
        }
        else
        {
            left_for_matcher  = l_image.clone();
            right_for_matcher = r_image.clone();
        }

        Ptr<StereoBM> left_matcher = StereoBM::create(range_disp,wsize);
        wls_filter = createDisparityWLSFilter(left_matcher);
        Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

        // Perform block matching to find the disparities
        matching_time = (double)getTickCount();
        left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
        right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
        matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();

        wls_filter->setLambda(lambda);
        wls_filter->setSigmaColor(sigma);
        wls_filter->filter(left_disp,l_image,filtered_disp,right_disp);

        /*ROI = wls_filter->getROI();
        if(!no_downscale)
        {
            // upscale raw disparity and ROI back for a proper comparison:
            resize(left_disp,left_disp,Size(),2.0,2.0,INTER_LINEAR_EXACT);
            left_disp = left_disp*2.0;
            ROI = Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
        }*/
  }

  //Dispaly and save images
  //cv::Mat filtered_disp_vis;
  //getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
  //imwrite("directInToMatlabFilteredDispVIS.bmp",filtered_disp_vis);
  //imwrite("right.bmp",r_image);
  //imwrite("left.bmp",l_image);
  //filtered_disp=filtered_disp_vis.clone();
  //normalize(filtered_disp_vis, filtered_disp_vis, 0, 255, NORM_MINMAX);
	//bitwise_not(filtered_disp_vis, filtered_disp_vis);
  //applyColorMap(filtered_disp_vis, filtered_disp_vis, COLORMAP_JET);
  //imwrite("directInToMatlabFilteredDispVISCOL.bmp",filtered_disp_vis);


  //http://docs.ros.org/diamondback/api/stereo_image_proc/html/processor_8cpp_source.html
  // Fixed-point disparity is 16 times the true value: d = d_fp / 16.0 = x_l - x_r.
  static const int DPP = 16; // disparities per pixel
  static const double inv_dpp = 1.0 / DPP;

  //Filtered_disp is CV_16S
  sensor_msgs::Image& dimage = disp_msg->image;
  dimage.height = filtered_disp.rows;
  dimage.width = filtered_disp.cols;
  dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  dimage.step = dimage.width * sizeof(float);
  dimage.data.resize(dimage.step * dimage.height);
  cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);
   // We convert from fixed-point to float disparity and also adjust for any x-offset between
  // the principal points: d = d_fp*inv_dpp - (cx_l - cx_r)
  filtered_disp.convertTo(dmat, dmat.type(), inv_dpp, -(model_.left().cx() - model_.right().cx()));
  ROS_ASSERT(dmat.data == &dimage.data[0]);
   // Stereo parameters
  disp_msg->f = model_.right().fx();
  disp_msg->T = model_.baseline();
  // Disparity search range
  disp_msg->min_disparity = min_disp;
  disp_msg->max_disparity = min_disp + range_disp - 1;
  disp_msg->delta_d = inv_dpp;

  // Adjust for any x-offset between the principal points: d' = d - (cx_l - cx_r)
  double cx_l = model_.left().cx();
  double cx_r = model_.right().cx();
  if (cx_l != cx_r) {
    cv::Mat_<float> disp_image(disp_msg->image.height, disp_msg->image.width,
                              reinterpret_cast<float*>(&disp_msg->image.data[0]),
                              disp_msg->image.step);
    cv::subtract(disp_image, cv::Scalar(cx_l - cx_r), disp_image);
  }

  pub_disparity_.publish(disp_msg);
}

/* trine lage funksjon istede
void processDisparityStereoBM(const cv::Mat& left_rect, const cv::Mat& right_rect,
                      stereo_msgs::DisparityImage& disparity) const
{

}*/
/*
void DisparityNodelet::configCb(Config &config, uint32_t level)
{
  // Tweak all settings to be valid
  config.prefilter_size |= 0x1; // must be odd
  config.correlation_window_size |= 0x1; // must be odd
  config.disparity_range = (config.disparity_range / 16) * 16; // must be multiple of 16

  // check stereo method
  // Note: With single-threaded NodeHandle, configCb and imageCb can't be called
  // concurrently, so this is thread-safe.
  block_matcher_.setPreFilterCap(config.prefilter_cap);
  block_matcher_.setCorrelationWindowSize(config.correlation_window_size);
  block_matcher_.setMinDisparity(config.min_disparity);
  block_matcher_.setDisparityRange(config.disparity_range);
  block_matcher_.setUniquenessRatio(config.uniqueness_ratio);
  block_matcher_.setSpeckleSize(config.speckle_size);
  block_matcher_.setSpeckleRange(config.speckle_range);
  if (config.stereo_algorithm == stereo_image_proc::Disparity_StereoBM) { // StereoBM
    block_matcher_.setStereoType(StereoProcessor::BM);
    block_matcher_.setPreFilterSize(config.prefilter_size);
    block_matcher_.setTextureThreshold(config.texture_threshold);
  }
  else if (config.stereo_algorithm == stereo_image_proc::Disparity_StereoSGBM) { // StereoSGBM
    block_matcher_.setStereoType(StereoProcessor::SGBM);
    block_matcher_.setSgbmMode(config.fullDP);
    block_matcher_.setP1(config.P1);
    block_matcher_.setP2(config.P2);
    block_matcher_.setDisp12MaxDiff(config.disp12MaxDiff);
  }
*/

} // namespace stereo_image_proc

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(stereo_image_proc::DisparityNodelet,nodelet::Nodelet)
