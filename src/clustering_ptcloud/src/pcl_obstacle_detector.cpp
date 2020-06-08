#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <clustering_ptcloud/pclseg_reConfig.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

#include <custom_msgs/DetectedObjects.h>
#include <custom_msgs/Object.h>
#include <custom_msgs/NorthEastHeading.h>
#include <custom_msgs/gnssGGA.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <custom_libraries/convert_to_NED.hpp>

using namespace message_filters;

char path1[80] = "/home/stereo/STEREO_SYSTEM/Experiment/clusteringPtCloud_depths/bag333.csv";


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::CentroidPoint<pcl::PointXYZ> CentroidPoint;

double heading_MA;
ros::Publisher detected_objects_pub;

class PclObstacleDetector {
public:
    PclObstacleDetector(const ros::NodeHandle &nh);
    void cloudcb(const PointCloud::ConstPtr &msg); //endre msg
private:
    // --- Parameters ---

    /*** Crop ***/
    double crop_min_z;
    double crop_max_z;

    /*** sor: Statistical Outlier Removal ***/
    int meanK; /* The number of nearest neighbors to use for mean distance estimation. */
    double stddevMul; /* The standard deviation multiplier for the distance threshold calculation. */

    /*** vg: Voxel Grid, leaf size ***/
    float lx;
    float ly;
    float lz;

    /*** Accumulator ***/
    int buffer_size;

    /*** Cluster ***/
    double cluster_tolerance; /* The spatial cluster tolerance as a measure in the L2 Euclidean space */
    int cluster_min_points;    /* The minimum number of points that a cluster needs to contain in order to be considered valid */
    int cluster_max_points;    /* The maximum number of points that a cluster needs to contain in order to be considered valid*/


    // --- PointCloud Manipulators ---
    PointCloud::ConstPtr cut_cloud(const PointCloud::ConstPtr &cloud_input);
    PointCloud::ConstPtr sor_cloud(const PointCloud::ConstPtr &cloud_input);
    PointCloud::ConstPtr vg_cloud(const PointCloud::ConstPtr &cloud_input);
    PointCloud::Ptr accumulate_cloud(const PointCloud::ConstPtr &cloud_input);
    PointCloud::Ptr cluster_cloud(const PointCloud::ConstPtr &cloud_input);

    void configCallback(clustering_ptcloud::pclseg_reConfig &config, uint32_t level);
    dynamic_reconfigure::Server<clustering_ptcloud::pclseg_reConfig> dr_srv_;

    std::vector<PointCloud::Ptr> pcl_vector;

    // ROS
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub_cut;
    ros::Publisher pub_sor;
    ros::Publisher pub_vg;
    ros::Publisher pub_acc;
    ros::Publisher pub_cluster;
};


/***
** PassThrough filter
** simple filtering along a specified dimension
** cut off values that are either inside or outside a given user range.
***/
PointCloud::ConstPtr PclObstacleDetector::cut_cloud(const PointCloud::ConstPtr &cloud_input){
    PointCloud::Ptr cloud_filtered (new PointCloud);
    cloud_filtered->header = cloud_input->header;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_input);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (crop_min_z, crop_max_z);
    pass.filter (*cloud_filtered);

    pub_cut.publish(cloud_filtered);
    const PointCloud::ConstPtr cloud_filtered_const = cloud_filtered;
    return cloud_filtered_const;
}

/***
** Statistical Outlier Removal
** Remove noisy measurements, e.g. outliers,
** from a point cloud dataset using statistical analysis techniques.
***/
PointCloud::ConstPtr PclObstacleDetector::sor_cloud(const PointCloud::ConstPtr &cloud_input){
    PointCloud::Ptr cloud_filtered (new PointCloud);
    cloud_filtered->header = cloud_input->header;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_input);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (stddevMul);
    sor.filter (*cloud_filtered);

    pub_sor.publish(cloud_filtered);
    const PointCloud::ConstPtr cloud_filtered_const = cloud_filtered;
    return cloud_filtered_const;

}

/***
** Voxel Grid
** Downsample the point cloud using a voxelized grid approach
***/
PointCloud::ConstPtr PclObstacleDetector::vg_cloud(const PointCloud::ConstPtr &cloud_input){
    PointCloud::Ptr cloud_filtered (new PointCloud);
    cloud_filtered->header = cloud_input->header;

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_input);
    sor.setLeafSize (lx, ly, lz);
    sor.filter (*cloud_filtered);

    pub_vg.publish(cloud_filtered);
    const PointCloud::ConstPtr cloud_filtered_const = cloud_filtered;
    return cloud_filtered_const;

}

/***
** Accumulate  point clouds
** For increasing the 3D information, forming a point cloud
** representative of the the perceived environment
***/
PointCloud::Ptr PclObstacleDetector::accumulate_cloud(const PointCloud::ConstPtr &cloud_input){
    PointCloud::Ptr new_cloud(new PointCloud);
    (*new_cloud).header = cloud_input->header;
    while (pcl_vector.size() >= buffer_size) {
      pcl_vector.pop_back();
    }

    for (int i = 0; i < (*cloud_input).size(); i++)
      (*new_cloud).push_back(cloud_input->points[i]);

    pcl_vector.insert(pcl_vector.begin(), new_cloud);

    if (!pcl_vector.empty()) {
        PointCloud::Ptr merged_cloud(new PointCloud);
        merged_cloud->header = cloud_input->header;
        for (auto& cloud : pcl_vector) {
            *merged_cloud += *cloud;
        }

        merged_cloud->header.frame_id = "/velodyne";
        pub_acc.publish(merged_cloud);
        return merged_cloud;
    }else{
        ROS_WARN("Returned NULL PointCloud in Accumulate CB");
        return NULL;
    }
}


/***
** Euclidean Cluster Extraction
** represents a segmentation class for cluster extraction
** in an Euclidean sense, depending on pcl::gpu::octree
***/
PointCloud::Ptr PclObstacleDetector::cluster_cloud(const PointCloud::ConstPtr &cloud_input){

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_input);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setClusterTolerance(this->cluster_tolerance);
    euclid.setMinClusterSize(this->cluster_min_points);
    euclid.setMaxClusterSize(this->cluster_max_points);
    euclid.setSearchMethod(tree);
    euclid.setInputCloud(cloud_input);

    std::vector<pcl::PointIndices> indices;
    euclid.extract(indices);

    PointCloud::Ptr centroid_cloud(new PointCloud);
    centroid_cloud->header = cloud_input->header;
    for (auto &i: indices) {
      CentroidPoint centroid;
      for (auto &j: i.indices) {
        centroid.add(cloud_input->points[j]);
      }
      pcl::PointXYZ p;
      centroid.get(p);
      centroid_cloud->points.push_back(p);
    }
    pub_cluster.publish(centroid_cloud);
    return centroid_cloud;
}


void PclObstacleDetector::cloudcb(const PointCloud::ConstPtr &cloud_input)
{
    const PointCloud::ConstPtr &cloud_cut = cut_cloud(cloud_input);
    const PointCloud::ConstPtr &cloud_sor = sor_cloud(cloud_cut);
    const PointCloud::ConstPtr &cloud_vg = vg_cloud(cloud_sor);
    const PointCloud::ConstPtr &cloud_acc = accumulate_cloud(cloud_vg);
    const PointCloud::Ptr &cloud_clustered = cluster_cloud(cloud_sor);

    pub_cut.publish(cloud_cut);
    pub_sor.publish(cloud_sor);
    pub_vg.publish(cloud_vg);
    pub_cluster.publish(cloud_clustered);
    pub_acc.publish(cloud_acc);
}

/***
** Constructor
***/
PclObstacleDetector::PclObstacleDetector(const ros::NodeHandle &nh)
: nh(nh)
, crop_min_z(20)
, crop_max_z(200.0)
, meanK(50)
, stddevMul(0.0)
, lx(2.0f)
, ly(2.0f)
, lz(2.0f)
, buffer_size(3)
, cluster_tolerance(1.5)
, cluster_min_points(10)
, cluster_max_points(100)

{
    this->sub = this->nh.subscribe("/camera_array/points2", 1, &PclObstacleDetector::cloudcb, this); //!!!!!!!!!!!!!!!!!!!
    this->pub_cut = this->nh.advertise<PointCloud>("/ptcloud/cut", 1);
    this->pub_sor = this->nh.advertise<PointCloud>("/ptcloud/sor", 1);
    this->pub_vg = this->nh.advertise<PointCloud>("/ptcloud/vg", 1);
    this->pub_cluster = this->nh.advertise<PointCloud>("/ptcloud/cluster", 1);
    this->pub_acc = this->nh.advertise<PointCloud>("/ptcloud/accumulated", 1); ///////////<!!!!!!!!!!!!!!!!
    ROS_INFO_STREAM("INIT: PCL Obstacle Detector initalized");

    dynamic_reconfigure::Server<clustering_ptcloud::pclseg_reConfig>::CallbackType cb;
    cb = boost::bind(&PclObstacleDetector::configCallback, this, _1, _2);
    dr_srv_.setCallback(cb);
}


/***
** Dynamic Reconfigure for dynamically tuning the parameters
***/
void PclObstacleDetector::configCallback(clustering_ptcloud::pclseg_reConfig &config, uint32_t level){
  ROS_INFO_STREAM("Mean: " << this->meanK << "   std:" << this->stddevMul <<"lx: " << this->lx
  << " ly:" << this->ly << " lz:" << this->lz);
    this->crop_min_z = config.crop_min_z;
    this->crop_max_z = config.crop_max_z;
    this->meanK = config.meanK;
    this->stddevMul = config.stddevMul;
    this->lx = config.lx;
    this->ly = config.ly;
    this->lz = config.lz;
    this->buffer_size = config.buffer_size;
    this->cluster_tolerance = config.cluster_tolerance;
    this->cluster_min_points = config.cluster_min_points;
    this->cluster_max_points = config.cluster_max_points;
}



void headingCallback(const custom_msgs::NorthEastHeading::ConstPtr& heading){
  heading_MA = heading->heading;
}



void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const custom_msgs::gnssGGA::ConstPtr& pos){
  std::ofstream myfile;
  myfile.open(path1, std::ios::out | std::ios::app);

  custom_msgs::DetectedObjects detected_objects;
  custom_msgs::Object object;

  std::vector<double> position_MA = lla2nedPiren(pos->latitude, pos->longitude, pos->altitude);

  for(int i = 0; i <cloud->points.size(); i++){
    std::vector<double> obj_NED = objectCoord2NED(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, position_MA[0], position_MA[1], position_MA[2], heading_MA);
    object.north = obj_NED[0];
    object.east  = obj_NED[1];
    object.down  = obj_NED[2];
    object.width = 0;
    object.Class = "0";
    detected_objects.objects.push_back(object);
  }

  detected_objects.header = pos->header;
  detected_objects_pub.publish(detected_objects);
  myfile.close();
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_obstale_detector");
    ros::NodeHandle nh("");
    ros::Rate r(15.0);

    std::ofstream myfile;
    myfile.open(path1, std::ios::out);
    myfile <<"time"<<","<< "depth" <<std::endl;

    message_filters::Subscriber<PointCloud> ptCloud_sub(nh, "/ptcloud/cluster", 1);
    message_filters::Subscriber<custom_msgs::gnssGGA> position_sub(nh, "/vectorVS330/fix", 1);

    TimeSynchronizer<PointCloud, custom_msgs::gnssGGA> sync(ptCloud_sub, position_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Subscriber heading_sub = nh.subscribe("/navigation/eta", 1000, headingCallback);
    detected_objects_pub = nh.advertise<custom_msgs::DetectedObjects>("detected_objects", 10);

    PclObstacleDetector pcl_obst_det(nh);
    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }
    return 0;
}
