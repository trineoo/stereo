#include <ros/ros.h>

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <geometry_msgs/TransformStamped.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::CentroidPoint<pcl::PointXYZ> CentroidPoint;

class PclObstacleDetector {
public:
    PclObstacleDetector(const ros::NodeHandle &nh);
    void cloudcb(const PointCloud::ConstPtr &msg); //endre msg
private:

    // --- Parameters ---
    // Reducer
    double min_z;
    double max_z;
    double local_min_z;
    double local_max_z;
    double local_radius;

    // Segmenter
    double voxel_width;
    double voxel_height;
    double seg_cluster_tolerance;
    int min_cluster_points;
    int max_cluster_points;

    // Accumulator
    int buffer_size;

    // Clusterer
    double search_radius;
    int min_neighbors_in_radius;
    int min_points;
    int max_points;
    double clu_cluster_tolerance;
    bool centroid_mode;
    // --- End Parameters ---

    bool long_distance;
    float discard_distance;
    int it_count;

    // --- PointCloud Manipulators ---
    PointCloud::ConstPtr reduce_cloud(const PointCloud::ConstPtr &cloud_input);
    PointCloud::Ptr segment_cloud(const PointCloud::ConstPtr &cloud_input);
    PointCloud::Ptr accumulate_cloud(const PointCloud::ConstPtr &cloud_input);
    PointCloud::Ptr cluster_cloud(const PointCloud::ConstPtr &cloud_input);

    std::vector<PointCloud::Ptr> pcl_vector;

    // ROS
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub_reduced;
    ros::Publisher pub_segmented;
    ros::Publisher pub_acc;
    ros::Publisher pub_obstacles;
};

PointCloud::ConstPtr
PclObstacleDetector::reduce_cloud(const PointCloud::ConstPtr &cloud_input){

    PointCloud::Ptr cloud_filtered_a (new PointCloud);
    cloud_filtered_a->header = cloud_input->header;

    pcl::PointIndices::Ptr cloud_discard(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    for (int i = 0; i < (*cloud_input).size(); i++)
    {
        pcl::PointXYZ pt = cloud_input->points[i];

        if (pt.z < 0) {
          cloud_discard->indices.push_back(i);
        } else if (-pt.x > max_z || -pt.x < min_z) {
          cloud_discard->indices.push_back(i);
        } else {
          double dist = sqrt(pow(pt.z,2) + pow(pt.y,2));
          if (dist < local_radius) {
            cloud_discard->indices.push_back(i);
          }
        }

    }

    extract.setInputCloud(cloud_input);
    extract.setIndices(cloud_discard);
    extract.setNegative(true);
    extract.setKeepOrganized(true);
    extract.filter(*cloud_filtered_a);

    pub_reduced.publish(cloud_filtered_a);

    const PointCloud::ConstPtr cloud_filtered_a_const = cloud_filtered_a;
    return cloud_filtered_a_const;
}

PointCloud::Ptr
PclObstacleDetector::segment_cloud(const PointCloud::ConstPtr &cloud_input){
    PointCloud::Ptr cloud_downsampled(new PointCloud);
    cloud_downsampled->header = cloud_input->header;

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_input);
    sor.setLeafSize(this->voxel_width, this->voxel_width, this->voxel_height);
    sor.filter(*cloud_downsampled);


    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_input);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setClusterTolerance(this->seg_cluster_tolerance);
    euclid.setMinClusterSize(this->min_cluster_points);
    euclid.setMaxClusterSize(this->max_cluster_points);
    euclid.setSearchMethod(tree);
    euclid.setInputCloud(cloud_downsampled);

    std::vector<pcl::PointIndices> indices;
    euclid.extract(indices);
    PointCloud::Ptr cloud_output(new PointCloud);
    (*cloud_output).header = cloud_input->header;

    for (auto &i: indices) {
        CentroidPoint centroid;
        for (auto &j: i.indices) {
            centroid.add(cloud_downsampled->points[j]);
        }
        pcl::PointXYZ p;
        centroid.get(p);
	double dist = sqrt(pow(p.z,2) + pow(p.y,2));
	//if ((long_distance && dist > discard_distance) || !long_distance ){
	  (*cloud_output).points.push_back(p);
	//}
    }

    (*cloud_output).width = (*cloud_output).points.size();
    (*cloud_output).height = 1;
    (*cloud_output).is_dense = true;

    pub_segmented.publish(cloud_output);
    return cloud_output;
}


PointCloud::Ptr
PclObstacleDetector::accumulate_cloud(const PointCloud::ConstPtr &cloud_input){
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

        merged_cloud->header.frame_id = "OPAL";
        std::cout << "accumulated: " << pcl_vector.size() << std::endl;
        pub_acc.publish(merged_cloud);
        return merged_cloud;
    }else{
        ROS_WARN("Returned NULL PointCloud in Accumulate CB");
        return NULL;
    }
}

PointCloud::Ptr
PclObstacleDetector::cluster_cloud(const PointCloud::ConstPtr &cloud_input){
    PointCloud::Ptr inliers(new PointCloud);
    inliers->header = cloud_input->header;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> rom;
    rom.setInputCloud(cloud_input);
    rom.setRadiusSearch(this->search_radius);
    rom.setMinNeighborsInRadius(this->min_neighbors_in_radius);
    rom.filter(*inliers);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(inliers);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclid;
    euclid.setClusterTolerance(this->clu_cluster_tolerance);
    euclid.setMinClusterSize(this->min_points);
    euclid.setMaxClusterSize(this->max_points);
    euclid.setSearchMethod(tree);
    euclid.setInputCloud(inliers);

    std::vector<pcl::PointIndices> indices;
    euclid.extract(indices);

    PointCloud::Ptr centroid_cloud(new PointCloud);
    centroid_cloud->header = inliers->header;
    for (auto &i: indices) {
      CentroidPoint centroid;
      for (auto &j: i.indices) {
        centroid.add(inliers->points[j]);
      }
      pcl::PointXYZ p;
      centroid.get(p);
      centroid_cloud->points.push_back(p);
    }
    pub_obstacles.publish(centroid_cloud);
    return centroid_cloud;
}

void
PclObstacleDetector::cloudcb(const PointCloud::ConstPtr &cloud_input)
{
    const PointCloud::ConstPtr &cloud_raw = cloud_input;
    const PointCloud::ConstPtr &cloud_acc = accumulate_cloud(cloud_raw);
    const PointCloud::ConstPtr &cloud_segmented = segment_cloud(cloud_acc);
    //const PointCloud::ConstPtr &cloud_clustered = cluster_cloud(cloud_reduced);

    //Start counting when cloud is accumulated
    if (it_count > buffer_size && it_count%10 == 0) {
      ROS_DEBUG_STREAM("------------ Lidar counter ----------");
      ROS_DEBUG_STREAM("It: " << it_count);
      for (auto &p: cloud_segmented->points) {
	// Ros frame for easy comparison with rviz
	ROS_DEBUG_STREAM("Pos: " << p.z << " " << p.y << " " << -p.x);
      }
      ROS_DEBUG_STREAM("-------------------------------------");
    }
    it_count++;

    pub_reduced.publish(cloud_raw);
    pub_acc.publish(cloud_acc);
    pub_segmented.publish(cloud_segmented);
    //pub_obstacles.publish(cloud_clustered);
}

PclObstacleDetector::PclObstacleDetector(const ros::NodeHandle &nh)
: nh(nh)
, min_z(-7.0)
, max_z(7.0)
, local_radius(35000.0)
, voxel_width(5000.0)
, voxel_height(5000.0)
, seg_cluster_tolerance(400.0)
, min_cluster_points(1)
, max_cluster_points(50)
, buffer_size(1) //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
, search_radius(5000.0)
, min_neighbors_in_radius(1)
, min_points(1)
, max_points(20)
, clu_cluster_tolerance(10000.0)
, long_distance(true)
, discard_distance(120000.0)
, it_count(0)
{

    this->sub = this->nh.subscribe("/camera_array/points2", 1, &PclObstacleDetector::cloudcb, this); //!!!!!!!!!!!!!!!!!!!
    this->pub_reduced = this->nh.advertise<PointCloud>("/lidar/reduced", 1); ///////////<!!!!!!!!!!!!!!!!
    this->pub_segmented = this->nh.advertise<PointCloud>("/lidar/segmented", 1);  ///////////<!!!!!!!!!!!!!!!!
    this->pub_acc = this->nh.advertise<PointCloud>("/lidar/accumulated", 1); ///////////<!!!!!!!!!!!!!!!!
    ROS_INFO_STREAM("INIT: PCL Obstacle Detector initalized");
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_obstale_detector");
    ros::NodeHandle nh("");
    ros::Rate r(15.0);

    PclObstacleDetector pcl_obst_det(nh);
    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }
    return 0;
}
