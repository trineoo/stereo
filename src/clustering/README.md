# Point Cloud Library Obstacle Detector
Detecting cones in PointCloud from Velodyne Lidar Puck.

## How to add dynamic reconfigure
1) Add new .cfg file
2) Make it executable
```bash
chmod a+x MyParams.cfg
```
3) Add MyParams.cfg to CMakelists
```bash
generate_dynamic_reconfigure_options(
  cfg/accumulator.cfg
  cfg/params.cfg
  # cfg/clusterer.cfg
)
```

4) Add stuff to package.xml
```xml
    <build_depend>dynamic_reconfigure</build_depend>
```
5) Add stuff to constructor
```c++
// Dynamic Reconfigure stuff
dynamic_reconfigure::Server<r18dv_pclseg::pclseg_reConfig>::CallbackType cb;
cb = boost::bind(&PclObstacleDetector::configCallback, this, _1, _2);
dr_srv_.setCallback(cb);
```

6) Add stuff to class definition
```c++
void configCallback(r18dv_pclseg::pclseg_reConfig &config, uint32_t level);

dynamic_reconfigure::Server<r18dv_pclseg::pclseg_reConfig> dr_srv_;
```

7) Add callback Function
```c++
void 
PclObstacleDetector::configCallback(r18dv_pclseg::pclseg_reConfig &config, uint32_t level){
    ROS_INFO_STREAM("PclObstacleDetector::configCallback reached: " << this->search_radius << ", new: " << config.search_radius);
    this->search_radius =                   config.search_radius;
    this->min_neighbors_in_radius =                   config.min_neighbors_in_radius;
    this->min_points =                   config.min_points;
    this->max_points =                   config.max_points;
    this->seg_cluster_tolerance =                   config.seg_cluster_tolerance;
    this->centroid_mode =                   config.centroid_mode;
}
```
#### Dynamic reconfigure tricks
With this setup, your param.yaml file will overwrite which default values you have set in the .cfg file.
On startup the configCallback function will be called when reading the .yaml-files, updating the values.
You don't need to close rqt after you have restarted nodes, even tho it will show the old values in dynamic reconfigure, the yaml values will be in use.
