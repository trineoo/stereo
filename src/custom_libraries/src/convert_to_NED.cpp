#include "custom_libraries/convert_to_NED.hpp"
#include <vector>
#include <cmath>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/transform_broadcaster.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <Eigen/Dense>

//tf2_ros::Buffer tfBuffer;
//tf2_ros::TransformListener tfListener(tfBuffer);


/*
using namespace Eigen;
//Roll pitch and yaw in Radians
Quaternionf euler2quaternions(double roll, double pitch, double yaw){
  Quaternionf q;
  q = AngleAxisf(roll, Vector3f::UnitX())
      * AngleAxisf(pitch, Vector3f::UnitY())
      * AngleAxisf(yaw, Vector3f::UnitZ());
  return q;
}*/
/*
geometry_msgs::TransformStamped broadcast_tf(std:string parent, std:string child, std::vector<double> trans, double[] rot){
  geometry_msgs::TransformStamped t;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  tf2::Quaternion q;
  q.setRPY(rot[0], rot[1], rot[2]);
  q.normalize();
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  return t;
}*/


std::vector<double> lla2nedPiren(double lat_deg, double long_deg, double altitude){
    double lat0_deg  = 63.4389029083;
    double long0_deg = 10.39908278;
    double altitude0 = 39.923;

    double deg2rad = M_PI/180;
    double rad2deg = 180/M_PI;

    double lat_rad   = lat_deg * deg2rad;
    double long_rad  = long_deg * deg2rad;
    double lat0_rad  = lat0_deg * deg2rad;
    double long0_rad = long0_deg * deg2rad;

    //From vik 2012 - page 4 - table 1.1
    double re        = 6378137.0;  // [m]
    double rp        = 6356752.0;  // [m]
    double rad_curve = pow(re,2) / (sqrt(pow(re,2) * pow(cos(lat_rad),2) + pow(rp,2) * pow(sin(lat_rad),2)));  //N in vik

    // LLA to ECEF
    double xe = (rad_curve + altitude) * cos(lat_rad) * cos(long_rad);
    double ye = (rad_curve + altitude) * cos(lat_rad) * sin(long_rad);
    double ze = ((pow(rp, 2) / pow(re, 2)) * rad_curve + altitude) * sin(lat_rad);

    double xe0 = (rad_curve + altitude0) * cos(lat0_rad) * cos(long0_rad);
    double ye0 = (rad_curve + altitude0) * cos(lat0_rad) * sin(long0_rad);
    double ze0 = ((pow(rp, 2) / pow(re, 2)) * rad_curve + altitude0) * sin(lat0_rad);

    // ECEF to NED
    double cosPhi    = cos(lat0_rad);
    double sinPhi    = sin(lat0_rad);
    double cosLambda = cos(long0_rad);
    double sinLambda = sin(long0_rad);

    double dist   = cosLambda * xe + sinLambda * ye;
    double uNorth = -sinPhi * dist + cosPhi * ze;
    double vEast  = -sinLambda * xe + cosLambda * ye;
    double wDown  = -(cosPhi * dist + sinPhi * ze);

    // ECEF0 to NED0
    double cosPhi0    = cosPhi;
    double sinPhi0    = sinPhi;
    double cosLambda0 = cosLambda;
    double sinLambda0 = sinLambda;

    double dist0   = cosLambda0 * xe0 + sinLambda0 * ye0;
    double vEast0  = -sinLambda0 * xe0 + cosLambda0 * ye0;
    double wDown0  = -(cosPhi0 * dist0 + sinPhi0 * ze0);
    double uNorth0 = -sinPhi0 * dist0 + cosPhi0 * ze0;

    // Compute difference
    double N = uNorth - uNorth0;  // [m]
    double E = vEast - vEast0;   // [m]
    double D = wDown - wDown0;   // [m]

    std::vector<double> NED;
    NED.push_back(N);
    NED.push_back(E);
    NED.push_back(D);

    return NED;

}



std::vector<double> lla2ned(double lat_deg, double long_deg, double altitude, double lat0_deg, double long0_deg, double altitude0){
    double deg2rad = M_PI/180;
    double rad2deg = 180/M_PI;

    double lat_rad   = lat_deg * deg2rad;
    double long_rad  = long_deg * deg2rad;
    double lat0_rad  = lat0_deg * deg2rad;
    double long0_rad = long0_deg * deg2rad;

    //From vik 2012 - page 4 - table 1.1
    double re        = 6378137.0;  // [m]
    double rp        = 6356752.0;  // [m]
    double rad_curve = pow(re,2) / (sqrt(pow(re,2) * pow(cos(lat_rad),2) + pow(rp,2) * pow(sin(lat_rad),2)));  //N in vik

    // LLA to ECEF
    double xe = (rad_curve + altitude) * cos(lat_rad) * cos(long_rad);
    double ye = (rad_curve + altitude) * cos(lat_rad) * sin(long_rad);
    double ze = ((pow(rp, 2) / pow(re, 2)) * rad_curve + altitude) * sin(lat_rad);

    double xe0 = (rad_curve + altitude0) * cos(lat0_rad) * cos(long0_rad);
    double ye0 = (rad_curve + altitude0) * cos(lat0_rad) * sin(long0_rad);
    double ze0 = ((pow(rp, 2) / pow(re, 2)) * rad_curve + altitude0) * sin(lat0_rad);

    // ECEF to NED
    double cosPhi    = cos(lat0_rad);
    double sinPhi    = sin(lat0_rad);
    double cosLambda = cos(long0_rad);
    double sinLambda = sin(long0_rad);

    double dist   = cosLambda * xe + sinLambda * ye;
    double uNorth = -sinPhi * dist + cosPhi * ze;
    double vEast  = -sinLambda * xe + cosLambda * ye;
    double wDown  = -(cosPhi * dist + sinPhi * ze);

    // ECEF0 to NED0
    double cosPhi0    = cosPhi;
    double sinPhi0    = sinPhi;
    double cosLambda0 = cosLambda;
    double sinLambda0 = sinLambda;

    double dist0   = cosLambda0 * xe0 + sinLambda0 * ye0;
    double vEast0  = -sinLambda0 * xe0 + cosLambda0 * ye0;
    double wDown0  = -(cosPhi0 * dist0 + sinPhi0 * ze0);
    double uNorth0 = -sinPhi0 * dist0 + cosPhi0 * ze0;

    // Compute difference
    double N = uNorth - uNorth0;  // [m]
    double E = vEast - vEast0;   // [m]
    double D = wDown - wDown0;   // [m]

    std::vector<double> NED;
    NED.push_back(N);
    NED.push_back(E);
    NED.push_back(D);

    return NED;

}


std::vector<double> objectCoord2NED(double x, double y, double z, double n_MA, double e_MA, double d_MA, double heading_MA){
  double E = (x*cos(heading_MA) - sin(heading_MA)*z)+e_MA;
  double N = (x*sin(heading_MA) + cos(heading_MA)*z)+n_MA;
  double D = d_MA;
/*
  double N = (x+n_MA)*cos(heading_MA) - sin(heading_MA)*(y+e_MA);
  double E = (x+n_MA)*sin(heading_MA) + cos(heading_MA)*(y+e_MA);
  double D = d_MA;
*/
  std::vector<double> NED;
  NED.push_back(N);
  NED.push_back(E);
  NED.push_back(D);

  return NED;
}

/*
std::vector<double> objectCoord_to_NED(double[] object_coord, std::vector<double> ma_NED, double ma_heading){
  //Parent=piren, child=gps_antenna, translation = NED posisjon nå, orientation
  //transform fra origio til nåværende posisjon

  geometry_msgs::TransformStamped t = broadcast_tf("piren", "camera_frame", ma_NED, [0, 0, heading]);
  //NED i forhold til camera frame
  geometry_msgs::TransformStamped tarns = t.lookup_transform("camera_frame", "camera_frame", 2);

  double N_cam = trans.transform.translation.x + object_coord[0];
  double E_cam = trans.transform.translation.y + object_coord[1];
  double D_cam = MA_d;

  std::vector<double> NED_cam;
  NED_cam.push_back(N_cam);
  NED_cam.push_back(E_cam);
  NED_cam.push_back(D_cam);

  geometry_msgs::TransformStamped t1 = broadcast_tf("piren", "camera_frame", ma_NED, [0, 0, heading]);

  return NED;


}*/
