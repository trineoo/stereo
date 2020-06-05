#ifndef CONVERT_TO_NED_H
#define CONVERT_TO_NED_H

#include <ros/ros.h>

std::vector<double> lla2ned(double lat_deg, double long_deg, double altitude, double lat0_deg, double long0_deg, double altitude0);


std::vector<double> objectCoord2NED(double x, double y, double z, double n_MA, double e_MA, double d_MA, double heading_MA);


#endif
