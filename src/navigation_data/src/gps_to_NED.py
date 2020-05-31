#!/usr/bin/env python
import csv
import math
import string
import numpy as np
from std_msgs.msg import String
from math import sqrt, atan, sin, cos, pi, radians
from datetime import datetime, timedelta


def lla2ned(lat_deg, long_deg, altitude, lat0_deg, long0_deg, altitude0):
    deg2rad = np.pi/180
    rad2deg = 180/np.pi

    lat_rad   = lat_deg * deg2rad
    long_rad  = long_deg * deg2rad
    lat0_rad  = lat0_deg * deg2rad
    long0_rad = long0_deg * deg2rad

    #From vik 2012 - page 4 - table 1.1
    re        = 6378137.0  # [m]
    rp        = 6356752.0  # [m]
    rad_curve = pow(re,2) / (math.sqrt(pow(re,2) * pow(math.cos(lat_rad),2) + pow(rp,2) * pow(math.sin(lat_rad),2)))  #N in vik

    #LLA to ECEF
    xe = (rad_curve + altitude) * math.cos(lat_rad) * math.cos(long_rad)
    ye = (rad_curve + altitude) * math.cos(lat_rad) * math.sin(long_rad)
    ze = ((pow(rp, 2) / pow(re, 2)) * rad_curve + altitude) * math.sin(lat_rad)

    xe0 = (rad_curve + altitude0) * math.cos(lat0_rad) * math.cos(long0_rad)
    ye0 = (rad_curve + altitude0) * math.cos(lat0_rad) * math.sin(long0_rad)
    ze0 = ((pow(rp, 2) / pow(re, 2)) * rad_curve + altitude0) * math.sin(lat0_rad)

    # ECEF to NED
    cosPhi    = math.cos(lat0_rad)
    sinPhi    = math.sin(lat0_rad)
    cosLambda = math.cos(long0_rad)
    sinLambda = math.sin(long0_rad)

    dist   = cosLambda * xe + sinLambda * ye
    uNorth = -sinPhi * dist + cosPhi * ze
    vEast  = -sinLambda * xe + cosLambda * ye
    wDown  = -(cosPhi * dist + sinPhi * ze)

    # ECEF0 to NED0
    cosPhi0    = cosPhi
    sinPhi0    = sinPhi
    cosLambda0 = cosLambda
    sinLambda0 = sinLambda

    dist0   = cosLambda0 * xe0 + sinLambda0 * ye0
    vEast0  = -sinLambda0 * xe0 + cosLambda0 * ye0
    wDown0  = -(cosPhi0 * dist0 + sinPhi0 * ze0)
    uNorth0 = -sinPhi0 * dist0 + cosPhi0 * ze0

    # Compute difference
    N = uNorth - uNorth0  # [m]
    E = vEast - vEast0    # [m]
    D = wDown - wDown0    # [m]

    NED = [N, E, D]
    return NED


'''
def lla2ned(latitude, longitude, latitude_ref, longitude_ref):
	R = 6378137.0#Earth radius, WGS84
	f = 0.003352810664747#Flattening factor, WGS84

	d_lat = radians(latitude - latitude_ref)
	d_long = radians(longitude - longitude_ref)

	R_N = R/(sqrt(1.0-(2.0*f-f*f)*(sin(radians(latitude_ref))**2)))
	R_M = R_N*(1-(2*f-f*f))/(1-(2*f-f*f)*(sin(radians(latitude_ref))**2))

	dN = d_lat/(atan(1.0/R_M))
	dE = d_long/(atan(1.0/(R_N*cos(radians(latitude_ref)))))

	return [dN, dE, 0]
'''
def main():
    ### Piren
    lat0_deg  = 63.4389029083
    long0_deg = 10.39908278
    altitude0 = 39.923

    lla_points = np.loadtxt('track_points.csv', delimiter=',', skiprows=1, dtype=String)

    #lla_points = np.delete(lla_points, np.s_[2:6],1)
    #lla_points = np.delete(lla_points, np.s_[5:29],1)

    a = np.asarray([ ["N","E","D","time"]])
    with open('GPS_pluss_4.csv','ab') as fd:
        np.savetxt(fd, a, delimiter=",", fmt="%s")

    for row in lla_points:
        NED = lla2ned(float(row[1]), float(row[0]), float(row[5]) ,lat0_deg, long0_deg, altitude0)

        #string = row[6]
        #new_time = string[0:11] + str(int(string[11:12]) + 2) + string[13:len(string)]
        date_str = row[6]
        date = datetime.strptime(date_str, '%Y/%m/%d %H:%M:%S+%f')
        new_time = date + timedelta(hours=2) + timedelta(seconds=4)

        b = np.asarray([ [NED, new_time]])

        with open('GPS_pluss_4.csv','ab') as fd:
            np.savetxt(fd, b, delimiter=",", fmt="%s")




if __name__ == '__main__':
    main()
