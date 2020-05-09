#!/usr/bin/env python
import csv
import math
import numpy as np
from std_msgs.msg import String



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


def main():
    lat0_deg  = 63.4389029083
    long0_deg = 10.39908278
    altitude0 = 39.923

    lat_deg = 63.438977
    long_deg = 10.398927

    lla_points = np.loadtxt('track_points.csv', delimiter=',', skiprows=1, dtype=String)

    lla_points = np.delete(lla_points, np.s_[2:6],1)
    lla_points = np.delete(lla_points, np.s_[5:29],1)

    #with open('sjekk.csv','ab') as fd:
    #    np.savetxt(fd, llt_points, delimiter=",", fmt="%s")

    #lla_points = [lat_deg, long_deg]

    #a = np.asarray([ ["X","Y","track_fid","track_seg","ele","time"]])
    #with open('nyGPSData.csv','ab') as fd:
    #    np.savetxt(fd, a, delimiter=",", fmt="%s")

    for row in lla_points:
        NED = np.asarray([lla2ned(float(row[0]), float(row[1]), 0.0, lat0_deg, long0_deg, altitude0)])
        with open('nyGPSData.csv','ab') as fd:
            np.savetxt(fd, NED, delimiter=",", fmt="%s")

    NED = [lla2ned(lat_deg, long_deg, altitude0, lat0_deg, long0_deg, altitude0)]
    with open('nyGPSData.csv','ab') as fd:
        np.savetxt(fd, NED, delimiter=",", fmt="%s")



if __name__ == '__main__':
    main()
