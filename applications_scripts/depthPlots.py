#!/usr/bin/env python
#coding=utf-8

import os
import matplotlib.pyplot as plt
import subprocess
import matplotlib.cm as cm
import csv
import numpy as np
import math
from math import cos, sin
import statistics
import numpy as np

PATH_GPS = "Experiment/handheldGPS/4hanholdGPS_NED.csv"
PATH_MILLI = "Experiment/maGPS/4milli_bag_sat_time.csv"
PATH_STEREO = "Experiment/clusteringPtCloud_depths/bag4.csv"
#PATH_STEREO = "Experiment/clusteringCNN_Depths/bag5new.csv"






def read_gps_to_list(path):
    with open(path, "r") as gpsTrack:
	csvReader = csv.reader(gpsTrack)
	header = next(csvReader)

	northIndex = header.index("N")
	eastIndex = header.index("E")
	timeIndex = header.index("time")
	
	coordList = []

	for row in csvReader:
	    north = row[northIndex][1:]
	    east = row[eastIndex]
	    time = row[timeIndex].replace('+', ' ').split(' ')[1:2]
	    coordList.append([time, north, east])
    return coordList

def objectCoord2NED(x, y, z, n_ma, e_ma, d_ma, heading):
    ned = []
    N = ((z)*math.cos(heading)-(x)*math.sin(heading))+n_ma
    E = ((z)*math.sin(heading) + (x)*math.cos(heading))+e_ma
    D = d_ma
    ned = [N, E, D]
    return ned


def read_milli_to_list(path):
    with open(path, "r") as gpsTrack:
	csvReader = csv.reader(gpsTrack)
	header = next(csvReader)

	northIndex = header.index("N")
	eastIndex = header.index("E")
	downIndex = header.index("D")
	timeIndex = header.index("time")
	rosSecIndex = header.index("ros time sec")
	rosNsecIndex = header.index("ros time nsec")

	coordList = []

	for row in csvReader:
            north = row[northIndex][1:]
            east = row[eastIndex]
	    down = row[downIndex][0:len(row[downIndex])-1]
	    time = row[timeIndex].replace('.', ' ').split(' ')[1:2]
            rosSec = row[rosSecIndex]
            rosNsec = row[rosNsecIndex]

	    coordList.append([time,north,east,down,rosSec,rosNsec])

    return coordList


def median_if_multiple_timestamps(data):
    correct = []
    temp = []

    while len(data) >= 1:
	first = data.pop(0)
	temp.append(first)

	while len(data) >= 1:
	    cmp = data[0]
	    if cmp[0] == first[0]:
		old = data.pop(0)
		temp.append(old)
	    else:
		break

	new = temp[0]
	north = []
	east = []
	down = []
	rosNs = []
	for i in range(0,len(temp)):
	    north.append(float(temp[i][1]))
	    east.append(float(temp[i][2]))
	    down.append(float(temp[i][3]))
	    rosNs.append(int(temp[i][5]))

	north = statistics.median(north)
	east = statistics.median(east)
	down = statistics.median(down)
	rosNs = int(statistics.median(rosNs))
	new[1]=north
	new[2]=east
	new[3]=down
	new[5]=rosNs

	temp = []
	correct.append(new)

    return correct


def read_stereo_to_list(path):
    with open(path, "r") as stereoDepth:
	csvReader = csv.reader(stereoDepth)
	header = next(csvReader)
	rosSecIndex = header.index("time")
	xIndex = header.index("x")
	yIndex = header.index("y")
	zIndex = header.index("z")
	headingIndex = header.index("heading")

	coordList = []

	for row in csvReader:
            time = row[rosSecIndex]
            x = row[xIndex]
	    y = row[yIndex]
	    z = row[zIndex]
	    heading = row[headingIndex]

	    coordList.append([time,x,y,z,heading])

    return coordList

def calc_NED(stereo, milli):
    NED = []

    for coord in stereo:
	for row in milli:
	    if(coord[0] == row[4]):
		obj_coord = objectCoord2NED(float(coord[1]), float(coord[2]), float(coord[3]), float(row[1]), float(row[2]), float(row[3]), float(coord[4]))		
		rosTime = coord[0]
		NED.append([rosTime, obj_coord[0], obj_coord[1], obj_coord[2]])

    return NED


def plot(stereo, gps, milli):
    plt.figure(1)
    plt.xlabel("Time")
    plt.ylabel("North")
    plt.title("Scenario 6")
    plt.axis([0, 80, -40, 10])
	
    for j in range(0, len(gps)):
	plt.plot(j, float(gps[j][1]), '.', color='k')

    	for i in range(0, len(stereo)):
            if gps[j][0] == stereo[i][0]:
	        plt.plot(j, stereo[i][1], '.', color='deepskyblue')

    plt.tight_layout()
    plt.show()

    plt.figure(2)
    plt.xlabel("Time")
    plt.ylabel("East")
    plt.title("Scenario 6")
    plt.axis([0, 80, -350, -300])


    for j in range(0, len(gps)):
	plt.plot(j-42, float(gps[j][2]), '.',color='k')
        for i in range(0, len(stereo)):
            if gps[j][0] == stereo[i][0]:
                plt.plot(j, stereo[i][2], '.', color='deepskyblue')

  
    plt.tight_layout()
    plt.show()
   
    plt.figure(3)
    plt.xlabel("East")
    plt.ylabel("North")
    plt.title("ptCloud-clustering")
    plt.axis([-332,-318, -25, 10])

    for j in range(0, len(gps)):
	plt.plot(float(gps[j][2]), float(gps[j][1])-4.52, '.', color='k')

    for i in range(0, len(stereo)):
	plt.plot(stereo[i][2], stereo[i][1], '.',color='deepskyblue')
    plt.tight_layout()
    plt.show()




def calculate_depth(x1, y1, x2, y2):
    dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    return dist


def match_timestamp(gps, milli):
    time_depth = []
    count = 0
    for coord in milli:
    	if coord[0] == gps[count][0]:
	    depth = calculate_depth(float(coord[1]),float(coord[2]),float(gps[count][1]),float(gps[count][2]))
 	    rosTime = coord[4]
            time_depth.append([rosTime, gps[count][1], gps[count][2], depth])
	    count += 1
	else:
	    print(False)
    return time_depth

def find_depth_stereo(stereo, milli):
    new_list = []
    for coord in stereo:
	for coordMA in milli: 
    	    if float(coordMA[4]) == float(coord[0])-1:
	        depth = calculate_depth(float(coordMA[1]), float(coordMA[2]), float(coord[1]), float(coord[2]))     
	        new_list.append([float(coord[0]),coord[1],coord[2],coord[3],depth])
    return new_list


def plot_depth_error_percentage(gps, stereo):
    stereo.pop(0)
    plt.figure(6)
    plt.xlabel("Time")
    plt.ylabel("Depth error [%]")
    plt.title("Scenario 5")
    plt.axis([0,len(gps), -0.25, 0.07])
    for i in range (0, len(gps)):
        for j in range(0, len(stereo)):
            if(int(stereo[j][0])) == int(gps[i][0]):
                error = (float(stereo[j][4])-(float(gps[i][3])))/(float(gps[i][3]))
                plt.plot(i, error, '.', color='r')
    plt.tight_layout()
    plt.show()
    return



def plot_depth_error(gps, stereo):
    stereo.pop(0)
    plt.figure(6)
    plt.xlabel("Time")
    plt.ylabel("Distance error")
    plt.title("CNN-clustering")
    plt.axis([0, len(gps), -30, 10])
    for i in range (0, len(gps)):
        for j in range(0, len(stereo)):
            if(int(stereo[j][0])) == int(gps[i][0]):
                error = (float(stereo[j][4])-(float(gps[i][3])))
                plt.plot(gps[i][0], error, '.', color='r')
 
    plt.plot(-100, 300, '.', color='m', label="expected stereo [1 pixel error]")
    plt.plot(-100, float(stereo[j][4]), '.', color='r', label="error")
 
    plt.legend()
    plt.tight_layout()
    plt.show()
   
    return


	
def plot_north_error(gps, stereo):
    stereo.pop(0)
    plt.figure(6)
    plt.xlabel("Time")
    plt.ylabel("North error")
    plt.title("Scenario 6")
    plt.axis([0, len(gps), -40, 30])
    for i in range (0, len(gps)):
        for j in range(0, len(stereo)):
            if(int(stereo[j][0])) == int(gps[i][0]):
                error = (float(stereo[j][1])-(float(gps[i][1])))
                plt.plot(i, error, '.', color='r')
    plt.tight_layout()
    plt.show()
    return


	
def plot_east_error(gps, stereo):
    stereo.pop(0)
    plt.figure(6)
    plt.xlabel("Time")
    plt.ylabel("East error")
    plt.title("Scenario 6")
    plt.axis([0, len(gps), -10, 10])

    for i in range (0, len(gps)):
        for j in range(0, len(stereo)):
            if(int(stereo[j][0])) == int(gps[i][0]):
                error = (float(stereo[j][2])-(float(gps[i][2])))
                plt.plot(i, error, '.', color='r')
    plt.tight_layout()
    plt.show()
    return





def plot_depth(gps, stereo):
    plt.figure(7)
    plt.xlabel("Time")
    plt.ylabel("Depth")

    for j in range(0, len(gps)):
        plt.plot(float(gps[j][0]), float(gps[j][3]), '.', color='k')
    
    for i in range(0, len(stereo)):
        plt.plot(float(stereo[i][0]), float(stereo[i][4]), '.', color='deepskyblue')

    plt.plot(float(gps[j][0]), float(gps[j][3]), '.', color='k', label='GPS')
    plt.plot(float(gps[j][0]), float(gps[j][3]), '.', color='deepskyblue', label='CNN')
    plt.legend(bbox_to_anchor=(1.0, 1.0))

    plt.show() 






def main():
    milli_data = read_milli_to_list(PATH_MILLI)
    milli_data = median_if_multiple_timestamps(milli_data)

    stereo_data = read_stereo_to_list(PATH_STEREO)

    gps_data = read_gps_to_list(PATH_GPS)
    gps_data = match_timestamp(gps_data, milli_data)

    stereo_NED = calc_NED(stereo_data, milli_data)
    plot(stereo_NED, gps_data, milli_data)
    stereo_NED = find_depth_stereo(stereo_NED, milli_data)
   

    plot_depth_error_percentage(gps_data, stereo_NED)
    plot_depth_error(gps_data, stereo_NED)
    plot_depth(gps_data, stereo_NED)

    plot_north_error(gps_data, stereo_NED)

    plot_east_error(gps_data, stereo_NED)

    plot_depth_error(gps_data, stereo_NED)



if __name__ == "__main__":
    main()
