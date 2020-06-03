/*
*  3iRoboticsLIDAR System II
*  Driver Interface
*
*  Copyright 2017 3iRobotics
*  All rights reserved.
*
*	Author: 3iRobotics, Data:2017-09-15
*
*/

#include <stdlib.h>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include "C3iroboticsLidar.h"
#include "CSerialConnection.h"

#define DEG2RAD(x) ((x)*M_PI/180.)
#define R (2*6) //6rps ,2s
#define SNUM 20 //
#define Zero 0.0001
#define Range 360
#define Threshold 0.05  // empty/nonemty; 每转的数据是否相似


typedef struct _rslidar_data
{
    _rslidar_data()
    {
        signal = 0;
        angle = 0.0;
        distance = 0.0;
    }
    uint8_t signal;
    float   angle;
    float   distance;
}RslidarDataComplete;


using namespace std;
using namespace everest::hwdrivers;


vector<RslidarDataComplete> chooseEmpty(vector<RslidarDataComplete> all_lidar_scan_data[R])
{
    bool compare[R];
    for (int i = 0; i < R-1; i++)
    {
	vector<RslidarDataComplete> d1 = all_lidar_scan_data[i];
	vector<RslidarDataComplete> d2 = all_lidar_scan_data[i+1];
	for (int j = 0; j < d1.size() && j < d2.size(); j++)
	{
	    if (d1[j].distance > Zero && d2[j].distance > Zero)
		if ((abs(d1[j].angle - d2[j].angle) > Threshold || abs(d1[j].distance - d2[j].distance) > Threshold))
		{
		    compare[i] = false; break;
		}
	}
	if (compare[i] != false)//
	{
	    compare[i] = true;
	    if (i - 2 >= 0 && compare[i - 1] == true && compare[i - 2] == true)
	    {
		return all_lidar_scan_data[i];
	    }
	}
    }
    return all_lidar_scan_data[R/2-1];//
}


int main(int argc, char * argv[])
{
    int opt_com_baudrate = 230400;
    string opt_com_path = "/dev/ttyUSB0";

    CSerialConnection serial_connect;
    C3iroboticsLidar robotics_lidar;

    serial_connect.setBaud(opt_com_baudrate);
    serial_connect.setPort(opt_com_path.c_str());
    if(serial_connect.openSimple()) ;
    else return -1;

    robotics_lidar.initilize(&serial_connect);

    int row = 0;
    vector<RslidarDataComplete> empty_data; //
    vector<RslidarDataComplete> all_lidar_scan_data[R];
    
    bool setempty=false;
    while (true)
    {
	if(setempty==true) break;
	
	TLidarGrabResult result = robotics_lidar.getScanData();
        switch(result)
        {
            case LIDAR_GRAB_ING:
            {
                break;
            }
            case LIDAR_GRAB_SUCESS:
            {
                TLidarScan lidar_scan = robotics_lidar.getLidarScan();
                size_t lidar_scan_size = lidar_scan.getSize();
		all_lidar_scan_data[row].resize(Range);
                RslidarDataComplete one_lidar_data;
		size_t i=0;
                for(int angle = 0; angle < Range; angle++)
                {
		    int count=0;
		    one_lidar_data.angle=angle;
		    one_lidar_data.signal=0;
		    one_lidar_data.distance=0;
		    while(i < lidar_scan_size && int(lidar_scan.angle[i]+0.5)==angle)
                    {
			one_lidar_data.signal += lidar_scan.signal[i];
			one_lidar_data.distance += lidar_scan.distance[i];
			count++; i++;
		    }
		    one_lidar_data.signal /=count;
		    one_lidar_data.distance /= count;
		    all_lidar_scan_data[row][angle] = one_lidar_data;
                }
		row++;
		if (row == R)
		{
		    if (empty_data.size() == 0)//
		    {
			empty_data = chooseEmpty(all_lidar_scan_data);
			setempty=true;
			ofstream out_txt_file;
			out_txt_file.open("./empty.txt", ios::out | ios::trunc);
			out_txt_file << fixed;
			for(int i = 0; i < empty_data.size(); i++)
			{
			    out_txt_file << setprecision(4);
			    out_txt_file << int(empty_data[i].signal)<<" ";
			    out_txt_file << int(empty_data[i].angle)<<" ";
			    out_txt_file << empty_data[i].distance<<endl;
			}
			// 关闭文件
			out_txt_file.close();
			printf("OK\n");
		    }
		    row=0;
		}
                break;
            }
            case LIDAR_GRAB_ERRO:
            {
                break;
            }
            case LIDAR_GRAB_ELSE:
            {
                break;
            }
        }
        //usleep(50);
    }

    return 0;
}
