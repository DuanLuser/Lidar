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
#include<fstream>
#include "C3iroboticsLidar.h"
#include "CSerialConnection.h"

#define DEG2RAD(x) ((x)*M_PI/180.)
#define R (2*6) //6rps ,2s
#define SNUM 20 //
#define Zero 0.0001
#define Range 360
#define Threshold 0.05  // empty/nonemty; 每转的数据是否相似
//#define Threshold1 0.01 


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

//
typedef struct _site
{
    _site()
    {
        minAngle = 0.0;
        maxAngle = 0.0;
        averageDis = 0.0;
    }
    float   minAngle, maxAngle;
    float   averageDis;
}Site;


vector<RslidarDataComplete> readData()
{
    vector<RslidarDataComplete> data;
    data.resize(Range);
    ifstream readFile("empty.txt");
    float signal, angle, distance;
    for(int i=0; i<Range; i++)
    {
	readFile >> signal >> angle >> distance;
	data[i].signal=signal;
	data[i].angle=angle;
	data[i].distance=distance;
    }
    return data;
}

vector<Site> forSite(vector<RslidarDataComplete> empty_data, vector<RslidarDataComplete> all_lidar_scan_data[R])
{
    vector<Site> site;
    site.resize(SNUM);
    int s = 0;
    bool first = false;
    for (int i = 0; i < R/2; i++)
    {
	vector<RslidarDataComplete> d1 = all_lidar_scan_data[i];
	vector<RslidarDataComplete> d2 = all_lidar_scan_data[i+R/2];
	for (int j = 0; j < empty_data.size() && j < d1.size(); j++)
	{
	    if (empty_data[j].distance > Zero && d1[j].distance > Zero)
	    {
		if(abs(d1[j].distance - empty_data[j].distance) > Threshold && abs(d2[j].distance - d1[j].distance) < Threshold)
		{
		    int count = 1;
		    float Dis = d1[j].distance;
		    site[s].minAngle = d1[j].angle; 
		    j++;
		    while (empty_data[j].distance > Zero && d1[j].distance > Zero && abs(d1[j].distance - empty_data[j].distance) > Threshold && abs(d2[j].distance - d1[j].distance) < Threshold)
		    {
			Dis += d1[j].distance; count++;
			j++;
			if (j > empty_data.size()) break;
		    }
		    site[s].maxAngle = d1[j-1].angle;
		    site[s].averageDis = Dis / count;
		    s++; first = true;
		}
	    }
	}
	if (first == true)
	{/*
	    for(int i=0;i<empty_data.size();i++)
		printf("ea:%f ca1:%f ca2:%f ed:%f cd1:%f cd2:%f\n",empty_data[i].angle,d1[i].angle,d2[i].angle,empty_data[i].distance,d1[i].distance,d2[i].distance);
	    for (int i = 0; i < SNUM; i++)
	    {
		if (site[i].maxAngle-site[i].minAngle>5)
		{
		    printf("%d, 5minAngle:%f, 5maxAngle:%f, distance:%f\n", i, site[i].minAngle, site[i].maxAngle, site[i].averageDis);
		}
	    }
	    sleep(5);*/
	    break;
	}
    }
    return site;
}

bool checkforEmpty(vector<RslidarDataComplete> empty_data, vector<RslidarDataComplete> all_lidar_scan_data[R])
{
    for (int i = 0; i < R; i++)
    {
	vector<RslidarDataComplete> d = all_lidar_scan_data[i];
	for (int j = 0; j < empty_data.size() && j < d.size(); j++)
	{
	    int angle=0;
	    while(empty_data[j].distance > Zero && abs(d[j].distance - empty_data[j].distance) > Threshold)
	    {
		j++; angle++;
	    }
	    if(angle>=3)
		return false;
	}
    }
    return true;
}

int main(int argc, char * argv[])
{
    int opt_com_baudrate = 230400;
    string opt_com_path = "/dev/ttyUSB0";

    CSerialConnection serial_connect;
    C3iroboticsLidar robotics_lidar;

    serial_connect.setBaud(opt_com_baudrate);
    serial_connect.setPort(opt_com_path.c_str());
    if(serial_connect.openSimple())
    {
	;
	//printf("[AuxCtrl] Open serail port sucessful!\n");
        //printf("baud rate:%d\n",serial_connect.getBaud());
    }
    else
    {
        //printf("[AuxCtrl] Open serail port %s failed! \n", opt_com_path.c_str());
        return -1;
    }

    //printf("C3iroboticslidar connected\n");
    robotics_lidar.initilize(&serial_connect);

    
    vector<RslidarDataComplete> empty_data; //
    empty_data=readData();
    
    int row = 0;
    vector<RslidarDataComplete> all_lidar_scan_data[R];
    
    bool print=false;
    while (true)
    {
	if(print==true) break;
	
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
		//sleep(30);
                //printf("Lidar count %d!\n", lidar_scan_size);
		row++;
		if (row == R)
		{
		    int count=0;
		    bool empty=true; 
		    vector<Site> s = forSite(empty_data, all_lidar_scan_data);
		    for (int i = 0; i < SNUM; i++)
		    {
			if (s[i].maxAngle-s[i].minAngle>5)
			{
			    //printf("%d, minAngle:%f, maxAngle:%f, distance:%f\n", count, s[i].minAngle, s[i].maxAngle, s[i].averageDis);
			    printf("nonempty,%03d-%03d,%.2fm\n", int(s[i].minAngle),int(s[i].maxAngle), s[i].averageDis);
			    empty=false;  count++; print=true;
			}
		    }
		    if(empty==true)
		    {
			bool check=checkforEmpty(empty_data, all_lidar_scan_data);
			if(check==true)
			{
			    printf("empty\n");print=true;
			}
			//else
			//    printf("");
		    }
		    row = 0;
		}
                break;
            }
            case LIDAR_GRAB_ERRO:
            {
                break;
            }
            case LIDAR_GRAB_ELSE:
            {
                //printf("[Main] LIDAR_GRAB_ELSE!\n");
                break;
            }
        }
        //usleep(50);
    }

    return 0;
}
