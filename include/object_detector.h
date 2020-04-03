#pragma once
#include <vector>
#include <iostream>
#include <math.h>

#include "gl_driver.h"

#define LIDAR_DATA_LENGTH 1000

typedef struct _GROUP_T
{
    std::vector<int> element_index;
	std::vector<int> lidar_index;
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double center_x;
    double center_y;
} group_t;

class OD
{
	public: 
        Gl gl;
        double angle[LIDAR_DATA_LENGTH];
        double radius[LIDAR_DATA_LENGTH];
        double radius_ref[LIDAR_DATA_LENGTH];
        
        double max_distance = 30.0; //in m
        double min_distance = 0.1;	//in m
        double min_thr_dist = 0.06;	//in m
        double max_thr_dist = 0.2;	//in m

        double filt_max_dist = 20.0;//in m
        double filt_min_dist = 1.0;	//in m
        int filt_max_point_num = 10;	//in m
        int filt_min_point_num = 1;	//in m

        double screen_height = 720;
        double screen_width = 1280;

        double group_box_size_threshold = 0.8;//m^2
    	
		OD(std::string port, int baudrate);
		~OD();

		std::vector<group_t> object_detecion();
        bool near_object(double x1, double y1, double x2, double y2, double threshold);
		void Distance2Screen(double input_x, double input_y, double* output_x, double* output_y);
		void SetRef(double *angle, double *radius, double *radius_ref, double min_radius, double max_radius);
};