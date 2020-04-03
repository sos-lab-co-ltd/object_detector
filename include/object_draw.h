#pragma once

#include "visualization_msgs/Marker.h"
#include <object_detector.h> //for typedef

class DRAW
{
	public: 

	    int loop_hz = 40;
    	int rviz_pub_hz = 10;

		double rviz_scale = 1.0;
		double area_max = 3.0;
		double area_interval = 0.5;
		double ref_save_time = 2.0;
		double radius_ref[LIDAR_DATA_LENGTH];
		double angle[LIDAR_DATA_LENGTH];

		bool first = true;

		DRAW();
		~DRAW();
		visualization_msgs::Marker draw_box(int box_num, group_t group, double scale, double r, double g, double b, double height);
		visualization_msgs::Marker draw_text(int text_num, group_t group, double scale, double r, double g, double b, double height);
		visualization_msgs::Marker draw_area(int area_num, double area_size, double scale, double r, double g, double b, double height);
		
	private:


};

