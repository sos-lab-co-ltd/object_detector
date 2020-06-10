#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "visualization_msgs/Marker.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "object_detector.h"
#include "object_draw.h"

int main(int argc, char** argv)
{
    std::string serial_port_name = "/dev/ttyUSB0";
    std::string frame_id = "laser_frame";
    std::string pub_topicname_lidar = "scan";
    std::string pub_topicname_obj = "obj";
    std::string pub_topicname_obj_text = "obj_text";
    std::string pub_topicname_obj_box = "obj_box";

    sensor_msgs::PointCloud cloud;
    sensor_msgs::PointCloud2 object_cloud;    

    ros::init(argc, argv, "object_detector_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    nh_priv.param("serial_port_name", serial_port_name, serial_port_name);
	nh_priv.param("frame_id", frame_id, frame_id);
	nh_priv.param("pub_topicname_lidar", pub_topicname_lidar, pub_topicname_lidar);   
    nh_priv.param("pub_topicname_obj", pub_topicname_obj, pub_topicname_obj);
    nh_priv.param("pub_topicname_obj_text", pub_topicname_obj_text, pub_topicname_obj_text);
    nh_priv.param("pub_topicname_obj_box", pub_topicname_obj_box, pub_topicname_obj_box);

    DRAW draw; //open drawing function
    OD obj_detect(serial_port_name,921600); //open gl and object detection
	
    nh_priv.param("ref_save_time", draw.ref_save_time, draw.ref_save_time);
    nh_priv.param("rviz_scale", draw.rviz_scale, draw.rviz_scale);
    nh_priv.param("loop_hz", draw.loop_hz, draw.loop_hz);
    nh_priv.param("rviz_pub_hz", draw.rviz_pub_hz, draw.rviz_pub_hz);

    nh_priv.param("group_box_size_threshold", obj_detect.group_box_size_threshold, obj_detect.group_box_size_threshold);
	nh_priv.param("max_distance", obj_detect.max_distance, obj_detect.max_distance);
	nh_priv.param("min_distance", obj_detect.min_distance, obj_detect.min_distance);
	nh_priv.param("min_thr_dist", obj_detect.min_thr_dist, obj_detect.min_thr_dist);
	nh_priv.param("max_thr_dist", obj_detect.max_thr_dist, obj_detect.max_thr_dist);
	nh_priv.param("filt_max_dist", obj_detect.filt_max_dist, obj_detect.filt_max_dist);
	nh_priv.param("filt_min_dist", obj_detect.filt_min_dist, obj_detect.filt_min_dist);
	nh_priv.param("filt_max_point_num", obj_detect.filt_max_point_num, obj_detect.filt_max_point_num);
	nh_priv.param("filt_min_point_num", obj_detect.filt_min_point_num, obj_detect.filt_min_point_num);

    ros::Publisher data_pub = nh.advertise<sensor_msgs::LaserScan>(pub_topicname_lidar, 100);
    ros::Publisher obj_text_pub = nh.advertise<visualization_msgs::Marker>( pub_topicname_obj_text, 0 );
    ros::Publisher obj_box_pub = nh.advertise<visualization_msgs::Marker>( pub_topicname_obj_box, 0 );
    ros::Publisher cloud2_pub = nh.advertise<sensor_msgs::PointCloud2>(pub_topicname_obj, 50); 

    ros::Rate loop_rate(draw.loop_hz);

	std::vector<group_t> group_vector;
   
    ros::Time start_time = ros::Time::now();

    int count = 0;
    int pub_count = ((double)draw.loop_hz/(double)draw.rviz_pub_hz);
    while(ros::ok())
    {
        group_vector = obj_detect.object_detecion();  //update sensor, read gl data

		//save initial points for certain period
		if( (ros::Time::now()-start_time).toSec() < draw.ref_save_time ){
            obj_detect.SetRef(obj_detect.angle, obj_detect.radius, obj_detect.radius_ref, 0, obj_detect.max_distance);
		}
        if(draw.first){
            draw.first = false;
            for(int i=0;i<LIDAR_DATA_LENGTH;i++) draw.radius_ref[i] = obj_detect.radius[i];
        }

        //publish raw data
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = ros::Time::now();

        scan_msg.header.frame_id = frame_id;
        scan_msg.angle_min = obj_detect.angle[0];
        scan_msg.angle_max = obj_detect.angle[1000-1];
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(1000-1);
        scan_msg.range_min = 0.1;
        scan_msg.range_max = 30.0;
        scan_msg.ranges.resize(1000);
        for(int i=0; i<1000; i++)
        {
            scan_msg.ranges[i] = obj_detect.radius[i];
        }
        data_pub.publish(scan_msg);
 
        //publish object detection data
        cloud.points.clear();    
        for (int i = 0; i < group_vector.size(); i++){
			group_t group = group_vector[i];

            //marker initialization
			visualization_msgs::Marker text_marker,box_marker;

			//draw box and publish
			box_marker = draw.draw_box(i,group,0.01,0,1,0, 0.0);//boxnum, group, scale, r,g,b, height
            if(count % pub_count ==0 ) obj_box_pub.publish( box_marker );

			//draw text and publish
			int obj_idx = ( group.lidar_index.back() - group.lidar_index.front() )/2 + group.lidar_index.front(); //center idx of group to the whole lidar index
			double obstruction_x = draw.radius_ref[obj_idx]*cos(obj_detect.angle[obj_idx]);
			double obstruction_y = draw.radius_ref[obj_idx]*sin(obj_detect.angle[obj_idx]);

			if(obj_detect.near_object(group.center_x,group.center_y,obstruction_x,obstruction_y,obj_detect.group_box_size_threshold)){
				text_marker = draw.draw_text(i,group,draw.rviz_scale,0,1,0, 0.0); //obstruction object. green
			}else{
				text_marker = draw.draw_text(i,group,draw.rviz_scale,1,0,0, 0.0); //new object. red
			}
			if(count % pub_count ==0 ) obj_text_pub.publish( text_marker );

            //object detected pointcloud save
            for (int j=0;j<group.element_index.size();j++){
				int obj_idx = group.element_index[j];
				double x = obj_detect.radius[obj_idx]*cos(obj_detect.angle[obj_idx]);
				double y = obj_detect.radius[obj_idx]*sin(obj_detect.angle[obj_idx]);
                if(obj_detect.radius[obj_idx]<31.0){
                    cloud.header.stamp = ros::Time::now();                    
                    cloud.header.frame_id = frame_id;
                    geometry_msgs::Point32 single_point;
                    single_point.x = x;
                    single_point.y = y;
                    single_point.z = 0.0; 
                    cloud.points.push_back(single_point);
                }
			}
		}//for loop end
        
        //object detection point publish
        sensor_msgs::convertPointCloudToPointCloud2(cloud, object_cloud);        
        cloud2_pub.publish(object_cloud);
        
        count++;
        if(count== draw.loop_hz) count = 0;
                    
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 1;
}
