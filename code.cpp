#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "core_api/GimbalSet.h"
#include <math.h>
#include <core_script_bridge/navigation_bridge.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/param.h>
#include <string>
#include <sensor_msgs/Range.h>

Navigation nav;

int flag = 0;
int lidar_set = 0;
float lidar_data = 0.0;
float ax,bx = 0.0;
float dx = 0.0;
float dy = 0.0;

//float a=b=0.0;
ros::Subscriber _parameter_updated_sub;
float curr_altitude = 0.0;

// controller variables for gimbal based on centroid of target

float gain_kp = 0.3;
float gain_ki = 0.1;
float gain_kd = 0.0066;

float prop_roll,prop_pitch,prop_yaw = 0.0;
float integral_roll,integral_pitch,integral_yaw = 0.0;
float der_pitch,der_yaw,der_roll = 0;

float prev_roll,prev_pitch,prev_yaw = 0.0;
float curr_roll,curr_pitch,curr_yaw = 0.0; 
float roll,pitch,yaw = 0.0;


float navXtr,navYtr=0.0;
geometry_msgs::TwistStampedConstPtr imu_data;
float data[3][3];
float yaw_of_drone = 0.0;
//image variables

int count = 0;
float pi = 3.14;
float norm_factor = 640/480;
float rad = pi/180;
bool do_rate(false);


//postion variables of the drone

float x,y = 0.0;
float curr_x_pos,curr_y_pos,curr_z_pos = 0.0;
float prev_error_x,prev_error_y = 0.0;
float error_x,error_y = 0.0;
float prop_x_pos,der_x_pos,prop_y_pos,der_y_pos,integral_x_pos,integral_y_pos = 0.0;
float x_pos,y_pos = 0.0;
float kp_pos = 1.2;
float kd_pos = 0.6;
float ki_pos = 0.01;
float x_est,y_est,z_est=0.0;
float prev_z_est,curr_z_est,error_z_est=0.0;
float kp_z = 1.1;
float kd_z = 0.06;
float corr_z=0.0;

//coordinates to hover and track the object without following
float hover_x,hover_y;
float go_to_x,go_to_y;
float total_angle = 0.0;	

//lidar variables
float lid_dist = 0.0;		

//list of parameters which can be modified from the web-app

struct param{
      float gain_kp;
      float gain_ki;
      float gain_kd;
      float kd_pos,kp_pos,ki_pos;
      bool gimbal_follow;
      bool hover_set;
      float max_pitch;
      float x_max;
	  float hover_radius;
	  float hover_angle;
      std::string global_namespace;
  }_param;

void paramUpdatedCb(const std_msgs::StringConstPtr &param_name);

void init()
  {
  	_param.gain_kp = 0.3;
    _param.gain_ki = 0.15;
    _param.gain_kd = 0.066;
    _param.kp_pos = 1.2;
	_param.ki_pos = 0.01;
    _param.kd_pos = 0.6;
    _param.gimbal_follow = false;
    _param.hover_set = false;
    _param.max_pitch = 0.3;
    _param.x_max = 4.0;
	_param.hover_radius = 1.0;
	_param.hover_angle = 10.0;
    ros::param::get("/global_namespace",_param.global_namespace);
	ros::NodeHandle n;
	_parameter_updated_sub = n.subscribe("/flytpod/parameter_updated",10,paramUpdatedCb);

	
 	std::string param_val;
 	if(ros::param::has("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_p"))
 	{
        ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_p",param_val);
        _param.gain_kp = std::stod(param_val);
    }
    else
    {
        ros::param::set("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_p",std::to_string(_param.gain_kp));
    }
    //-----------------------------------------------------------------------------------------------------------
    if(ros::param::has("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_i"))
 	{
        ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_i",param_val);
        _param.gain_ki = std::stod(param_val);
    }
    else
    {
        ros::param::set("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_i",std::to_string(_param.gain_ki));
    }
    //-------------------------------------------------------------------------------------------------------------
    if(ros::param::has("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_d"))
 	{
        ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_d",param_val);
        _param.gain_kd = std::stod(param_val);
    }
    else
    {
        ros::param::set("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_d",std::to_string(_param.gain_kd));
    }
    //-------------------------------------------------------------------------------------------------------------
    if(ros::param::has("/"+_param.global_namespace+"/parameters/flyt/ob_track_x_max"))
 	{
        ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_x_max",param_val);
        _param.x_max = std::stod(param_val);
    }
    else
    {
        ros::param::set("/"+_param.global_namespace+"/parameters/flyt/ob_track_x_max",std::to_string(_param.x_max));
    }
    //-------------------------------------------------------------------------------------------------------------
    if(ros::param::has("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_kp"))
 	{
        ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_kp",param_val);
        _param.kp_pos = std::stod(param_val);
    }
    else
    {
        ros::param::set("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_kp",std::to_string(_param.kp_pos));
    }
    //-------------------------------------------------------------------------------------------------------------
    if(ros::param::has("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_kd"))
 	{
        ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_kd",param_val);
        _param.kd_pos = std::stod(param_val);
    }
    else
    {
        ros::param::set("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_kd",std::to_string(_param.kd_pos));
    }
	//-------------------------------------------------------------------------------------------------------------
    if(ros::param::has("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_ki"))
 	{
        ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_ki",param_val);
        _param.ki_pos = std::stod(param_val);
    }
    else
    {
        ros::param::set("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_ki",std::to_string(_param.ki_pos));
    }
    //-------------------------------------------------------------------------------------------------------------
    if(ros::param::has("/"+_param.global_namespace+"/parameters/flyt/ob_track_gimbal_follow"))
	{
        ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_gimbal_follow",param_val);
        _param.gimbal_follow = std::stod(param_val);
    }
    else
    {
        ros::param::set("/"+_param.global_namespace+"/parameters/flyt/ob_track_gimbal_follow",std::to_string(_param.gimbal_follow));
    }
    //-------------------------------------------------------------------------------------------------------------
	if(ros::param::has("/"+_param.global_namespace+"/parameters/flyt/ob_track_gimbal_angle")){
        ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_gimbal_angle",param_val);
        _param.max_pitch = std::stod(param_val);
    }
    else
    {
        ros::param::set("/"+_param.global_namespace+"/parameters/flyt/ob_track_gimbal_angle",std::to_string(_param.max_pitch));    
    }
	//-----------------------------------------------------------------------------------------------------------
    if(ros::param::has("/"+_param.global_namespace+"/parameters/flyt/ob_track_hover_radius"))
 	{
        ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_hover_radius",param_val);
        _param.hover_radius = std::stod(param_val);
    }
    else
    {
        ros::param::set("/"+_param.global_namespace+"/parameters/flyt/ob_track_hover_radius",std::to_string(_param.hover_radius));
    }
    //-----------------------------------------------------------------------------------------------------------
    if(ros::param::has("/"+_param.global_namespace+"/parameters/flyt/ob_track_hover_angle"))
 	{
        ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_hover_angle",param_val);
        _param.hover_angle = std::stod(param_val);
    }
    else
    {
        ros::param::set("/"+_param.global_namespace+"/parameters/flyt/ob_track_hover_angle",std::to_string(_param.hover_angle));
    }
    //-----------------------------------------------------------------------------------------------------------
    if(ros::param::has("/"+_param.global_namespace+"/parameters/flyt/ob_track_hover_set"))
 	{
        ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_hover_set",param_val);
        _param.hover_set = std::stod(param_val);
    }
    else
    {
        ros::param::set("/"+_param.global_namespace+"/parameters/flyt/ob_track_hover_set",std::to_string(_param.hover_set));
    }

  }


void paramUpdatedCb(const std_msgs::StringConstPtr &param_name)

{
    std::string param_value;
    if(param_name->data == "ob_track_parameter_p")
	{
      ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_p",param_value);
	  ROS_INFO("param value_kp = %s",param_value.c_str());
      _param.gain_kp = std::stod(param_value);
    }
    else if(param_name->data == "ob_track_parameter_i")
	{
      ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_i",param_value);
      _param.gain_ki = std::stod(param_value);
    }
    else if(param_name->data == "ob_track_parameter_d")
	{
      ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_parameter_d",param_value);
      _param.gain_kd = std::stod(param_value);
    }
   else if(param_name->data == "ob_track_gimbal_follow")
	{
      ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_gimbal_follow",param_value);
      _param.gimbal_follow = std::stod(param_value);
    }
    else if(param_name->data == "ob_track_x_max")
	{
      ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_x_max",param_value);
	  ROS_INFO("param value x_max = %s",param_value.c_str());
      _param.x_max = std::stod(param_value);
    }
    else if(param_name->data == "ob_track_kp")
	{
      ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_kp",param_value);
	  ROS_INFO("param value kp = %s",param_value.c_str());
      _param.kp_pos = std::stod(param_value);
    }
    else if(param_name->data == "ob_track_kd")
	{
      ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_kd",param_value);
      _param.kd_pos = std::stod(param_value);
    }
	else if(param_name->data == "ob_track_ki")
	{
      ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_ki",param_value);
      _param.kd_pos = std::stod(param_value);
    }
    else if(param_name->data == "ob_track_gimbal_angle")
	{
      ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_gimbal_angle",param_value);
      _param.max_pitch = std::stod(param_value);
    }
	else if(param_name->data == "ob_track_hover_radius")
	{
      ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_hover_radius",param_value);
      _param.hover_radius = std::stod(param_value);
    }
	else if(param_name->data == "ob_track_hover_angle")
	{
      ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_hover_angle",param_value);
		ROS_INFO("param value angle = %s",param_value.c_str());
      _param.hover_angle = std::stod(param_value.c_str());
    }
    else if(param_name->data == "ob_track_hover_set")
	{
      ros::param::get("/"+_param.global_namespace+"/parameters/flyt/ob_track_hover_set",param_value);
      _param.hover_set = std::stod(param_value);
    }
}

void from_euler(float roll, float pitch, float yaw)
{
	float cp = cosf(pitch);
    float sp = sinf(pitch);
    float sr = sinf(roll);
    float cr = cosf(roll);
    float sy = sinf(yaw);
    float cy = cosf(yaw);

    data[0][0] = cp * cy;
    data[0][1] = (sr * sp * cy) - (cr * sy);
    data[0][2] = (cr * sp * cy) + (sr * sy);
    data[1][0] = cp * sy;
    data[1][1] = (sr * sp * sy) + (cr * cy);
    data[1][2] = (cr * sp * sy) - (sr * cy);
    data[2][0] = -sp;
    data[2][1] = sr * cp;
    data[2][2] = cr * cp;
}

void callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{

	std::vector<float> a = msg->data;
	//std::cout<< a[0] <<" "<< a[1] <<std::endl;
	count = count + 1;
	dx = a[0];
	dy = a[1];
	
	ax = dx;
	bx = dy;
/*
	if(ax > -(40.0) && ax < (40.0) && bx > -(40.0) && bx < (40.0))
	{
		
			flag = 1;
			std::cout<<"flag = 1"<<std::endl;
		 
	}
	else
	{
		flag = 0;
	}*/


	prev_pitch = curr_pitch;
	prev_yaw = curr_yaw;

	curr_pitch = (dx*0.016532);
	curr_yaw = (dy*0.016532);
	//std::cout<< a_x <<std::endl;
	
			
				
		prop_pitch = curr_pitch;
		prop_yaw = curr_yaw;
		integral_pitch = integral_pitch + curr_pitch;
		integral_yaw = integral_yaw + curr_yaw;

		der_pitch = (curr_pitch - prev_pitch);
		der_yaw = (curr_yaw - prev_yaw);
		
		pitch = (_param.gain_kp*prop_pitch) + (_param.gain_ki*integral_pitch) + (_param.gain_kd*der_pitch);
		yaw = (_param.gain_kp*prop_yaw) + (_param.gain_ki*integral_yaw) + (_param.gain_kd*der_yaw);

		pitch = -pitch;
		yaw = -yaw;

		if(pitch > 1.2)
		{
			pitch = 1.2;
		}
		

		//rosclient call to send command to the gimbal
		x = curr_altitude*(cos(yaw))*(1/tan(pitch));
		y = -curr_altitude*(sin(yaw))*(1/tan(pitch));
		//std::cout<<curr_altitude<<"  "<<y<<std::endl;
		float temp[3] = {x,y,0.0};
		float new_temp[3];
        if(imu_data)
        {
			std::cout<<"true"<<std::endl;
            from_euler(0,0,imu_data->twist.linear.z);
            for (int i = 0; i < 3; i++)
             {
                new_temp[i] = 0.0f;
                for (int j = 0; j < 3; j++)
                 {
                    new_temp[i] += data[i][j] * temp[j];
                }
            }
        }

       navXtr = new_temp[0];
       navYtr = new_temp[1];
		
		
		ros::NodeHandle n;
		ros::ServiceClient client = n.serviceClient<core_api::GimbalSet>("/flytpod/navigation/gimbal_set");
		core_api::GimbalSet srv;
		srv.request.pose.twist.linear.x = 0.0;
		srv.request.pose.twist.linear.y = pitch;
		srv.request.pose.twist.linear.z = yaw;
		srv.request.pose.twist.angular.x = 0.0;
		srv.request.pose.twist.angular.y = 0.0;
		srv.request.pose.twist.angular.z = 0.0;
		srv.request.do_rate = do_rate;
		client.call(srv);

		
	
	

}


void pos_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	
	prev_error_x = error_x;
	prev_error_y = error_y;
	error_x = navXtr;
	error_y = navYtr;

	prop_x_pos = (_param.kp_pos)*error_x;
	prop_y_pos = (_param.kp_pos)*error_y;
	der_x_pos = (_param.kd_pos)*(error_x - prev_error_x);
	der_y_pos = (_param.kd_pos)*(error_y - prev_error_y);
	integral_x_pos = _param.ki_pos*(integral_x_pos + error_x);
	integral_y_pos = _param.ki_pos*(integral_y_pos + error_x);

	x_pos = prop_x_pos + der_x_pos+ integral_x_pos;
	y_pos = prop_y_pos + der_y_pos+ integral_y_pos;;
	//y_pos = -y_pos;
	
	
	total_angle =  (_param.hover_angle);

	if(_param.hover_angle > 0 && _param.hover_angle <= 90)
	{
		hover_x = _param.hover_radius*sin(total_angle*(pi/180));
		hover_y = _param.hover_radius*cos(total_angle*(pi/180));
	}
	else if (_param.hover_angle > 90 && _param.hover_angle <= 180)
	{
		hover_x = _param.hover_radius*sin(total_angle*(pi/180));
		hover_y = _param.hover_radius*cos(total_angle*(pi/180));	
	}
	else if (_param.hover_angle > 180 && _param.hover_angle <= 270)
	{
		hover_x = _param.hover_radius*sin(total_angle*(pi/180));
		hover_y = _param.hover_radius*cos(total_angle*(pi/180));	
	}
	else if (_param.hover_angle > 270 && _param.hover_angle <= 360)
	{
		hover_x = _param.hover_radius*sin(total_angle*(pi/180));
		hover_y = _param.hover_radius*cos(total_angle*(pi/180));	
	}

	go_to_x = x_pos + hover_x;
	go_to_y = y_pos + hover_y;
	
	if(_param.hover_set == true && _param.gimbal_follow == false)
	{
		//delay(3000);
		nav.position_set(go_to_x,go_to_y,-curr_altitude,0.0,0.5,false,false,true,true);
		nav.position_hold();
		std::cout<<go_to_x<<"   "<<go_to_y<<"   "<<x_pos<<"   "<<y_pos<<std::endl;	
	}
	
	else if(_param.hover_set == true && _param.gimbal_follow == true)
	{
		nav.position_hold();
		std::cout<<" invalid input"<<y_pos<<std::endl;	
	}

	else if(_param.hover_set == false && _param.gimbal_follow == true)
	{
				lidar_set++;
				if(pitch > 1.19 && pitch < 1.67)
				{
					nav.position_hold();
					std::cout<<"quad in hover mode"<<std::endl;
				}
		
				else
				{
					
						nav.position_set(x_pos,y_pos,-(lidar_data),0.0,0.0,false,true,false,false);
						std::cout<<"x_position = "<<x_pos<<"y_position = "<<y_pos<<"  "<<-lidar_data<<std::endl;
					
				}
				
			
	}

	else if(_param.hover_set == false && _param.gimbal_follow == false)
	{
		nav.position_hold();
		//lidar_set = 0;
		std::cout<<" in position hold / hover mode"<<std::endl;
	}

}

void imu_callback(const geometry_msgs::TwistStampedConstPtr &imu_msg)
{
	imu_data = imu_msg;
	
}


void lidar_callback(const sensor_msgs::Range::ConstPtr& msg)
{
	curr_altitude = msg->range;
	if(lidar_set > 0 && lidar_set < 10)
	{
		lidar_data = curr_altitude;
	}
	//std::cout<<curr_altitude<<std::endl;
	
}


int main(int argc, char **argv)
{
	
	ros::init(argc,argv,"listener");
	ros::NodeHandle n;
	ros::Subscriber sub_1 = n.subscribe("/flytpod/object/centroid",100,callback);
	ros::Subscriber sub_2 = n.subscribe("/flytpod/mavros/local_position/local",100,pos_callback);
	ros::Subscriber sub_3 = n.subscribe("/flytpod/mavros/distance_sensor/lidarlite_pub",100,lidar_callback);
	ros::Subscriber sub_4 = n.subscribe("/flytpod/mavros/imu/data_euler",100,imu_callback);
	
	init();
	
	
	ros::spin();
	return 0;
}