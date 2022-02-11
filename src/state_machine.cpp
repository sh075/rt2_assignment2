#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include <rt2_assignment1/GotopointAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <chrono>

bool start = false;

bool not_moving = true;

std_msgs::Bool reached;

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res)
	{
		if (req.command == "start")
			{
				start = true;
			}
		else 
			{
				start = false;
			}
		return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_machine");
	ros::NodeHandle n;
	ros::NodeHandle n1;
	ros::NodeHandle n2;	
	ros::NodeHandle n3;	
	ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
	ros::ServiceClient client_rp = n1.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
	actionlib::SimpleActionClient<rt2_assignment1::GotopointAction> ac("/Gotopoint");
	ros::Publisher pub=n2.advertise<std_msgs::Bool>("/reach", 1000);
    ros::Publisher pub_time=n3.advertise<std_msgs::Float32>("/time", 1000);   
	rt2_assignment1::RandomPosition rp; 
	rp.request.x_max = 5.0;
	rp.request.x_min = -5.0;
	rp.request.y_max = 5.0;
	rp.request.y_min = -5.0;
    auto start_time=std::chrono::steady_clock::now();
    auto end_time=std::chrono::steady_clock::now();
    float elapsed_time;
    std_msgs::Float32 time_total; 
	while(ros::ok())
		{
			ros::spinOnce();
			if (start)
				{
					if (not_moving)
						{
							client_rp.call(rp);
							rt2_assignment1::GotopointGoal goal;
							goal.target_pose.header.frame_id = "base_link";
							goal.target_pose.header.stamp = ros::Time::now();
							goal.target_pose.pose.position.x = rp.response.x;
							goal.target_pose.pose.position.y = rp.response.y;
							goal.target_pose.pose.orientation.z = rp.response.theta;
							std::cout << "\nGoing to the position: x= " << rp.response.x << " y= " <<rp.response.y << " theta = " <<rp.response.theta << std::endl;
							ac.sendGoal(goal);
							not_moving = false;
						}
					else 
						{
							if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
								{
									std::cout << "Goal completed" << std::endl;
									not_moving= true;
									reached.data=true;
									pub.publish(reached);
									end_time=std::chrono::steady_clock::now();
									std::chrono::duration<double> elapsed_seconds = end_time-start_time;
									std::cout <<"elapsed time:" <<elapsed_seconds.count() <<"s\n";
									elapsed_time=float(elapsed_seconds.count());
									time_total.data=elapsed_time;
									pub_time.publish(time_total);



								}
						}
				}
			else 
				{
					if (!not_moving)
						{
							ac.cancelAllGoals();
							std::cout << "Goal cancelled" << std::endl;
							not_moving= true;
							reached.data=false;
							pub.publish(reached);
						}
				}
  

		}
   return 0;
}
