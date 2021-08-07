/**
* \file state_machine.cpp
* \brief This file implement a state machine that start and stops a go to point goal
* \author Shahrzad Eskandari Majdar
* \version 0.1
* \date 07/08/2021
*
* \param start boolean to know if to start of stop the go to point action
*
* \details
*
*
* Services : <BR>
* ° /user_interface
* ° /position_server
* 
* Action : <BR>
*   /go_to_point

*  Action Client: <BR>
*    /go_to_point
*
* Description :
*

 This node is a server for the user interface, it receives what the user
* 	 writes, and it start depending on it. If the client ask for the random 
* 	 position behaviour it calls the server '/random_position' and it waits 
*  	 for it to be finished Also checks if the client requests the behaviour
* 	 to stop. In that case, it cancels the previous goal and waits for the
* 	 next command.while, it also sends information to the user_interface
* 	 node. It publishes a topic when a goal has been reached (T) or cancelled
* 	 (F).whenevera goal is reached, it publishes on the topic '/time' the
* 	 time in seconds between the request of the goal and the completion of it.
*/

#include "ros/ros.h"
#include "rt2_assignment2/Command.h"
#include "rt2_assignment2/Position.h"
#include "rt2_assignment2/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment2/PositionAction.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"


bool start = false; ///< For setting the value of the request from the user interface
bool not_moving = true;	/* used to know wheather robot is already moving towards a goal or not*/
std_msgs::Bool reached;	/* Message to state wheather the goal was completed or cancelled   */



/**
* \brief callback function for handling the request sent from the user interface
* \param req the request sent from the client
* \param res the response to be sent from the server to the client 
* \return always true as this function cannot fail.
*
* This function receives the request sent from the user interface client and set the value
* the start global variable. 
*
*/
bool user_interface(rt2_assignment2::Command::Request &req, rt2_assignment2::Command::Response &res)
{
    // if the request command is start start is set to true
    if (req.command == "start")
    {
        start = true;
    }
    else
    {
        // if the command is not start we set start to false
        start = false;
    }
    return true;
}

/**
 * \brief main function 
 * \param argc 
 * \param argv
 
 * \return always 0 
 * 
 * The main funtion initializes the node, service and action client object and waits to receive a request to 
 * the initialized service.  
 */
int main(int argc, char **argv)   // Initialization of the node
{
    ros::init(argc, argv, "state_machine");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("/user_interface", user_interface);
    ros::ServiceClient client_rp = n.serviceClient<rt2_assignment2::RandomPosition>("/position_server");
    actionlib::SimpleActionClient<rt2_assignment2::PositionAction> ac("go_to_point", true);

    rt2_assignment2::RandomPosition rp;
    rt2_assignment2::PositionGoal goal;

    rp.request.x_max = 5.0;
    rp.request.x_min = -5.0;
    rp.request.y_max = 5.0;
    rp.request.y_min = -5.0;

    while (ros::ok())
    {
        ros::spinOnce();
        if (start)
        {
            client_rp.call(rp);
            goal.x = rp.response.x;
            goal.y = rp.response.y;
            goal.theta = rp.response.theta;

            ac.waitForServer(); //will wait for infinite time
            std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " << goal.theta << std::endl;
            ac.sendGoal(goal);
            while (true)
            {
                ros::spinOnce();
                if (start == false)
                {
                    ac.cancelGoal();
                    std::cout << "\nThe Goal has been Cancelled" << std::endl;
                    break;
                }
                else
                {
                    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        std::cout << "\nGoal Reached!" << std::endl;
                        break;
                    }
                }
            }
        }
    }
    return 0;
}

