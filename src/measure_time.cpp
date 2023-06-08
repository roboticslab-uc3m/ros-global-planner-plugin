/*  
 * University Carlos III of Madrid, Spain.
 * All rights reserved.
 *        Files: get_3d_demo.cpp
 *   Created on: 22, 12, 2021 
 *       Author: Fran J Naranjo 
 *        Email: fjnaranjoc@gmail.com
 */

// demo to test a loop to get the 3d point of the qr

#include <ros/ros.h>
#include <actionlib_msgs/GoalStatusArray.h>


int status_;
void callback(const actionlib_msgs::GoalStatusArray msg);


int main(int argc, char **argv)
{
    // Inicializating node
    ros::init(argc, argv, "measure_time");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber subs = nh.subscribe("/move_base/status", 1,callback);

    int status;
    bool first = true;
    bool goal_reached = false;
    ros::Time begin, end;
    std::cout << "Start " << std::endl;


    while(ros::ok() && !goal_reached){
        
        if (status_ == 1 && first)
        {
            first = false;
            begin = ros::Time::now();
            std::cout << "goal sent " << std::endl;

        }
        if (status_ == 3){
            goal_reached = true;
            end = ros::Time::now();
        }

        ros::spinOnce();
        ros::WallDuration(0.1).sleep();

    }
    
    std::cout << "Measuring time in reching goal: " << end - begin << std::endl;

    return 0;

}

void callback(const actionlib_msgs::GoalStatusArray msg)
{
    //std::cout << "callabck"<< std::endl;
    if(msg.status_list.size()>0)
        status_ = msg.status_list.back().status;
}
