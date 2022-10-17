/*  
 * University Carlos III of Madrid, Spain.
 * All rights reserved.
 *        Files: roboticslab_global_planner.cpp
 *   Created on: 27, 07, 2022 
 *       Author: Ainhoa de Matias 
 *        Email: amatias@pa.uc3m.es
 */

/*
 * Last update.
 *     Date: 28, 07, 2022
 *   Author: Ainhoa de Matias
 *    Email: amatias@pa.uc3m.es
 */

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

//ros
#include <ros/ros.h>

//measure time
#include <chrono>

#include <math.h>

//planner
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

//geometry
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
//tf
#include <tf/transform_broadcaster.h> 

//opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


//costmap
#include <nav_msgs/OccupancyGrid.h>
#include <vector>


namespace roboticslab{


class GlobalPlanner : public nav_core::BaseGlobalPlanner {

    private: 

        //costmap_2d::Costmap2DROS* _costmap_ros;
        //costmap_2d::Costmap2D* _costmap;


        //Costmap matrix
        cv::Mat MatCostmap, threshold_MatCostmap, layers_MatCostmap[8];
        int advance[8][2];
        //std::vector<std::vector<int>> advance_vector;

        int _pixelsX, _pixelsY, _pixelsX_reduced, _pixelsY_reduced;
        double _resolution;
        double _originX;
        double _originY;
        double _metersX, _metersY;
        double _planningScale;

    //METHODS
    
    public:

        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        // Callback for the current pose of the robot
        bool makePlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan
                    );
};






}; //nameespace
#endif
