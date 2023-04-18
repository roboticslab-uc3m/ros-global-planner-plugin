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
 *     Date: 05, 09, 2022
 *   Author: Ainhoa de Matias
 *    Email: amatias@pa.uc3m.es
 */

#include <pluginlib/class_list_macros.h>

#include "ros-global-planner-plugin/roboticslab_plan_node.hpp"
#include "ros-global-planner-plugin/roboticslab_global_planner.hpp"



//register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(roboticslab::GlobalPlanner, nav_core::BaseGlobalPlanner)

//Default Constructor
namespace roboticslab {

GlobalPlanner::GlobalPlanner(){
    //this->init();
}

GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
initialize(name, costmap_ros);
}


// void GlobalPlanner::init(){
//     std::cout<<"INIT GLOBAL PLANNER"<<std::endl;
// }

void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    std::cout<<"=============================== GLOBAL PLANNER: initialize begin"<<std::endl;
    ROS_INFO("=============================== roboticslab: GLOBAL PLANNER: initialize begin");


    _planningScale = 12.0;

    costmap_2d::Costmap2D* _costmap; 
    _costmap = costmap_ros->getCostmap();

    _pixelsX = _costmap->getSizeInCellsX(); // 204
    _pixelsY = _costmap->getSizeInCellsY(); // 297
    std::cout << "width: " << _pixelsX << std::endl;
    std::cout << "height: " << _pixelsY << std::endl;

    _resolution = _costmap->getResolution();
    _originX = _costmap->getOriginX();
    _originY = _costmap->getOriginY();
    _metersX = _costmap->getSizeInMetersX();
    _metersY = _costmap->getSizeInMetersY();
    
    std::cout<< "Resolution: "<<_resolution<<" Origin (x,y): "<<_originX<<", "<<_originY<<std::endl;

    //Save costmap pgm file (/home/ainhoa/.ros)
    _costmap->saveMap("costmap_savemap");

    //Save costmap png file (/home/ainhoa/.ros)
    MatCostmap = cv::Mat(_pixelsX, _pixelsY, CV_8UC1,cv::Scalar(0));
    for (int i=0; i<_pixelsX; i++){
        for(int j=0;j<_pixelsY; j++){
            MatCostmap.at<uchar>(i,j)=_costmap->getCost(i,j);
        }
    }

    cv::imwrite("costmap.png",MatCostmap);
    //cv::imshow("Costmap", MatCostmap);
   
    //costmap image resize()
    cv::Mat reduced_MatCostmap;
    //_pixelsX_reduced = 20;
    //_pixelsY_reduced = 40;
    _pixelsX_reduced = _pixelsX/_planningScale;
    _pixelsY_reduced = _pixelsY/_planningScale;

    cv::resize(MatCostmap, reduced_MatCostmap, cv::Size(_pixelsY_reduced,_pixelsX_reduced));
    MatCostmap.release();   //MUY IMPORTANTE
    cv::imwrite("reduced.png",reduced_MatCostmap);


    //Threshold costmap 0-255
    threshold_MatCostmap = cv::Mat(_pixelsX_reduced, _pixelsY_reduced, CV_8UC1,cv::Scalar(0));
    cv::threshold(reduced_MatCostmap, threshold_MatCostmap, 95, 255, cv::THRESH_BINARY);
    std::cout<<"Threshold costmap: "<<std::endl<<threshold_MatCostmap<<std::endl;
    reduced_MatCostmap.release();       //MUY IMPORTANTE
    cv::imwrite("threshold.png",threshold_MatCostmap);
    //cv::imshow("threshold", threshold_MatCostmap);

    // 0 UP
        
    layers_MatCostmap[0] = threshold_MatCostmap.clone();
    std::cout<<"0  UP "<<std::endl;

    for (int i=_pixelsX_reduced; i>0; i--){
        for (int j=0; j<_pixelsY_reduced; j++){
            //_costmap->setCost(i,j,0);
            if ( layers_MatCostmap[0].at<uchar>(i,j)==255 && (i!=_pixelsX_reduced-1) && (j!=0)){
                 layers_MatCostmap[0].at<uchar>(i+1,j)=255;
                //_costmap->setCost(i+1,j,255);

            }
        }
    }
            
    //_costmap->saveMap("costmap_0");
    cv::imwrite("0filterup.png", layers_MatCostmap[0]);
    //cv::imshow("filterup",  layers_MatCostmap[0]);
    // layers_MatCostmap[0].release();     
            
        
    //1 UP RIGHT
    std::cout<<"1 UP RIGHT"<<std::endl;
     layers_MatCostmap[1] = threshold_MatCostmap.clone();

    for (int i=_pixelsX_reduced; i>0; i--){
        for (int j=1; j<_pixelsY_reduced; j++){
            //_costmap->setCost(i,j,0);
            if ( layers_MatCostmap[1].at<uchar>(i,j)==255 && (i!=_pixelsX_reduced-1)){
                 layers_MatCostmap[1].at<uchar>((i+1),(j-1))=255;
                //_costmap->setCost(i+1,j-1,255);

            }
        }
    }
    //_costmap->saveMap("costmap_1");
    cv::imwrite("1filterupright.png", layers_MatCostmap[1]);
    //cv::imshow("filterupright",  layers_MatCostmap[1]);


    // 2 RIGHT
    std::cout<<"2 RIGHT"<<std::endl;
     layers_MatCostmap[2] = threshold_MatCostmap.clone();

    for (int i=1; i<_pixelsX_reduced; i++){
        for (int j=1; j<_pixelsY_reduced; j++){
            //_costmap->setCost(i,j,0);
            if ( layers_MatCostmap[2].at<uchar>(i,j)==255){
                 layers_MatCostmap[2].at<uchar>(i,j-1)=255;
                //_costmap->setCost(i,j-1,255);
            }
        }
    }
    //_costmap->saveMap("costmap_2");
    cv::imwrite("2filterright.png", layers_MatCostmap[2]);
    //cv::imshow("filterright",  layers_MatCostmap[2]);
        
        
    // 3 DOWN RIGHT
    std::cout<<"3 DOWN RIGHT"<<std::endl;
    layers_MatCostmap[3] = threshold_MatCostmap.clone();

    for (int i=1; i<_pixelsX_reduced; i++){
        for (int j=1; j<_pixelsY_reduced; j++){
            //_costmap->setCost(i,j,0);
            if (layers_MatCostmap[3].at<uchar>(i,j)==255){
                layers_MatCostmap[3].at<uchar>((i-1),(j-1))=255;
                //_costmap->setCost(i-1,j-1,255);
            }
        }
    }
    //_costmap->saveMap("costmap_3");
    cv::imwrite("3filterdownright.png",layers_MatCostmap[3]);
    //cv::imshow("filterdownright", layers_MatCostmap[3]);
    
        
    // 4 DOWN
    std::cout<<"4 DOWN"<<std::endl;
     layers_MatCostmap[4] = threshold_MatCostmap.clone();

    for (int i=1; i<_pixelsX_reduced; i++){
        for (int j=1; j<_pixelsY_reduced; j++){
            //_costmap->setCost(i,j,0);
            if ( layers_MatCostmap[4].at<uchar>(i,j)==255){
                 layers_MatCostmap[4].at<uchar>(i-1,j)=255;
                //_costmap->setCost(i-1,j,255);
            }
        }
    }
    //_costmap->saveMap("costmap_4");
    cv::imwrite("4filterdown.png", layers_MatCostmap[4]);
    //cv::imshow("filterdown",  layers_MatCostmap[4]);   
    

    // 5 DOWN LEFT
    std::cout<<"5 DOWN LEFT"<<std::endl;
     layers_MatCostmap[5] = threshold_MatCostmap.clone();

    for (int i=1; i<_pixelsX_reduced; i++){
        for (int j=1; j<_pixelsY_reduced; j++){
            //_costmap->setCost(i,j,0);
            if ( layers_MatCostmap[5].at<uchar>(i,j)==255){
                 layers_MatCostmap[5].at<uchar>((i-1),(j+1))=255;
                //_costmap->setCost(i-1,j+1,255);
            }
        }
    }
    //_costmap->saveMap("costmap_5");
    cv::imwrite("5filterdownleft.png", layers_MatCostmap[5]);
    // cv::imshow("filterdownleft",  layers_MatCostmap[5]);

    
    // 6 LEFT
    std::cout<<"6 LEFTT"<<std::endl;
     layers_MatCostmap[6] = threshold_MatCostmap.clone();

    for (int i=_pixelsX_reduced; i>0; i--){
        for (int j=_pixelsY_reduced; j>0; j--){
            //_costmap->setCost(i,j,0);
            if ( layers_MatCostmap[6].at<uchar>(i,j)==255 && (i!=_pixelsX_reduced-1) && (j!=_pixelsY_reduced-1)){
                 layers_MatCostmap[6].at<uchar>(i,j+1)=255;
                //_costmap->setCost(i,j+1,255);
            }
        }
    }   
    //_costmap->saveMap("costmap_6");
    cv::imwrite("6filterleft.png", layers_MatCostmap[6]);
    //cv::imshow("filterleft",  layers_MatCostmap[6]);
           

    //  7 UP LEFT
    std::cout<<"7 UP LEFT"<<std::endl;
    layers_MatCostmap[7] = threshold_MatCostmap.clone();

    for (int i=_pixelsX_reduced; i>0; i--){
        for (int j=1; j<_pixelsY_reduced; j++){
            _costmap->setCost(i,j,0);
            if (layers_MatCostmap[7].at<uchar>(i,j)==255 && (i!=_pixelsX_reduced-1)){
                layers_MatCostmap[7].at<uchar>((i+1),(j+1))=255;
                _costmap->setCost(i+1,j+1,255);
            }
        }
    }
    //_costmap->saveMap("costmap_7");
    cv::imwrite("7filterupleft.png",layers_MatCostmap[7]);
    // cv::imshow("filterupleftt", layers_MatCostmap[7]);

    // advance[floor][X/Y]
    advance[0][0] = -1; advance[0][1] = 0; //UP CASE 0
    advance[1][0] = -1; advance[1][1] = 1; //UP RIGHT CASE 1
    advance[2][0] = 0; advance[2][1] = 1; //RIGHT CASE 2
    advance[3][0] = 1; advance[3][1] = 1; //DOWN RIGHT CASE 3
    advance[4][0] = 1; advance[4][1] = 0; //DOWN CASE 4
    advance[5][0] = 1; advance[5][1] = -1; //DOWN LEFT CASE 5
    advance[6][0] = 0; advance[6][1] = -1; //LEFT CASE 6 
    advance[7][0] = -1; advance[7][1] = -1; //UP LEFT CASE 7 

    // advance_vector = {
    //     {-1, 0},        //UP CASE 0
    //     {-1, 1},        //UP RIGHT CASE 1
    //     {0, 1},         //RIGHT CASE 2
    //     {1, 1}          //DOWN RIGHT CASE 3
    //     {1, 0},         //DOWN CASE 4
    //     {1, -1},        //DOWN LEFT CASE 5
    //     {0, -1},        //LEFT CASE 6
    //     {-1, -1}        //UP LEFT CASE 7
    // };

    std::cout<<"Finish initializing roboticslab global planner"<<std::endl;
    //cv::waitKey();
    std::cout<<"=============================== GLOBAL PLANNER: initialize end"<<std::endl;
    ROS_INFO("=============================== roboticslab: GLOBAL PLANNER: initialize end");

}       //initialize


bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    std::cout<<"=============================== GLOBAL PLANNER: makePlan begin"<<std::endl;
    ROS_INFO("=============================== roboticslab:GLOBAL PLANNER: makePlan begin");


    double pixelPerMeterX = _pixelsX/_metersX;
    double pixelPerMeterY = _pixelsY/_metersY;
    std::cout << "(using resolution instead) pixelPerMeterX, pixelPerMeterY: " << pixelPerMeterX << ", " << pixelPerMeterY << std::endl;

    std::cout << "Resolution: " << _resolution << std::endl;
    std::cout << "Inverse of Resolution: " << 1.0/_resolution << std::endl;

    std::cout << "_metersX, _metersY: " << _metersX << ", " << _metersY << std::endl;
    std::cout << "_pixelsX, _pixelsY: " << _pixelsX << ", " << _pixelsY << std::endl;
    std::cout << "_pixelsX_reduced, _pixelsY_reduced: " << _pixelsX_reduced << ", " << _pixelsY_reduced << std::endl;

    std::cout<<"Start (meters)(x,y): "<<start.pose.position.x<<", "<< start.pose.position.y<<std::endl;
    std::cout<<"Goal (meters)(x,y): "<<goal.pose.position.x<<", "<< goal.pose.position.y<<std::endl;

    std::cout << "Origin (meters)(x,y): " << _originX << ", " << _originY << std::endl;

    double offsetStartMeterX = start.pose.position.x - _originX;
    double offsetStartMeterY = start.pose.position.y - _originY;
    double offsetGoalMeterX = goal.pose.position.x - _originX;
    double offsetGoalMeterY = goal.pose.position.y - _originY;

    std::cout<<"offsetStartMeter (meters)(x,y): "<<offsetStartMeterX<<", "<< offsetStartMeterY<<std::endl;
    std::cout<<"offsetGoalMeter (meters)(x,y): "<<offsetGoalMeterX<<", "<< offsetGoalMeterY<<std::endl;

    int startX = offsetStartMeterX / (_resolution * _planningScale);
    int startY = offsetStartMeterY / (_resolution * _planningScale);
    int goalX = offsetGoalMeterX / (_resolution * _planningScale);
    int goalY = offsetGoalMeterY / (_resolution * _planningScale);

    std::cout<<"Start (pixels): "<<startX<<", "<< startY<<std::endl;
    std::cout<<"Goal (pixels): "<<goalX<<", "<< goalY<<std::endl;

    if ((startX<0)||(startY<0)||(goalX<0)||(goalY<0))
    {
        std::cout << "Check above values, must be positive! bye!" << std::endl;
        return false;
    }

    // * Orientation

    double roll,pitch,yaw;

    // ** Orientation: Start

    geometry_msgs::Quaternion start_quaternion = start.pose.orientation;
    tf::Quaternion tf_start;
    tf::quaternionMsgToTF(start_quaternion, tf_start);
    tf::Matrix3x3(tf_start).getRPY(roll,pitch,yaw);
    
    std::cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl;

    int floorInit = -1;
    int num_floors = 8;

    std::cout << "YAW/num_floors: "<< yaw/num_floors << std::endl;

    for (int i=0; i<num_floors; i++)
    {
        std::cout<< "kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk"<<std::endl;
        std::cout<< "yaw y los limites: "<< yaw <<std::endl;
        std::cout<<M_PI-(M_PI/num_floors)*(2*(i-1)+1)<< std::endl;
        std::cout<<M_PI-(M_PI/num_floors)*(2*i+1)<< std::endl;
        if(i ==0)
        {
            if((yaw>=2.74 && yaw<3.14) || (yaw>=-3.14 && yaw<-2.74))     //UP CASE 0
            {floorInit = 0;}
        }
        else
        {
            if(yaw<=(M_PI-(M_PI/num_floors)*(2*(i-1)+1)) && yaw>(M_PI-(M_PI/num_floors)*(2*i+1)) )
            {
                std::cout<< "dentro del if"<<std::endl;
                floorInit = i;
            }
        }   
    }

   
    // if((yaw>=2.74 && yaw<3.14) || (yaw>=-3.14 && yaw<-2.74)){     //UP CASE 0
    //     floor = 0;
    // }else if(yaw>=1.96 && yaw<2.74){                     //UP-RIGHT CASE 1
    //     floor=1;
    // }else if(yaw>=1.17 && yaw<1.96){                     //RIGHT CASE 2
    //     floor=2;
    // }else if(yaw>=0.39 && yaw<1.17){                     //DOWN-RIGHT CASE 3
    //     floor=3;
    // }else if((yaw>=-0.39 && yaw<0) || (yaw>=0 && yaw<0.39)){     //DOWN CASE 4
    //     floor=4;
    // }else if(yaw>=-1.17 && yaw<-0.39){                   //DOWN-LEFT CASE 5
    //     floor=5;
    // }else if(yaw>=-1.96 && yaw<-1.17){                  //LEFT CASE 6
    //     floor=6;
    // }else if(yaw>=-2.47 && yaw<-1.96){                  //UP-LEFT CASE 7
    //     floor=7;
    // }else{
    //     std::cout << "Unidentified floor for yaw [" << yaw << "]. bye!" << std::endl;
    //     return false;
    // }

    std::cout<<"Start floor: "<<floorInit<<std::endl;
    ROS_INFO_STREAM("=============================== roboticslab: GLOBAL PLANNER: start floor: "<< floorInit<< " .");


    // ** Orientation: Goal

    int goalFloor = -1;

    geometry_msgs::Quaternion goal_quaternion = goal.pose.orientation;
    tf::Quaternion tf_goal;
    tf::quaternionMsgToTF(goal_quaternion, tf_goal);
    tf::Matrix3x3(tf_goal).getRPY(roll,pitch,yaw);
    
    std::cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl;

    for (int i=0; i<num_floors; i++)
    {
        std::cout<< "kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk"<<std::endl;
        std::cout<< "yaw y los limites: "<< yaw <<std::endl;
        std::cout<<M_PI-(M_PI/num_floors)*(2*(i-1)+1)<< std::endl;
        std::cout<<M_PI-(M_PI/num_floors)*(2*i+1)<< std::endl;
        if(i ==0)
        {
            if((yaw>=2.74 && yaw<3.14) || (yaw>=-3.14 && yaw<-2.74))     //UP CASE 0
            {goalFloor = 0;}
        }
        else
        {
            if(yaw<=(M_PI-(M_PI/num_floors)*(2*(i-1)+1)) && yaw>(M_PI-(M_PI/num_floors)*(2*i+1)) )
            {
                std::cout<< "dentro del if"<<std::endl;
                goalFloor = i;
            }
        }        
    }

    // if((yaw>=2.74 && yaw<3.14) || (yaw>=-3.14 && yaw<-2.74)){     //UP CASE 0
    //     goalFloor = 0;
    // }else if(yaw>=1.96 && yaw<2.74){                     //UP-RIGHT CASE 1
    //     goalFloor=1;
    // }else if(yaw>=1.17 && yaw<1.96){                     //RIGHT CASE 2
    //     goalFloor=2;
    // }else if(yaw>=0.39 && yaw<1.17){                     //DOWN-RIGHT CASE 3
    //     goalFloor=3;
    // }else if((yaw>=-0.39 && yaw<0) || (yaw>=0 && yaw<0.39)){     //DOWN CASE 4
    //     goalFloor=4;
    // }else if(yaw>=-1.17 && yaw<-0.39){                   //DOWN-LEFT CASE 5
    //     goalFloor=5;
    // }else if(yaw>=-1.96 && yaw<-1.17){                  //LEFT CASE 6
    //     goalFloor=6;
    // }else if(yaw>=-2.74 && yaw<-1.96){                  //UP-LEFT CASE 7
    //     goalFloor=7;
    // }else{
    //     std::cout << "Unidentified goalFloor for yaw [" << yaw << "]. bye!" << std::endl;
    //     return false;
    // }

    int otherFloor = floor((yaw+M_PI)*8/(2*M_PI));
    std::cout<<"Goal floor: "<<goalFloor<<std::endl;
    std::cout<<"Other floor: "<<otherFloor<<std::endl;
    ROS_INFO_STREAM(" =============================== roboticslab: GLOBAL PLANNER: goal floor: "<< goalFloor<< " .");


    //BFS planning
    // ### * 0: libre
    // ### * 255: ocupado 
    // ### * 80: visitado
    // ### * 180: start
    // ### * 181: goal

    cv::Mat copy_layers_MatCostmap[8];
    for(int i=0;i<8;i++)
        copy_layers_MatCostmap[i] = layers_MatCostmap[i].clone();
    
    copy_layers_MatCostmap[floorInit].at<uchar>(startX,startY) = 180;
    copy_layers_MatCostmap[goalFloor].at<uchar>(goalX,goalY) = 181;

    std::cout<< "After changing values of start and goal in map"<<std::endl;
    
    std::vector<Node*> nodes;

    Node* init = new Node(startX, startY, floorInit, 0, -2);
    nodes.push_back(init);
    std::cout<< "After pushback"<<std::endl;

    bool done = false;
    int goalParentId;

    //Measure time find path
    std::chrono::steady_clock::time_point begin_path = std::chrono::steady_clock::now();

    while (!done)
    {
        std::cout<< "Looking for the goal"<<std::endl;
        ROS_INFO(" =============================== roboticslab: GLOBAL PLANNER:looking for the goal");

        int keepNodeSize = nodes.size();
        std::cout<<"Nodes size: "<<keepNodeSize<<std::endl;
        
        for(int nodeIdx=0; nodeIdx<keepNodeSize; nodeIdx++)
        {
            std::cout<<"# Unit depth step of Node (" << nodes[nodeIdx]->_id << "): X: " << nodes[nodeIdx]->_x << ", Y: " << nodes[nodeIdx]->_y << " , Z: " << nodes[nodeIdx]->_z << std::endl;

            int tmpX, tmpY, tmpZ;
            
            //RELATIVE ADVANCE
            tmpZ = nodes[nodeIdx]->_z;
            tmpX = nodes[nodeIdx]->_x + advance[tmpZ][0];
            tmpY = nodes[nodeIdx]->_y + advance[tmpZ][1];
            if( copy_layers_MatCostmap[tmpZ].at<uchar>(tmpX, tmpY) == 181)
            {
                std::cout<<"## RELATIVE ADVANCE: Goal reached---------------"<<std::endl;
                goalParentId = nodes[nodeIdx]->_id;
                done = true;
                break;
            }
            else if( copy_layers_MatCostmap[tmpZ].at<uchar>(tmpX, tmpY) == 0 ){
                Node* node = new Node(tmpX, tmpY, tmpZ, nodes.size(), nodes[nodeIdx]->_id);
                std::cout<<"## RELATIVE ADVANCE: Create Node (" << node->_id << "): X: " << node->_x << ", Y: " << node->_y << " , Z: " << node->_z << std::endl;
                copy_layers_MatCostmap[tmpZ].at<uchar>(tmpX, tmpY) = 80;
                nodes.push_back(node);
            }else if( copy_layers_MatCostmap[tmpZ].at<uchar>(tmpX, tmpY) == 80 ){
                std::cout<<"## RELATIVE ADVANCE: Already Visited"<<std::endl;
            }else
                std::cout<<"## RELATIVE ADVANCE: Wall or similar"<<std::endl;
            

            //RELATIVE ROTATE RIGHT
            tmpX = nodes[nodeIdx]->_x;
            tmpY = nodes[nodeIdx]->_y;
            tmpZ = nodes[nodeIdx]->_z+1;
            if(tmpZ==8)
                tmpZ=0;
            if( copy_layers_MatCostmap[tmpZ].at<uchar>(tmpX, tmpY) == 181)
            {
                std::cout<<"## RELATIVE ROTATE RIGHT: Goal reached------------------"<<std::endl;
                goalParentId = nodes[nodeIdx]->_id;
                done = true;
                break;
            }
            else if( copy_layers_MatCostmap[tmpZ].at<uchar>(tmpX, tmpY) == 0 )
            {
                Node* node = new Node(tmpX, tmpY, tmpZ, nodes.size(), nodes[nodeIdx]->_id);
                std::cout<<"## RELATIVE ROTATE RIGHT: Create Node (" << node->_id << "): X: " << node->_x << ", Y: " << node->_y << " , Z: " << node->_z << std::endl;
                copy_layers_MatCostmap[tmpZ].at<uchar>(tmpX, tmpY) = 80;
                nodes.push_back(node);
            }else if( copy_layers_MatCostmap[tmpZ].at<uchar>(tmpX, tmpY) == 80 ){
                std::cout<<"## RELATIVE ROTATE RIGHT: Already Visited"<<std::endl;
            }else
                std::cout<<"## RELATIVE ROTATE RIGHT: Wall or similar"<<std::endl;

            //RELATIVE ROTATE LEFT
            tmpX = nodes[nodeIdx]->_x;
            tmpY = nodes[nodeIdx]->_y;
            tmpZ = nodes[nodeIdx]->_z-1;
            if(tmpZ==-1)
                tmpZ=7;
            if( copy_layers_MatCostmap[tmpZ].at<uchar>(tmpX, tmpY) == 181)
            {
                std::cout<<"## RELATIVE ROTATE LEFT: Goal reached----------------"<<std::endl;
                goalParentId = nodes[nodeIdx]->_id;
                done = true;
                break;
            }
            else if( copy_layers_MatCostmap[tmpZ].at<uchar>(tmpX, tmpY) == 0 )
            {
                Node* node = new Node(tmpX, tmpY, tmpZ, nodes.size(), nodes[nodeIdx]->_id);
                std::cout<<"## RELATIVE ROTATE LEFT: Create Node (" << node->_id << "): X: " << node->_x << ", Y: " << node->_y << " , Z: " << node->_z << std::endl;
                copy_layers_MatCostmap[tmpZ].at<uchar>(tmpX, tmpY) = 80;
                nodes.push_back(node);
            }else if( copy_layers_MatCostmap[tmpZ].at<uchar>(tmpX, tmpY) == 80 ){
                std::cout<<"## RELATIVE ROTATE LEFT: Already Visited"<<std::endl;
            }else
                std::cout<<"## RELATIVE ROTATE LEFT: Wall or similar"<<std::endl;


        }

    }

    //Measure time find path
    std::chrono::steady_clock::time_point end_path = std::chrono::steady_clock::now();
    std::cout << "Time difference find path = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end_path - begin_path).count() << "[ns]" << std::endl;

    std::cout<<"-----------------------begin traceback"<<std::endl;


    std::deque<Node*> planNodes;
    bool ok = false;

    //Measure time traceback
    std::chrono::steady_clock::time_point begin_traceback = std::chrono::steady_clock::now();
    cv::Mat path_MatCostmap = cv::Mat(_pixelsX_reduced, _pixelsY_reduced, CV_8UC1,cv::Scalar(255));

    if (nodes.size()==1)
    {
        Node* tmp = nodes[0];
        std::cout<<"Node (" << tmp->_id << "): " << tmp->_x << ", " << tmp->_y << " ," << tmp->_z << std::endl;
        planNodes.push_front(tmp);
        path_MatCostmap.at<uchar>(tmp->_x, tmp->_y) = 80;
        std::cout << "Path matcostmap   " << std::endl << path_MatCostmap << std::endl;

        goalParentId = nodes[0]->_parentId;
        if( goalParentId == 0)
        {
            ok = true;
            cv::imwrite("path.png",path_MatCostmap);
            std::cout<<"...........FINISH.................."<<std::endl;
        }
    }
    else if(nodes.size()>1)
    {
        while( ! ok )
        {
            ROS_INFO("=============================== roboticslab: GLOBAL PLANNER: traceback begin ");
            for(int nodeIdx=0; nodeIdx<nodes.size(); nodeIdx++)
            {
                if( nodes[nodeIdx]->_id == goalParentId )
                {
                    Node* tmp = nodes[nodeIdx];
                    std::cout<<"Node (" << tmp->_id << "): " << tmp->_x << ", " << tmp->_y << " ," << tmp->_z << std::endl;
                    if (layers_MatCostmap[tmp->_z].at<uchar>(tmp->_x,tmp->_y)==0)
                    {
                        planNodes.push_front(tmp);
                    }
                
                    path_MatCostmap.at<uchar>(tmp->_x, tmp->_y) = 80;
                    std::cout << "Path matcostmap   " << std::endl << path_MatCostmap << std::endl;

                    goalParentId = nodes[nodeIdx]->_parentId;
                    if( goalParentId == 0 || tmp->_id == 0)
                    {
                        ok = true;
                        cv::imwrite("path.png",path_MatCostmap);
                        std::cout<<"...........FINISH.................."<<std::endl;
                        break;

                    }
                }
            }
        }
    }
    else
    {return false;}
    
    

    //Measure time traceback
    std::chrono::steady_clock::time_point end_traceback = std::chrono::steady_clock::now();
    std::cout << "Time difference traceback = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end_traceback- begin_traceback).count() << "[ns]" << std::endl;
    
    ROS_INFO("=============================== roboticslab: GLOBAL PLANNER: traceback END ");


    std::cout<<"-----------------------end traceback"<<std::endl;
    std::cout<<"-----------------------begin planNodes"<<std::endl;

    plan.push_back(start);

    //Measure time plan
    std::chrono::steady_clock::time_point begin_plan = std::chrono::steady_clock::now();


    for(int i=0;i<planNodes.size();i++)
    {
        Node* tmp = planNodes[i];
        std::cout<<"Node (" << tmp->_id << "): " << tmp->_x << ", " << tmp->_y << " ," << tmp->_z <<  std::endl;

        //Position
        double goalX_meters = (tmp->_x) * (_resolution * _planningScale) + _originX;
        double goalY_meters = (tmp->_y) * (_resolution * _planningScale) + _originY;


        geometry_msgs::PoseStamped new_goal = goal;
        new_goal.pose.position.x = goalX_meters;
        new_goal.pose.position.y = goalY_meters;
        std::cout<<"New goal in meters: "<< new_goal.pose.position.x << ", "<< new_goal.pose.position.y <<std::endl;

        //Orientation
        geometry_msgs::Quaternion goal_quaternion;
        tf::Quaternion goal_tf;
        double goal_yaw = 0;

        std::cout << "Z : " << tmp->_z << std::endl;
        std::cout  << "Yaw before: " << goal_yaw << std::endl;     


        //int num_floors = 8;
       
        for (int i = 0; i < num_floors; i++){
            if (tmp->_z == num_floors){
                goal_yaw = M_PI - (2*i*M_PI)/num_floors;

            }
        }

        // if ( tmp->_z == 0 ){
        //     goal_yaw = 3.14;
        //     std::cout << "Floor: 0" << std::endl;
        // }else if ( tmp->_z == 1 ){
        //     goal_yaw = 2.35; 
        //     std::cout << "Floor: 1" << std::endl;
        // }else if( tmp->_z ==2 ){
        //     goal_yaw = 1.56;
        //     std::cout << "Floor: 2" << std::endl;
        // }else if ( tmp->_z == 3 ){
        //     goal_yaw = 0.78;
        //     std::cout << "Floor: 3" << std::endl;
        // }else if( tmp->_z == 4 ){
        //     goal_yaw = 0;
        //     std::cout << "Floor: 4" << std::endl;
        // }else if( tmp->_z == 5 ){
        //     goal_yaw = -0.78;
        //     std::cout << "Floor: 5" << std::endl;
        // }else if ( tmp->_z == 6 ){
        //     goal_yaw = -1.56;
        //     std::cout << "Floor: 6" << std::endl;
        // }else if( tmp->_z == 7 ){
        //     goal_yaw = -2.35;
        //     std::cout << "Floor: 7" << std::endl;
        // }

        std::cout  << "Yaw: " << goal_yaw << std::endl;
        goal_quaternion = tf::createQuaternionMsgFromYaw(goal_yaw);
        new_goal.pose.orientation =  goal_quaternion;   //geometry_msgs/Quaternion

        std::cout << "New goal orientation: (Yaw: " << goal_yaw << ") " << new_goal.pose.orientation << std::endl;

        ROS_INFO("=============================== roboticslab: GLOBAL PLANNER: new goal");
        plan.push_back(new_goal);

    }
    plan.push_back(goal);

    //Measure time plan
    std::chrono::steady_clock::time_point end_plan = std::chrono::steady_clock::now();
    std::cout << "Time difference plan = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end_plan- begin_plan).count() << "[ns]" << std::endl;


    std::cout<<"-----------------------end planNodes"<<std::endl;
    
    std::cout<<"=============================== GLOBAL PLANNER: makePlan end"<<std::endl;
    ROS_INFO("=============================== roboticslab: GLOBAL PLANNER: makePlan end");

    return true;
}



};     //namespace