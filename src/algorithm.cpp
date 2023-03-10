// After various trial and errors I came to the conclusion to use following control parameters:
// k_linear = 0.25; <-- this is faster than usual (usual for HÜ2.1 is 0.125) but for the application it works because no sharp turns will needed to be performed and it get's the robot faster to the endgoal
// k_linear is also > 0
// k_alpha = 0.8; <-- abs(k_alpha) needs to be bigger than abs(k_linear) see Siegwart: Introduction to mobile robots
// k_beta = -0.5; <-- beta is smaller than 0 see Siegwart: Introduction to mobile robots
//
#include <ros/ros.h>
#include <ros/duration.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/OccupancyGrid.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <typeinfo>
#include <cmath>
#include <thread>
// #include <tf/transform_listener.h>

// #include "dynamic_reconfigure/server.h"
// #include "eschweiler_hw3/linearControllerConfig.h"

struct Quaternion{
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

struct RobotPose_m {
    
    double x_m, y_m, th_rad;
};

struct RobotPos_px {
    
    double x_px, y_px;

    bool operator==(const RobotPos_px& other) const {
        return x_px == other.x_px && y_px == other.y_px;
    }
};

ros::Publisher publisher;
ros::Publisher pub_viz;
ros::Publisher pub_path;
visualization_msgs::MarkerArray marker_array;
// ros::Publisher pose_publisher;
ros::Subscriber subscriber_pose;
ros::Subscriber sub_map;
ros::Subscriber sub_goal;
ros::ServiceClient client;
nav_msgs::Odometry turtle_odom;
std_msgs::String pose_info;
geometry_msgs::Twist vel_msg;
RobotPose_m current_pose;
bool flag;

double PI = 3.1415926535897;
bool visualization_on = false;
ros::Time current_time, last_time;
bool checkbox_start_status_earlier = false;
bool checkbox_reset_status_earlier = false;
bool reset_is_set = false;
double resolution_m_per_px;
int counter_marker = 0;
nav_msgs::Path path_msg;

double goal_x;
double goal_y;
double goal_th;

//DECLARATION OF FUNCTIONS!
void moveStraightLine(double distance, bool moveForward, double velocity);
void moveStraightLineOdom(double distance, bool moveForward, double velocity);
void rotateByAngle(double angleDg, bool positiveRot, double angVelocityDg);
void pose_callback(const nav_msgs::Odometry::ConstPtr & pose_message);
void message_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void go_to_pose_via_algorithm(RobotPose_m goal);
void go_to_pos(RobotPose_m goal);
void visualizeVectorPositions(std::vector<RobotPos_px>* v);
std::vector<RobotPos_px> getAdjacentPixelIfNotCloseToWall(RobotPos_px initPx);
std::vector<RobotPos_px> check_if_path_is_found_yet(std::vector<std::vector<std::vector<RobotPos_px>>>* paths, RobotPos_px startingPos_px, RobotPos_px goal_px);
bool isPosInPointsVisited(RobotPos_px current_adj_px, std::vector<RobotPos_px>* points_visited);
bool areWallsAround(RobotPos_px initPx);
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr & pose_message);

RobotPos_px convertRobotPose_mToRobotPos_px(RobotPose_m pose_m);
RobotPose_m convertRobotPos_pxToRobotPose_m(RobotPos_px pos_px);
// void dynamic_callback(eschweiler_hw3::linearControllerConfig &config, uint32_t level);
double put_angle_in_range(double angle);
void go_to_goal(RobotPose_m goal);
void endMovement();
double rad2deg(double rad);
double deg2rad(double deg);
EulerAngles ToEulerAngles(Quaternion q);
std::vector<std::vector<int>> vector_map;
void setMarkerAtRobotPos_px(RobotPos_px pos_px, std::string color, bool permanent);
void visualize_path_finding();
void find_path(RobotPose_m);
bool map_received = false;

ros::Duration ten_seconds;

int main(int argc, char **argv)
{
    
    if (__cplusplus == 201703L) ROS_INFO_STREAM("C++17\n");
    else if (__cplusplus == 201402L) ROS_INFO_STREAM("C++14\n");
    else if (__cplusplus == 201103L) ROS_INFO_STREAM("C++11\n");
    else if (__cplusplus == 199711L) ROS_INFO_STREAM("C++98\n");
    else ROS_INFO_STREAM("pre-standard C++\n");
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "algorithm");
	ros::NodeHandle n;

    ros::Rate rate(10);

    int isUebung3_2, isUebung3_3;

    ros::param::get("~uebung3_2", isUebung3_2);
    ros::param::get("~uebung3_3", isUebung3_3);
    ros::param::get("~resolution", resolution_m_per_px);
    ros::param::get("~goal_x", goal_x);
    ros::param::get("~goal_y", goal_y);
    ros::param::get("~goal_th", goal_th);
    
    ROS_INFO_STREAM("isUebung3_2: " << isUebung3_2);
    ROS_INFO_STREAM("isUebung3_3: " << isUebung3_3);
    ROS_INFO_STREAM("resolution_m_per_px: " << resolution_m_per_px);
    ROS_INFO_STREAM("goal_x: " << goal_x);
    ROS_INFO_STREAM("goal_y: " << goal_y);
    ROS_INFO_STREAM("goal_th: " << goal_th);

    for (int i=0;i<30; i++){
        rate.sleep();
    }
    // dynamic_reconfigure::Server<eschweiler_hw3::linearControllerConfig> server;
    // dynamic_reconfigure::Server<eschweiler_hw3::linearControllerConfig>::CallbackType f;

    // f = boost::bind(&dynamic_callback, _1, _2);
    // server.setCallback(f);

    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Duration ten_seconds(0.5);

	publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    pub_viz = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 100000000);
    pub_path = n.advertise<nav_msgs::Path>("/breadth_first_path", 1000, true);
    client = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    subscriber_pose = n.subscribe("/odom", 10, pose_callback);
    sub_map = n.subscribe("/map", 10, message_callback);
    sub_goal = n.subscribe("/move_base_simple/goal", 10, goal_callback);

    // std::vector<int> vic = {1,2,3,4,5};

    // for(int i =0; i<100;i++)
    // {
    //     ros::spinOnce();
    //     ROS_INFO_STREAM("Sleeping to attach debugger... [" << i << "/200]");
    //     rate.sleep();
    // }

    int counter = 0;
    // while (map_received == false)
    while (pub_viz.getNumSubscribers() == 0 || map_received == false)
    {
        ros::spinOnce();
        ROS_INFO_STREAM("Wating for RVIZ to finish loading... [" << counter << "]");
        rate.sleep();
        counter += 1;
    }
    //Test your code here
    ROS_INFO_STREAM("\n\n\n******START TESTING************\n");    

    RobotPos_px px_1 = {200, 200};
    RobotPos_px px_2 = {0, 0};
    RobotPos_px px_3 = {200, 191};
    // RobotPose_m output_m1 = convertRobotPos_pxToRobotPose_m(px_1);
    // RobotPose_m output_m2 = convertRobotPos_pxToRobotPose_m(px_2);

    // ROS_INFO_STREAM("output_m1.x_m: " << output_m1.x_m << "   output_m1.y_m: " << output_m1.y_m);
    // ROS_INFO_STREAM("output_m2.x_m: " << output_m2.x_m << "   output_m2.y_m: " << output_m2.y_m);

    // RobotPose_m m_1 = {-0.001,-0.001,0};
    // RobotPose_m m_2 = {3, -4, 0};
    // RobotPos_px output_px1 = convertRobotPose_mToRobotPos_px(m_1);
    // RobotPos_px output_px2 = convertRobotPose_mToRobotPos_px(m_2);

    // ROS_INFO_STREAM("output_px1.x_px: " << output_px1.x_px << "   output_px1.y_px: " << output_px1.y_px);
    // ROS_INFO_STREAM("output_px2.x_px: " << output_px2.x_px << "   output_px2.y_px: " << output_px2.y_px);
    // std::vector<RobotPos_px> adj_px_1 = getAdjacentPixelIfNotCloseToWall(px_1);
    // std::vector<RobotPos_px> adj_px_2 = getAdjacentPixelIfNotCloseToWall(px_2);
    // std::vector<RobotPos_px> adj_px_3 = getAdjacentPixelIfNotCloseToWall(px_3);
    // visualizeVectorPositions(&adj_px_1);
    // visualizeVectorPositions(&adj_px_2);
    // visualizeVectorPositions(&adj_px_3);

    // setMarkerAtRobotPos_px(px_1);
    // setMarkerAtRobotPos_px(px_2);
    // setMarkerAtRobotPos_px(px_3);

    if (isUebung3_2){
        ROS_INFO_STREAM("Hausuebung3_2 will start now!!");
        RobotPose_m RobotPose_ms[4] = {{2,2,PI/2}, {2,4,PI/2}, {0,6,PI}, {-2,4,PI*3/2}};
        ros::Rate loop_rate(10);
        
        for (int i=0; i<3; i++){
            for (int j=0; j<4; j++){
                go_to_goal(RobotPose_ms[j]);
            }
        }
    } else if(isUebung3_3){
        //CALLIBRATION
        moveStraightLine(1.5, true, 2);
        rotateByAngle(90, true, 45);
        moveStraightLine(1.5, true, 2);
        rotateByAngle(90, true, 45);
        moveStraightLine(1.5, true, 2);
        rotateByAngle(90, true, 45);
        moveStraightLine(1.5, true, 2);
        ROS_INFO_STREAM("Hausuebung3_3 will start now!!");
    }
    //go_to_goal(xi_1);
    //
    ROS_INFO("\n\n\n******END OF Hausuebung3***********\n");   
    //ros::shutdown();

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

   return 0;
}

void find_path(RobotPose_m goal) {
    flag = false;
    go_to_pose_via_algorithm(goal);
    flag = true;
}

void visualize_path_finding() {
    while(!flag){
        ros::Rate rate(5);
        rate.sleep();
        pub_viz.publish(marker_array);
        pub_path.publish(path_msg);
    }
    marker_array.markers.clear();
    path_msg.poses.clear();
}

// void dynamic_callback(eschweiler_hw3::linearControllerConfig &config, uint32_t level) {
//     ROS_INFO_STREAM("Reconfigure Request: \n" << "\tk_linear: " << config.k_linear << std::endl <<
//             "\tk_alpha: " << config.k_alpha << std::endl <<
//             "\tk_beta: " << config.k_beta << std::endl <<
//             "\tx_goal: " << config.x_goal << std::endl <<
//             "\ty_goal: " << config.y_goal << std::endl <<
//             "\tth_goal: " << config.th_goal << std::endl <<
//             "\tstart_on_check: " << config.start_on_check << std::endl << std::endl);
//     k_alpha = config.k_alpha;
//     k_beta = config.k_beta;
//     k_linear = config.k_linear;
//     double th_goal = deg2rad(config.th_goal);
//     if (checkbox_start_status_earlier == false && config.start_on_check == true){
//         RobotPose_m rp;
//         rp.x = config.x_goal;
//         rp.y = config.y_goal;
//         rp.th = th_goal;

//         go_to_goal(rp);
//     }
//     if (checkbox_reset_status_earlier == false && config.reset_on_check == true){
//         ROS_INFO("SERVICE IS CALLED TO RESET GAZEBO SIMULATION!!!");
//         std_srvs::Empty srv;
//         //send service
//         if (client.call(srv)){

//             ROS_INFO("Returned and reset!");

//         }else {

//             ROS_INFO("Return failed!");Fros
//         }
//     }
//     checkbox_start_status_earlier = config.start_on_check;
//     checkbox_reset_status_earlier = config.reset_on_check;
// }

void message_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // Print the message to the console
    ROS_INFO_STREAM("Amount of pixels: " << msg->data.size());
    ROS_INFO_STREAM("Pixel at (1,1): " << (int)(msg->data[1]));
    for (int i=0;i<msg->info.height;i++){
        std::vector<int> line_vec;
        for (int j=0; j<msg->info.width;j++){
            // ROS_INFO_STREAM("i: " << i << "   j: " << j << "   i*msg->info.width + j: " << i*msg->info.width + j);
            int cellstate = (int)msg->data[i*msg->info.width + j]; //This extracts the cellvalue from occupancy grid message
            line_vec.push_back(cellstate);
        }
        vector_map.push_back(line_vec);
    }    
    ROS_INFO_STREAM("Amount of pixels: " << msg->data.size());
    ROS_INFO_STREAM("vector size: x: " << vector_map[0].size() << " y: " << vector_map.size());

    // THIS IS FOR TESTING PURPOSES, ONLY UNCOMMENT IF YOU WANT TO CHECK WHAT VALUES THE VECTOR STORES!!!
    // for (int i=0;i<vector_map.size(); i++){
    //     for (int j=0; j<vector_map[i].size(); j++){
    //         ROS_INFO_STREAM("Vector value at [i:" << i << ",j:" << j << "]: " << vector_map[i][j]);
    //     }
    // }

    ROS_INFO_STREAM("Map received finished! ");
    map_received = true;
}

void pose_callback(const nav_msgs::Odometry::ConstPtr & pose_message){
    // ros::spinOnce();
    Quaternion q ;

    current_pose.x_m = pose_message->pose.pose.position.x;
    current_pose.y_m = pose_message->pose.pose.position.y;

    q.w = pose_message->pose.pose.orientation.w;
    q.x = pose_message->pose.pose.orientation.x;
    q.y = pose_message->pose.pose.orientation.y;
    q.z = pose_message->pose.pose.orientation.z;

    EulerAngles angles = ToEulerAngles(q);
    current_pose.th_rad = angles.yaw;

    // std::string s;
    // std::stringstream sstm;
    // sstm << "x=" << current_pose.x_m << ", y=" << current_pose.y_m << ", theta=" << current_pose.th_rad << "\n";
    // pose_info.data = sstm.str();
	// pose_publisher.publish(pose_info);    
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr & pose_message){
    // ros::spinOnce();
    Quaternion q ;
    RobotPose_m goal;
    
    ROS_INFO_STREAM("RECEIVED GOAL!");

    goal.x_m = pose_message->pose.position.x;
    goal.y_m = pose_message->pose.position.y;

    q.w = pose_message->pose.orientation.w;
    q.x = pose_message->pose.orientation.x;
    q.y = pose_message->pose.orientation.y;
    q.z = pose_message->pose.orientation.z;

    EulerAngles angles = ToEulerAngles(q);
    goal.th_rad = angles.yaw;
    
    std::thread t1(find_path, goal);
    std::thread t2(visualize_path_finding);

    // Wait for the threads to complete
    t1.join();
    t2.join();

}

void go_to_pose_via_algorithm(RobotPose_m goal){
    RobotPose_m currentPose_C_Odom = {current_pose.x_m, current_pose.y_m, current_pose.th_rad};
    RobotPos_px startingPos_px = convertRobotPose_mToRobotPos_px(currentPose_C_Odom);
    RobotPos_px goal_px = convertRobotPose_mToRobotPos_px(goal);

    ROS_INFO_STREAM("startingPos_px.x_px: " << startingPos_px.x_px << "    startingPos_px.y_px: " << startingPos_px.y_px );
    ROS_INFO_STREAM("goal_px.x_px: " << goal_px.x_px << "    goal_px.y_px: " << goal_px.y_px );

    std::vector<RobotPos_px> points_visited;
    std::vector<std::vector<std::vector<RobotPos_px>>> paths; 
    RobotPos_px current_last_px_of_string, current_adj_px;
    std::vector<RobotPos_px> current_adj_px_vec, copy_of_current_string, new_string;
    //First vector stores the level: level two would be AB, level 4 would be ABCD
    //Second vector stores all the strings like for l3 for example: ABC, ABD, ACE, ACF, ...
    //Third vector stores all the connected RobotPos's

    std::vector<std::vector<RobotPos_px>> level1; 
    std::vector<RobotPos_px> starting_point = {startingPos_px};
    level1.push_back(starting_point);
    paths.push_back(level1);
    points_visited.push_back(startingPos_px);

    while(check_if_path_is_found_yet(&paths, startingPos_px, goal_px).empty()){//continue searching, as long as vector is empty
        std::vector<std::vector<RobotPos_px>> next_level;
        int highest_level = paths.size() - 1; //-1 to determine the index of the highest level
        ROS_INFO_STREAM("Highest level:" << highest_level);
        for(int i=0;i<paths[highest_level].size(); i++){
            current_last_px_of_string = paths[highest_level][i][highest_level];
            current_adj_px_vec = getAdjacentPixelIfNotCloseToWall(current_last_px_of_string );
            // ROS_INFO_STREAM("current_last_px_of_string: (x_px: " << current_last_px_of_string.x_px << ", y_px: " << current_last_px_of_string.y_px << ")");
            for (int j=0;j< current_adj_px_vec.size(); j++){
                current_adj_px = current_adj_px_vec[j];
                if(isPosInPointsVisited(current_adj_px, &points_visited)){
                    // ROS_INFO_STREAM("This point was already visited! (x_px: " << current_adj_px.x_px << ", y_px: " << current_adj_px.y_px << ")");
                    // continue;setMarkerAtRobotPos_px
                    if (visualization_on){
                        setMarkerAtRobotPos_px(current_adj_px, "red", false);
                    }
                }else{
                    if (visualization_on){
                        setMarkerAtRobotPos_px(current_adj_px, "green", false);
                    }
                    // ROS_INFO_STREAM("This point will be added! (x_px: " << current_adj_px.x_px << ", y_px: " << current_adj_px.y_px << ")");
                    new_string = paths[highest_level][i];
                    new_string.push_back(current_adj_px);
                    next_level.push_back(new_string);
                    points_visited.push_back(current_adj_px);
                }
            }
        }
        paths.push_back(next_level);
        ROS_INFO_STREAM("Next Level was created!!");
    }
    std::vector<RobotPos_px> path_to_goal = check_if_path_is_found_yet(&paths, startingPos_px, goal_px);
    visualizeVectorPositions(&path_to_goal);
    path_msg.header.frame_id = "odom";
    path_msg.header.stamp = ros::Time::now();
    for (int i=0; i<path_to_goal.size(); i++){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "odom";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = convertRobotPos_pxToRobotPose_m(path_to_goal[i]).x_m;
        pose.pose.position.y = convertRobotPos_pxToRobotPose_m(path_to_goal[i]).y_m;
        pose.pose.position.z = 0;
        pose.pose.orientation.w = 1;
        pose.header.seq = i;
        path_msg.poses.push_back(pose);
        
    }

    for (int i=0; i<path_to_goal.size(); i++){
        go_to_pos(convertRobotPos_pxToRobotPose_m(path_to_goal[i]));
    }
    endMovement();
    marker_array.markers.clear();
    path_msg.poses.clear();
    path_to_goal.clear();
    ros::Duration(1).sleep();
    ROS_INFO_STREAM("REACHED GOAL; CONGRATULATION LITTLE ROBOT");
    ROS_INFO_STREAM("REACHED GOAL; CONGRATULATION LITTLE ROBOT");
    ROS_INFO_STREAM("REACHED GOAL; CONGRATULATION LITTLE ROBOT");
}

bool isPosInPointsVisited(RobotPos_px current_adj_px, std::vector<RobotPos_px>* points_visited){
    for (int i=0;i<(*points_visited).size();i++){
        RobotPos_px p = (*points_visited)[i];
        if (current_adj_px == p){
            return true;
        }
    }
    return false;
}

std::vector<RobotPos_px> check_if_path_is_found_yet(std::vector<std::vector<std::vector<RobotPos_px>>>* paths, RobotPos_px startingPos_px, RobotPos_px goal_px){
    int highest_level = paths->size()-1;
    ROS_INFO_STREAM("paths[highest_level].size(): " << (*paths)[highest_level].size());
    for (int i=0; i<(*paths)[highest_level].size(); i++){
        int startingPos_amount = std::count((*paths)[highest_level][i].begin(), (*paths)[highest_level][i].end(), startingPos_px);
        int endingPos_amount = std::count((*paths)[highest_level][i].begin(), (*paths)[highest_level][i].end(), goal_px);
        if (startingPos_amount > 0 && endingPos_amount > 0){
            ROS_INFO_STREAM("Path found!");
            return (*paths)[highest_level][i];
        }
    }
    std::vector<RobotPos_px> null_vector;
    null_vector.clear();
    return null_vector;
}

int get_occupancy_state_at_pos(int x, int y){
    // ROS_INFO_STREAM("get_occupancy_state_at_pos started! x: " << x << "y: " << y);
    // ROS_INFO_STREAM("vector_size: x: " << vector_map[y].size() << "y: " << vector_map.size());

    int state = vector_map[y][x];
    // ROS_INFO_STREAM("state: " << state);
    return state;
}

void go_to_goal(RobotPose_m goal){
    

    double k_linear = 0.25;
    double k_alpha = 1.2;
    // double_alpha_fein = 0.4;
    double k_beta = -0.5;
    ros::Rate loop_rate(10);

    ROS_INFO_STREAM("START VALUES START: " );
    ROS_INFO_STREAM("start_x=" << current_pose.x_m << ", start_y=" << current_pose.y_m << ", start_th=" << current_pose.th_rad);

    double e_distance = abs( sqrt( pow( current_pose.x_m - goal.x_m, 2 ) + pow( current_pose.y_m - goal.y_m, 2 ) ) );
    double e_angle_alpha = put_angle_in_range(atan2((goal.y_m - current_pose.y_m), (goal.x_m - current_pose.x_m)) - current_pose.th_rad);
    double e_angle_beta = put_angle_in_range( goal.th_rad -current_pose.th_rad -e_angle_alpha );


    ROS_INFO_STREAM("e_distance[" << e_distance << "] = abs( sqrt( pow( current_pose.x_m[" << current_pose.x_m << "] - goal.x_m[ " << goal.x_m << 
    "], 2 ) + pow( current_pose.y_m[" << current_pose.y_m << "] - goal.y_m[" << goal.y_m << "], 2 ) ) );");
    ROS_INFO_STREAM("e_alpha[" << e_angle_alpha << "] = atan2((goal.y_m[" << goal.y_m << "] - current_pose.y_m[" << current_pose.y_m << "]), (goal.x_m[" << goal.x_m << "] - current_pose.x_m[" 
    << current_pose.x_m << "])) - current_pose.th_rad[" << current_pose.th_rad << "]");
    ROS_INFO_STREAM("e_angle_beta[" << e_angle_beta << "] = goal_th[" << goal.th_rad << "] - current_pose.th_rad[" << current_pose.th_rad << "] - e_angle_alpha[" << e_angle_alpha << "]");
    ROS_INFO_STREAM("START VALUES END" << std::endl);
    // ros::Rate loop_rate(10);
    sleep(1);


    //while (e_distance > 0.05){
    ROS_INFO_STREAM ("e_angle_alpha=" << abs(e_angle_alpha) << "> deg2rad(5)=" << deg2rad(5) );

    // while (abs(e_angle_alpha) > deg2rad(2)){
    while (e_distance > 0.02 ){
    // while (e_distance > 0.04 || abs(e_angle_alpha) > deg2rad(5) || abs(e_angle_beta) > deg2rad(5)){
        e_distance = abs( sqrt( pow( current_pose.x_m - goal.x_m, 2 ) + pow( current_pose.y_m - goal.y_m, 2 ) ) );
        e_angle_alpha = put_angle_in_range(atan2((goal.y_m - current_pose.y_m), (goal.x_m - current_pose.x_m)) - current_pose.th_rad);
        e_angle_beta = put_angle_in_range( goal.th_rad - current_pose.th_rad -e_angle_alpha);
        vel_msg.linear.x = k_linear * e_distance;
        // vel_msg.angular.z = k_alpha * e_angle_alpha;
        // vel_msg.angular.z = k_beta * e_angle_beta;
        vel_msg.angular.z = k_alpha * e_angle_alpha + k_beta * e_angle_beta;

        ROS_INFO_STREAM("e_distance[" << e_distance << "] = abs( sqrt( pow( current_pose.x_m[" << current_pose.x_m << "] - goal.x_m[ " << goal.x_m << 
        "], 2 ) + pow( current_pose.y_m[" << current_pose.y_m << "] - goal.y_m[" << goal.y_m << "], 2 ) ) );");
        ROS_INFO_STREAM("e_alpha[" << e_angle_alpha << "] = atan2((goal.y_m[" << goal.y_m << "] - current_pose.y_m[" << current_pose.y_m << "]), (goal.x_m[" << goal.x_m << "] - current_pose.x_m[" 
        << current_pose.x_m << "])) - current_pose.th_rad[" << current_pose.th_rad << "]");
        ROS_INFO_STREAM("e_angle_beta[" << e_angle_beta << "] = goal.th_rad[" << goal.th_rad << "] - current_pose.th_rad[" << current_pose.th_rad << "] - e_angle_alpha[" << e_angle_alpha << "]");
        
        ROS_INFO_STREAM("vel_msg.linear.x =" << vel_msg.linear.x);
        ROS_INFO_STREAM("vel_msg.angular.z =" << vel_msg.angular.z << std::endl);
        publisher.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    endMovement();
}

void go_to_pos(RobotPose_m goal){
    double k_linear = 0.25*5;
    double k_alpha_grob = 2;
    double k_alpha_fein = 0.6;
    double k_beta = -0.5;
    ros::Rate rate(10);
    ROS_INFO_STREAM("START VALUES START: " );
    ROS_INFO_STREAM("start_x=" << current_pose.x_m << ", start_y=" << current_pose.y_m << ", start_th=" << current_pose.th_rad);
    ROS_INFO_STREAM("goal_x=" << goal.x_m << ", goal_y=" << goal.y_m << ", goal_th=" << goal.th_rad);

    double e_distance = abs( sqrt( pow( current_pose.x_m - goal.x_m, 2 ) + pow( current_pose.y_m - goal.y_m, 2 ) ) );
    double e_angle_alpha = put_angle_in_range(atan2((goal.y_m - current_pose.y_m), (goal.x_m - current_pose.x_m)) - current_pose.th_rad);

    // ROS_INFO_STREAM("e_distance[" << e_distance << "] = abs( sqrt( pow( current_pose.x_m[" << current_pose.x_m << "] - goal.x_m[ " << goal.x_m << 
    // "], 2 ) + pow( current_pose.y_m[" << current_pose.y_m << "] - goal.y_m[" << goal.y_m << "], 2 ) ) );");
    // ROS_INFO_STREAM("e_alpha[" << e_angle_alpha << "] = atan2((goal.y_m[" << goal.y_m << "] - current_pose.y_m[" << current_pose.y_m << "]), (goal.x_m[" << goal.x_m << "] - current_pose.x_m[" 
    // << current_pose.x_m << "])) - current_pose.th_rad[" << current_pose.th_rad << "]");
    // ROS_INFO_STREAM("START VALUES END" << std::endl);

    while(abs(e_angle_alpha) > deg2rad(7)){//start p-controller when e_alpha has less than 4 deg 
        e_angle_alpha = put_angle_in_range(atan2((goal.y_m - current_pose.y_m), (goal.x_m - current_pose.x_m)) - current_pose.th_rad);
        vel_msg.angular.z = k_alpha_grob * e_angle_alpha;
        publisher.publish(vel_msg);
        ros::spinOnce();
        rate.sleep();

    }

    while (e_distance > 0.015 ){
        // while (e_distance > 0.04 || abs(e_angle_alpha) > deg2rad(5) || abs(e_angle_beta) > deg2rad(5)){
            e_distance = abs( sqrt( pow( current_pose.x_m - goal.x_m, 2 ) + pow( current_pose.y_m - goal.y_m, 2 ) ) );
            e_angle_alpha = put_angle_in_range(atan2((goal.y_m - current_pose.y_m), (goal.x_m - current_pose.x_m)) - current_pose.th_rad);
            vel_msg.linear.x = k_linear * e_distance;
            // vel_msg.angular.z = k_alpha * e_angle_alpha;
            // vel_msg.angular.z = k_beta * e_angle_beta;
            vel_msg.angular.z = k_alpha_fein * e_angle_alpha;// + k_beta * e_angle_beta;

            // ROS_INFO_STREAM("e_distance[" << e_distance << "] = abs( sqrt( pow( current_pose.x_m[" << current_pose.x_m << "] - goal.x_m[ " << goal.x_m << 
            // "], 2 ) + pow( current_pose.y_m[" << current_pose.y_m << "] - goal.y_m[" << goal.y_m << "], 2 ) ) );");
            // ROS_INFO_STREAM("e_alpha[" << e_angle_alpha << "] = atan2((goal.y_m[" << goal.y_m << "] - current_pose.y_m[" << current_pose.y_m << "]), (goal.x_m[" << goal.x_m << "] - current_pose.x_m[" 
            // << current_pose.x_m << "])) - current_pose.th_rad[" << current_pose.th_rad << "]");
            
            // ROS_INFO_STREAM("vel_msg.linear.x =" << vel_msg.linear.x);
            // ROS_INFO_STREAM("vel_msg.angular.z =" << vel_msg.angular.z << std::endl);
            publisher.publish(vel_msg);
            ros::spinOnce();
            rate.sleep();
        }
        endMovement();
        ROS_INFO_STREAM("REACHED GOAL VALUES START: " );
        ROS_INFO_STREAM("current_x=" << current_pose.x_m << ", current_y=" << current_pose.y_m << ", current_th=" << current_pose.th_rad);
        ROS_INFO_STREAM("goal_x=" << goal.x_m << ", goal_y=" << goal.y_m << ", goal_th=" << goal.th_rad << std::endl << std::endl);

}


void endMovement(){
    ROS_INFO_STREAM("Movement finished, now it will be set to 0!" << std::endl << std::endl);

    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    publisher.publish(vel_msg);
    ros::spinOnce();
}

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

double rad2deg(double rad){
    return rad * 180 / PI;
}

double deg2rad(double deg){
    return deg * PI / 180;
}

double put_angle_in_range(double angle){
    if (angle > PI){
        // ROS_INFO_STREAM("angle > PI: angle=" << angle);
        return angle - 2 * PI;
    }else if (angle < -PI){
        // ROS_INFO_STREAM("angle < -PI: angle=" << angle);
        return angle + 2 * PI;
    }else {
        return angle;
    }
}

RobotPos_px convertRobotPose_mToRobotPos_px(RobotPose_m pose_m){
    RobotPos_px pos_px;
    pos_px.x_px = std::round((pose_m.x_m + 20 - (0.5 * resolution_m_per_px))/resolution_m_per_px); //add 20 because map is 40x40m and origin is in the middle
    pos_px.y_px = std::round((pose_m.y_m + 20 - (0.5 * resolution_m_per_px))/resolution_m_per_px); //add 20 because map is 40x40m and origin is in the middle
    
    return pos_px;
}

RobotPose_m convertRobotPos_pxToRobotPose_m(RobotPos_px pos_px){
    RobotPose_m pos_m;
    pos_m.x_m = ((pos_px.x_px * resolution_m_per_px) - 20) + (0.5 * resolution_m_per_px); //0.5 * resolution for middle point of pixel
    pos_m.y_m = ((pos_px.y_px * resolution_m_per_px) - 20) + (0.5 * resolution_m_per_px);
    pos_m.th_rad = 0;
    return pos_m;
}

void setMarkerAtRobotPos_px(RobotPos_px pos_px, std::string color, bool permanent){
    // Set up the marker message
    RobotPose_m pos_m = convertRobotPos_pxToRobotPose_m(pos_px);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.ns = counter_marker;
    marker.id = counter_marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos_m.x_m;  // Set the x-coordinate of the pixel
    marker.pose.position.y = pos_m.y_m;  // Set the y-coordinate of the pixel
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution_m_per_px;  // Set the size of the marker
    marker.scale.y = resolution_m_per_px;
    marker.scale.z = 0.001;
    if (color=="yellow"){
        marker.color.a = 0.8;  // Set the alpha (transparency) value
        marker.color.r = 1.0;  // Set the red color value
        marker.color.g = 1.0;  // Set the green color value
        marker.color.b = 0.0;  // Set the blue color valueCopy code
    }else if (color=="red"){
        marker.color.a = 0.1;  // Set the alpha (transparency) value
        marker.color.r = 1.0;  // Set the red color value
        marker.color.g = 0.0;  // Set the green color value
        marker.color.b = 0.0;  // Set the blue color valueCopy code
    }else if (color=="green"){
        marker.color.a = 0.1;  // Set the alpha (transparency) value
        marker.color.r = 0.0;  // Set the red color value
        marker.color.g = 1.0;  // Set the green color value
        marker.color.b = 0.0;  // Set the blue color valueCopy code
    }
    if (!permanent){
        marker.lifetime = ros::Duration(0.1);
    }else{
        marker.lifetime = ros::Duration(0);
    }
    marker_array.markers.push_back(marker);
    // ROS_INFO_STREAM("marker_array.markers.size(): " << marker_array.markers.size() );
    // while (ros::serialization::serializationLes

    if (marker_array.markers.size() > 300){
        marker_array.markers.clear();
    }

    uint32_t marker_array_size = ros::serialization::serializationLength(marker_array);
    // ROS_INFO_STREAM("Size of MarkerArray message: " << marker_array_size << " bytes");

    // pub_viz.publish(marker_array);markermarker
    counter_marker += 1;
}

std::vector<RobotPos_px> getAdjacentPixelIfNotCloseToWall(RobotPos_px initPx){//since cpp v11 you can return by value, without any performance issues
    // ROS_INFO_STREAM("getAdjacentPixelIfNotCloseToWall started!");
    std::vector<RobotPos_px> adj_px;
    // ROS_INFO_STREAM("get_occupancy_state_at_pos init: " << get_occupancy_state_at_pos(initPx.x_px, initPx.y_px));
    // ROS_INFO_STREAM("vector_map[0].size(): " << vector_map[0].size() << ", vector_map.size()" << vector_map.size());
    bool areWallsAround_var = areWallsAround(initPx);
    if( (initPx.x_px+1) < vector_map[0].size() && get_occupancy_state_at_pos(initPx.x_px+1, initPx.y_px) == 0
    && areWallsAround(RobotPos_px{initPx.x_px+1, initPx.y_px})){
        adj_px.push_back(RobotPos_px{initPx.x_px+1, initPx.y_px});
        // ROS_INFO_STREAM("Added Pixel to the right");
    }
    if( (initPx.x_px-1) >= 0 && get_occupancy_state_at_pos(initPx.x_px-1, initPx.y_px) == 0
    && areWallsAround(RobotPos_px{initPx.x_px-1, initPx.y_px})){
        adj_px.push_back(RobotPos_px{initPx.x_px-1, initPx.y_px});
        // ROS_INFO_STREAM("Added Pixel to the left");
    }
    if( (initPx.y_px+1) < vector_map.size() && get_occupancy_state_at_pos(initPx.x_px, initPx.y_px+1) == 0
    && areWallsAround(RobotPos_px{initPx.x_px, initPx.y_px+1})){
        adj_px.push_back(RobotPos_px{initPx.x_px, initPx.y_px+1});
        // ROS_INFO_STREAM("Added Pixel to the top");
    }
    if( (initPx.y_px-1) >= 0 && get_occupancy_state_at_pos(initPx.x_px, initPx.y_px-1) == 0
    && areWallsAround(RobotPos_px{initPx.x_px, initPx.y_px-1})){
        adj_px.push_back(RobotPos_px{initPx.x_px, initPx.y_px-1});
        // ROS_INFO_STREAM("Added Pixel to the bottom");
    }
    // //DIAGONAL PIXEL
    // if( (initPx.y_px-1) >= 0 && get_occupancy_state_at_pos(initPx.x_px-1, initPx.y_px-1) == 0
    // && areWallsAround(RobotPos_px{initPx.x_px-1, initPx.y_px-1})){
    //     adj_px.push_back(RobotPos_px{initPx.x_px-1, initPx.y_px-1});
    //     // ROS_INFO_STREAM("Added Pixel to the bottom");
    // }
    // if( (initPx.y_px-1) >= 0 && get_occupancy_state_at_pos(initPx.x_px+1, initPx.y_px-1) == 0
    // && areWallsAround(RobotPos_px{initPx.x_px+1, initPx.y_px-1})){
    //     adj_px.push_back(RobotPos_px{initPx.x_px+1, initPx.y_px-1});
    //     // ROS_INFO_STREAM("Added Pixel to the bottom");
    // }
    // if( (initPx.y_px-1) >= 0 && get_occupancy_state_at_pos(initPx.x_px-1, initPx.y_px+1) == 0
    // && areWallsAround(RobotPos_px{initPx.x_px-1, initPx.y_px+1})){
    //     adj_px.push_back(RobotPos_px{initPx.x_px-1, initPx.y_px+1});
    //     // ROS_INFO_STREAM("Added Pixel to the bottom");
    // }
    // if( (initPx.y_px-1) >= 0 && get_occupancy_state_at_pos(initPx.x_px+1, initPx.y_px+1) == 0
    // && areWallsAround(RobotPos_px{initPx.x_px+1, initPx.y_px+1})){
    //     adj_px.push_back(RobotPos_px{initPx.x_px+1, initPx.y_px+1});
    //     // ROS_INFO_STREAM("Added Pixel to the bottom");
    // }

    
    // ROS_INFO_STREAM("getAdjacentPixelIfNotCloseToWall ended!");
    return adj_px;
}

void visualizeVectorPositions(std::vector<RobotPos_px>* v){
    marker_array.markers.clear();
    ROS_INFO_STREAM("visualizeVectorPositions started!");
    for(int i=0; i<v->size();i++){
        setMarkerAtRobotPos_px((*v)[i], "yellow", true);
        ROS_INFO_STREAM("Pixel[" << i << "]: x->" << (*v)[i].x_px << ", y->" << (*v)[i].y_px);
    }
}

bool operator==(const RobotPos_px& lhs, const RobotPos_px& rhs) {
  return lhs.x_px == rhs.x_px && lhs.y_px == rhs.y_px;
}

bool areWallsAround(RobotPos_px pos){
    if (
        get_occupancy_state_at_pos(pos.x_px, pos.y_px+1) == 0 &&
        get_occupancy_state_at_pos(pos.x_px, pos.y_px+2) == 0 &&
        get_occupancy_state_at_pos(pos.x_px, pos.y_px+3) == 0 &&
        get_occupancy_state_at_pos(pos.x_px, pos.y_px+4) == 0 &&
        get_occupancy_state_at_pos(pos.x_px, pos.y_px+5) == 0 &&

        get_occupancy_state_at_pos(pos.x_px, pos.y_px-1) == 0 &&
        get_occupancy_state_at_pos(pos.x_px, pos.y_px-2) == 0 &&
        get_occupancy_state_at_pos(pos.x_px, pos.y_px-3) == 0 &&
        get_occupancy_state_at_pos(pos.x_px, pos.y_px-4) == 0 &&
        get_occupancy_state_at_pos(pos.x_px, pos.y_px-5) == 0 &&

        get_occupancy_state_at_pos(pos.x_px+1, pos.y_px) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+2, pos.y_px) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+3, pos.y_px) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+4, pos.y_px) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+5, pos.y_px) == 0 &&

        get_occupancy_state_at_pos(pos.x_px-1, pos.y_px) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-2, pos.y_px) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-3, pos.y_px) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-4, pos.y_px) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-5, pos.y_px) == 0 &&

        get_occupancy_state_at_pos(pos.x_px-5, pos.y_px+1) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-5, pos.y_px-1) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-5, pos.y_px+2) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-5, pos.y_px-2) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-5, pos.y_px+3) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-5, pos.y_px-3) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-5, pos.y_px+4) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-5, pos.y_px-4) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-5, pos.y_px+5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-5, pos.y_px-5) == 0 &&

        get_occupancy_state_at_pos(pos.x_px+1, pos.y_px+5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-1, pos.y_px+5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+2, pos.y_px+5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-2, pos.y_px+5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+3, pos.y_px+5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-3, pos.y_px+5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+4, pos.y_px+5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-4, pos.y_px+5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+5, pos.y_px+5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-5, pos.y_px+5) == 0 &&

        get_occupancy_state_at_pos(pos.x_px+5, pos.y_px+1) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+5, pos.y_px-1) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+5, pos.y_px+2) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+5, pos.y_px-2) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+5, pos.y_px+3) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+5, pos.y_px-3) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+5, pos.y_px+4) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+5, pos.y_px-4) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+5, pos.y_px+5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+5, pos.y_px-5) == 0 &&

        get_occupancy_state_at_pos(pos.x_px+1, pos.y_px-5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-1, pos.y_px-5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+2, pos.y_px-5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-2, pos.y_px-5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+3, pos.y_px-5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-3, pos.y_px-5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+4, pos.y_px-5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-4, pos.y_px-5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px+5, pos.y_px-5) == 0 &&
        get_occupancy_state_at_pos(pos.x_px-5, pos.y_px-5) == 0    

        // get_occupancy_state_at_pos(pos.x_px-4, pos.y_px+3) == 0 &&
        // get_occupancy_state_at_pos(pos.x_px-4, pos.y_px-3) == 0 &&
        // get_occupancy_state_at_pos(pos.x_px-3, pos.y_px+4) == 0 &&
        // get_occupancy_state_at_pos(pos.x_px-3, pos.y_px-4) == 0 &&    

        // get_occupancy_state_at_pos(pos.x_px+4, pos.y_px+3) == 0 &&
        // get_occupancy_state_at_pos(pos.x_px+4, pos.y_px-3) == 0 &&
        // get_occupancy_state_at_pos(pos.x_px+3, pos.y_px+4) == 0 &&
        // get_occupancy_state_at_pos(pos.x_px+3, pos.y_px-4) == 0 

    ){
        return true;
    }else{
        return false;
    }
}
void moveStraightLine(double distance, bool moveForward, double velocity){

    ROS_INFO_STREAM("Move in straight line!");
    double distanceMoved = 0;


    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate loop_rate(10);

    if (moveForward){
        vel_msg.linear.x = velocity;
    }else {
        vel_msg.linear.x = -velocity;
    }

    while (distanceMoved < distance){
        current_time = ros::Time::now();
        publisher.publish(vel_msg);
        ROS_INFO_STREAM("Distance moved: " << distanceMoved);
        ROS_INFO_STREAM("[Talker] I published a TwistMessage: \n" << vel_msg);
        distanceMoved += velocity * (current_time - last_time).toSec();
        ROS_INFO_STREAM("Distance moved (" << distanceMoved << ")= velocity (" << velocity << ") * (t1 (" << current_time.toSec() << ")-t0 (" << last_time.toSec() << "))(" << (current_time - last_time) << ")");
        ROS_INFO_STREAM("TIME is at the moment: " << ros::Time::now().toSec());
        
        ros::spinOnce();
        loop_rate.sleep();
        last_time = current_time;
    }
    ROS_INFO_STREAM("TIME is at the moment: " << ros::Time::now().toSec());
    ROS_INFO_STREAM("Distance moved: " << distanceMoved);
    endMovement();
}

void rotateByAngle(double angleDg, bool positiveRot, double angVelocityDg){
    double angle = angleDg * PI / 180;
    double angVelocity = angVelocityDg * PI / 180;
    ROS_INFO_STREAM("Move in straight line!");
    geometry_msgs::Twist vel_msg;
    double angleRotated = 0;
    double t0 = ros::Time::now().toSec();
    ros::Rate loop_rate(10);

    if (positiveRot){
        vel_msg.angular.z = angVelocity;
    }else {
        vel_msg.angular.z = -angVelocity;
    }

    while (angleRotated < angle){
        ROS_INFO_STREAM("[Talker] I published a TwistMessage: \n" << vel_msg);
        publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        angleRotated = angVelocity * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    }
    endMovement();
}
