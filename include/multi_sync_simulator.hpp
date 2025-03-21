#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>

#include <param.hpp>
#include <mission.hpp>
#include <util.hpp>
#include <agent_manager.hpp>
#include <obstacle_generator.hpp>

#include <utility>
#include <fstream>
#include <istream>

#include <mapf/pibt.hpp>

namespace MATP {
    class MultiSyncSimulator {
    public:
        MultiSyncSimulator(const ros::NodeHandle& _nh, const Param& _param, const Mission& _mission);

        void run();

    private:
        ros::NodeHandle nh;
        ros::Publisher pub_agent_trajectories;
        ros::Publisher pub_obstacle_trajectories;
        ros::Publisher pub_collision_model;
        ros::Publisher pub_start_goal_points_vis;
//        ros::Publisher pub_goal_positions_raw;
        ros::Publisher pub_agent_velocities_x;
        ros::Publisher pub_agent_velocities_y;
        ros::Publisher pub_agent_velocities_z;
        ros::Publisher pub_agent_accelerations_x;
        ros::Publisher pub_agent_accelerations_y;
        ros::Publisher pub_agent_accelerations_z;
        ros::Publisher pub_agent_vel_limits;
        ros::Publisher pub_agent_acc_limits;
        ros::Publisher pub_world_boundary;
        ros::Publisher pub_collision_alert;
//        ros::Publisher pub_desired_trajs_raw;
        ros::Publisher pub_desired_trajs_vis;
        ros::Publisher pub_grid_map;
        ros::Publisher pub_communication_range;
        ros::Publisher pub_communication_group;
        ros::Subscriber sub_global_map;
        ros::ServiceServer service_start_planning;
        ros::ServiceServer service_land;
        ros::ServiceServer service_patrol;
        ros::ServiceServer service_stop_patrol;

        const Param param;
        Mission mission;
        std::vector<std::unique_ptr<AgentManager>> agents;
        std::unique_ptr<GridBasedPlanner> grid_based_planner;
        ObstacleGenerator obstacle_generator;
        visualization_msgs::MarkerArray msg_agent_trajectories;
        visualization_msgs::MarkerArray msg_obstacle_trajectories;

        PlannerState planner_state;
        std::string mission_start_time, file_name_param;
        ros::Time sim_start_time, sim_current_time;
        bool is_collided, has_global_map, initial_update, mission_changed;
        double total_flight_time, total_distance;
        PlanningTimeStatistics planning_time;
        double safety_ratio_agent, safety_ratio_obs;
        std::vector<std::set<size_t>> groups; // Communication group

        //mapping
        std::shared_ptr<DynamicEDTOctomap> distmap_ptr;

        bool isPlannerReady();

        bool plan();

        void publish();

        bool isFinished();

        void doStep();

        void updateCollisionAlert();

        void decentralizedMAPP();

        void broadcastMsgs();

        void summarizeResult();

        void initializeROSMessage();

        void initializeSimTime();

        void initializePlannerState();

        void initializeAgents();

        void initializeObstacles();

        void saveResult();

        void saveResultAsCSV();

        void saveSummarizedResultAsCSV();

//        void saveNormalVectorAsCSV();

        double getTotalDistance();

        void globalMapCallback(const sensor_msgs::PointCloud2& pointcloud_map);

        bool startPlanningCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        bool landCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        bool patrolCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        bool stopPatrolCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

        void publishCollisionModel();

        void publishStartGoalPoints();

        void publishWorldBoundary();

        void publishAgentTrajectories();

        void publishObstacleTrajectories();

        void publishCollisionAlert();

        void publishAgentState();

        void publishDesiredTrajs();

//        void publishGridMap();

        void publishCommunicationRange();

        void publishCommunicationGroup();
    };
}