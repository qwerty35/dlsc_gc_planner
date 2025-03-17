#pragma once

#include <fstream>
#include <iostream>
#include <math.h>
#include <sp_const.hpp>
#include <mission.hpp>
#include <obstacle.hpp>
#include <random>

//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>

namespace MATP {
    class ObstacleGenerator {
    public:
        ObstacleGenerator(const ros::NodeHandle &_nh, const Param& _param, const Mission& _mission)
            : nh(_nh), param(_param), mission(_mission) {
            pub_obstacle_collision_model =
                    nh.advertise<visualization_msgs::MarkerArray>("/obstacle_collision_model",1);
            start_time = ros::Time::now();
            obstacles.resize(mission.on);
        }

        void update(double t, double observer_stddev = 0, const std::vector<point3d>& chasing_goal_points = {}){
            updateObstacles(t, chasing_goal_points);
            addNoise(observer_stddev);
        }

        void publish(const std::string& frame_id) {
            publishObstacles(frame_id);
        }

        [[nodiscard]] int getNumObs() const {
            return mission.on;
        }

        // get obstacle states which contain measurement error
        Obstacles getObstacles(){
            return obstacles;
        }

        void setWaypoints(int oi, const point3ds &waypoints) {
            if(mission.obstacles.size() <= oi or mission.obstacles[oi]->getType() != ObstacleType::DYN_PATROL) {
                return;
            }

            std::shared_ptr<PatrolObstacle> patrol_obstacle_ptr =
                    std::static_pointer_cast<PatrolObstacle>(mission.obstacles[oi]);
            patrol_obstacle_ptr->setWaypoints(waypoints);
        }

        void setStartTime(ros::Time _start_time){
            start_time = _start_time;
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher pub_obstacle_collision_model;

        Param param;
        Mission mission;
        ros::Time start_time;
        Obstacles obstacles;

        void updateObstacles(double t, const point3ds& chasing_points = {}) {
            // If obstacle is chasing, then update target and other obstacles information
            // The size of chasing points is mission.on
            if(chasing_points.size() == mission.on){
                for (size_t oi = 0; oi < mission.on; oi++) {
                    if (mission.obstacles[oi]->getType() == ObstacleType::DYN_CHASING) {
                        std::shared_ptr<ChasingObstacle> chasing_obstacle_ptr =
                                std::static_pointer_cast<ChasingObstacle>(mission.obstacles[oi]);
                        chasing_obstacle_ptr->setGoalPoint(chasing_points[oi]);
                        Obstacles obstacles_prev_step = obstacles;
                        obstacles_prev_step.erase(obstacles_prev_step.begin() + oi);
                        chasing_obstacle_ptr->setObstacles(obstacles_prev_step);
                    }
                }
            }

            //update obstacles
            if (!param.multisim_experiment and param.obs_duration > 0 and t > param.obs_duration) {
                obstacles.clear();
                mission.on = 0;
            } else {
                for (size_t oi = 0; oi < mission.on; oi++) {
                    obstacles[oi] = mission.obstacles[oi]->getObstacle(t);
                    obstacles[oi].id = oi;
                }
            }
        }

        void addNoise(double observer_stddev){
            std::random_device rd;
            std::mt19937 generator(rd());
            std::normal_distribution<double> distribution(0, observer_stddev);
            for (size_t oi = 0; oi < mission.on; oi++) {
                obstacles[oi] = obstacles[oi];
                obstacles[oi].observed_position.x() += distribution(generator);
                obstacles[oi].observed_position.y() += distribution(generator);
                obstacles[oi].observed_position.z() += distribution(generator);
            }
        }

        void publishObstacles(const std::string& frame_id) {
            visualization_msgs::MarkerArray msg_obstacle_collision_model;
            msg_obstacle_collision_model.markers.clear();

            for (const auto& obstacle : obstacles) {
                if(mission.obstacles[obstacle.id]->getType() == ObstacleType::DYN_REAL){
                    continue;
                }

                // obstacle collision model
                visualization_msgs::Marker mk_obs_true_position;
                mk_obs_true_position.header.frame_id = frame_id;
                mk_obs_true_position.ns = "true";
                mk_obs_true_position.id = obstacle.id;

                mk_obs_true_position.action = visualization_msgs::Marker::ADD;
//                mk_obs_true_position.lifetime = ros::Duration(1.0);
                mk_obs_true_position.pose.orientation.x = 0;
                mk_obs_true_position.pose.orientation.y = 0;
                mk_obs_true_position.pose.orientation.z = 0;
                mk_obs_true_position.pose.orientation.w = 1.0;
                mk_obs_true_position.pose.position = point3DToPointMsg(obstacle.position);

                mk_obs_true_position.type = visualization_msgs::Marker::SPHERE;
                mk_obs_true_position.scale.x = 2 * obstacle.radius;
                mk_obs_true_position.scale.y = 2 * obstacle.radius;
                mk_obs_true_position.scale.z = 2 * obstacle.radius * obstacle.downwash;

                mk_obs_true_position.color.a = 0.5;
                mk_obs_true_position.color.r = 0;
                mk_obs_true_position.color.g = 0;
                mk_obs_true_position.color.b = 0;
                msg_obstacle_collision_model.markers.emplace_back(mk_obs_true_position);

                visualization_msgs::Marker mk_obs_observed_position = mk_obs_true_position;
                mk_obs_observed_position.ns = "observed";
                mk_obs_observed_position.id = obstacle.id;
                mk_obs_observed_position.pose.position = point3DToPointMsg(obstacle.position);
                msg_obstacle_collision_model.markers.emplace_back(mk_obs_observed_position);
            }

            if(obstacles.empty()){
                visualization_msgs::Marker mk_delete_all;
                mk_delete_all.action = visualization_msgs::Marker::DELETEALL;
                msg_obstacle_collision_model.markers.emplace_back(mk_delete_all);
            }

            pub_obstacle_collision_model.publish(msg_obstacle_collision_model);
        }
    };
}