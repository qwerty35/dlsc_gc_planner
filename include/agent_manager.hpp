#ifndef DLSC_GC_PLANNER_AGENT_MANAGER_H
#define DLSC_GC_PLANNER_AGENT_MANAGER_H

#include <trajectory.hpp>
#include <traj_planner.hpp>
#include <map_manager.hpp>
#include <cmd_publisher.hpp>
#include <util.hpp>

namespace MATP {
    class AgentManager {
    public:
        AgentManager(const ros::NodeHandle &nh, const Param &param, const Mission &mission, int agent_id);

        void doStep(double time_step);

        PlanningReport plan(ros::Time sim_current_time);

        void publish();

        void publishMap();

        void obstacleCallback(const Obstacles &msg_obstacles);

        void mergeMapCallback(const octomap_msgs::Octomap& msg_merge_map);

        // Setter
        void setCurrentState(const State& msg_current_state);

        void setPlannerState(const PlannerState& new_planner_state);

        void setStartPosition(const point3d& new_start_position);

        void setDesiredGoal(const point3d& new_desired_goal);

        void setGlobalMap();

        void setGlobalMap(const sensor_msgs::PointCloud2& global_map);

        void setNextWaypoint(const point3d& next_waypoint);

        void setCollisionAlert(const CollisionAlert& collision_alert);

        // Getter
        bool isInitialStateValid() const;

        bool isAgentPoseUpdated() const;

        [[nodiscard]] point3d getCurrentPosition() const;

        [[nodiscard]] State getCurrentState() const;

        [[nodiscard]] State getFutureState(double future_time) const;

        [[nodiscard]] PlanningStatistics getPlanningStatistics() const;

        [[nodiscard]] traj_t getTraj() const;

        [[nodiscard]] int getPlannerSeq() const;

        [[nodiscard]] point3d getCurrentGoalPoint() const;

        [[nodiscard]] point3d getDesiredGoalPoint() const;

        [[nodiscard]] Obstacle getAgent() const;

        [[nodiscard]] octomap_msgs::Octomap getOctomapMsg() const;

        [[nodiscard]] point3d getNextWaypoint() const;

        [[nodiscard]] std::shared_ptr<DynamicEDTOctomap> getDistmap() const;

        [[nodiscard]] point3d getStartPoint() const;

        [[nodiscard]] CollisionAlert getCollisionAlert() const;

        [[nodiscard]] point3d getObservedAgentPosition() const;

        [[nodiscard]] point3d getObservedObstaclePosition(int obs_id) const;

        [[nodiscard]] State getObservedObstacleState(int obs_id) const;

    private:
        Param param;
        Mission mission;

        // Flags, states
        PlannerState planner_state;
        bool has_current_state, has_obstacles, has_local_map, is_disturbed;

        // Agent
        Agent agent;
        traj_t desired_traj;
        CollisionAlert collision_alert;

        //Traj Planner
        std::unique_ptr<TrajPlanner> traj_planner;
        std::unique_ptr<MapManager> map_manager;
        std::unique_ptr<CmdPublisher> cmd_publisher;

        void planningStateTransition();
    };
}

#endif //DLSC_GC_PLANNER_AGENT_MANAGER_H
