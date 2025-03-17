#ifndef DLSC_GC_PLANNER_GRIDBASEDPLANNER_HPP
#define DLSC_GC_PLANNER_GRIDBASEDPLANNER_HPP

#include <mapf/pibt.hpp>
#include <mapf/ecbs.hpp>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <mission.hpp>
#include <param.hpp>
#include <util.hpp>
#include <geometry.hpp>
#include <utility>
#include <collision_constraints.hpp>

// Wrapper for grid based path planner
namespace MATP {
    struct PlanResult {
        std::set<size_t> agent_ids;
        std::vector<point3ds> paths;
        void initialize();
        int getMakespan();
    };


    using DistanceTable = std::vector<std::vector<int>>;  // [agent][node_id]

    struct DOI {
        point3d agent_position;
        int closest_obs_id = -1;
        point3d closest_obs_point;
        float closest_obs_dist = SP_INFINITY;
        std::set<int> doi_cand_ids;

        bool exist() const { return !doi_cand_ids.empty(); }
    };

    struct MAPFAgent {
        size_t id;
        point3d current_agent_position, start_point, current_waypoint, goal_point;
        CollisionAlert collision_alert;
        DOI dyn_obs_interest;

        MAPFAgent(size_t _id,
                  const point3d &_current_position,
                  const point3d &_start_point,
                  const point3d &_current_way_point,
                  const point3d &_goal_point,
                  const CollisionAlert &_collision_alert = {})
                : id(_id), current_agent_position(_current_position), start_point(_start_point),
                  current_waypoint(_current_way_point), goal_point(_goal_point), collision_alert(_collision_alert) {}
    };

    typedef std::vector<MAPFAgent> MAPFAgents;


    class GridBasedPlanner {
    public:
        GridBasedPlanner(const MATP::Param &param,
                         const MATP::Mission &mission);

        // multi agent path planning
        bool planMAPF(const MAPFAgents &mapf_agents,
                      const std::shared_ptr<DynamicEDTOctomap> &_distmap_ptr,
                      double agent_radius,
                      const Obstacles &obstacles = {});

        // Getter
        [[nodiscard]] point3ds getPath(size_t i) const;

        [[nodiscard]] visualization_msgs::MarkerArray pathToMarkerMsg(int agent_id,
                                                                      const std::string &frame_id,
                                                                      std_msgs::ColorRGBA color) const;

    private:
        Mission mission;
        Param param;
        std::shared_ptr<DynamicEDTOctomap> distmap_ptr;

        Grid *grid_map;
        DistanceTable obs_distance_tables;
        size_t n_agents;
        PlanResult plan_result;

        std::array<double, 3> grid_min;
        std::array<double, 3> grid_max;
        std::array<int, 3> dim;
        double agent_downwash;

        void updateGridMap(double agent_radius,
                           const Obstacles &obstacles = {});

        void updateDistanceTables(const Obstacles &obstacles);

        void updateDOI(MAPFAgents &mapf_agents, const Obstacles &obstacles, double agent_radius);

        void updateGoal(MAPFAgents &mapf_agents);

        void updatePlanResult(const MAPF::Plan &plan, const MAPFAgents &mapf_agents);

        bool isMissionValid(const MAPFAgents &mapf_agents);

        bool isValid(const point3d &point);

        bool isOccupied(const point3d &point);

        bool isSolutionValid(const PlanResult &plan_result, const MAPFAgents &mapf_agents);

        bool runMAPF(const MAPFAgents &mapf_agents);

        PlanResult planInitialPath(const PlanResult &prev_plan_result, const MAPFAgents &mapf_agents);

        [[nodiscard]] Nodes pointsToNodes(const point3ds &points) const;

        [[nodiscard]] Nodes pointsToClosestNodes(const point3ds &points) const;

        [[nodiscard]] Node *point3DToNode(const point3d &point) const;

        [[nodiscard]] Node *point3DToClosestNode(const point3d &point) const;

        [[nodiscard]] point3d posToPoint3D(const Pos &pos) const;

        [[nodiscard]] point3d nodeToPoint3D(Node *node) const;

        [[nodiscard]] Pos point3DToPos(const point3d &point) const;

        [[nodiscard]] int point3DToID(const point3d &point) const;

        [[nodiscard]] double getGridResolution(int i) const;

        DistanceTable createDistanceTable(const point3ds &points);

        DistanceTable createDistanceTable(const std::vector<int> &ids);

        int getObsId(const Obstacles &obstacles, const point3d &obs_point);

        double getObsCost(const std::set<int> &obs_ids, int agent_id);
    };
}

#endif //DLSC_GC_PLANNER_GRIDBASEDPLANNER_HPP
