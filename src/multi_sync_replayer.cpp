#include <multi_sync_replayer.hpp>
#include <utility>

namespace MATP{
    MultiSyncReplayer::MultiSyncReplayer(const ros::NodeHandle& _nh,
                                         const Param& _param,
                                         const Mission& _mission)
                : nh(_nh), param(_param), mission(_mission), obstacle_generator(_nh, _param, _mission)
    {
        pub_agent_trajectories = nh.advertise<visualization_msgs::MarkerArray>("/agent_trajectories_history", 1);
        pub_obstacle_trajectories = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_trajectories_history", 1);
        pub_collision_model = nh.advertise<visualization_msgs::MarkerArray>("/collision_model", 1);
        pub_agent_velocities_x = nh.advertise<std_msgs::Float64MultiArray>("/agent_velocities_x", 1);
        pub_agent_velocities_y = nh.advertise<std_msgs::Float64MultiArray>("/agent_velocities_y", 1);
        pub_agent_velocities_z = nh.advertise<std_msgs::Float64MultiArray>("/agent_velocities_z", 1);
        pub_agent_accelerations_x = nh.advertise<std_msgs::Float64MultiArray>("/agent_accelerations_x", 1);
        pub_agent_accelerations_y = nh.advertise<std_msgs::Float64MultiArray>("/agent_accelerations_y", 1);
        pub_agent_accelerations_z = nh.advertise<std_msgs::Float64MultiArray>("/agent_accelerations_z", 1);
        pub_agent_vel_limits = nh.advertise<std_msgs::Float64MultiArray>("/agent_vel_limits", 1);
        pub_agent_acc_limits = nh.advertise<std_msgs::Float64MultiArray>("/agent_acc_limits", 1);
        pub_start_goal_points = nh.advertise<visualization_msgs::MarkerArray>("/start_goal_points", 1);
        pub_world_boundary = nh.advertise<visualization_msgs::MarkerArray>("/world_boundary", 1);
        pub_collision_alert = nh.advertise<visualization_msgs::MarkerArray>("/collision_alert", 1);
        pub_communication_group = nh.advertise<visualization_msgs::MarkerArray>("/communication_group", 1);

        timeStep = param.multisim_save_time_step;
        makeSpan = SP_INFINITY;

        if(param.world_use_octomap and param.world_use_global_map){
            fake_agent = std::make_unique<AgentManager>(nh, param, mission, 0);
            fake_agent->setGlobalMap();
            has_global_map = true;
        }
    }

    void MultiSyncReplayer::initializeReplay() {
        mission.agents.resize(mission.qn);
        mission.obstacles.resize(mission.on);
        msg_agent_trajectories_replay.markers.clear();
        msg_agent_trajectories_replay.markers.resize(mission.qn);
        msg_obstacle_trajectories_replay.markers.clear();
        msg_obstacle_trajectories_replay.markers.resize(mission.qn);

        agent_state_history.resize(mission.qn);
        obstacle_state_history.resize(mission.on);
    }

    void MultiSyncReplayer::replay(double t) {
        doReplay(t);
    }

    // read CSV and update history member variable
    void MultiSyncReplayer::readCSVFile(const std::string &file_name) {
        std::string full_file_name = param.package_path + "/log/" + file_name;
        std::ifstream file(full_file_name);

        if (file.fail()) {
            throw std::invalid_argument("[MultiSyncSimulator] invalid csv file, current file name: " + full_file_name);
        }

        int offset_agent = 12;
        int offset_obs = 6;
        int row_idx = 0;
        while (file.good()) {
            std::vector<std::string> row = csv_read_row(file, ',');

            if (row.size() < 2) {
                break;
            } else if (row_idx == 0) {
                // get qn, on
                mission.qn = 0;
                mission.on = 0;

                for (size_t i = 0; i < row.size() - 2; i++) {
                    if (row[i] == "id") {
                        mission.qn++;
                    } else if (row[i] == "obs_id") {
                        mission.on++;
                    }
                }
                initializeReplay();
            } else {
                for (int qi = 0; qi < mission.qn; qi++) {
                    State state;
                    state.position = point3d(std::stod(row[offset_agent * qi + 2]),
                                             std::stod(row[offset_agent * qi + 3]),
                                             std::stod(row[offset_agent * qi + 4]));
                    state.velocity = point3d(std::stod(row[offset_agent * qi + 5]),
                                             std::stod(row[offset_agent * qi + 6]),
                                             std::stod(row[offset_agent * qi + 7]));
                    state.acceleration = point3d(std::stod(row[offset_agent * qi + 8]),
                                                 std::stod(row[offset_agent * qi + 9]),
                                                 std::stod(row[offset_agent * qi + 10]));
                    agent_state_history[qi].emplace_back(state);
                    mission.agents[qi].radius = 0.15;
                }
                for (int oi = 0; oi < mission.on; oi++) {
                    State state;
                    state.position.x() = std::stod(row[offset_agent * mission.qn + offset_obs * oi + 2]);
                    state.position.y() = std::stod(row[offset_agent * mission.qn + offset_obs * oi + 3]);
                    state.position.z() = std::stod(row[offset_agent * mission.qn + offset_obs * oi + 4]);
                    obstacle_state_history[oi].emplace_back(state);
                    mission.obstacles[oi]->setRadius(std::stod(row[offset_agent * mission.qn + offset_obs * oi + 5]));
                }
                makeSpan = std::stod(row[1]);
            }
            row_idx++;
        }
    }

    void MultiSyncReplayer::setOctomap(std::string file_name) {
        octomap::OcTree octree(file_name);
        auto* octree_ptr = &octree;
        distmap_obj = std::make_shared<DynamicEDTOctomap>(param.world_max_dist, octree_ptr,
                                                          mission.world_min, mission.world_max,
                                                          false);
        distmap_obj->update();
    }

    void MultiSyncReplayer::doReplay(double t){
        if(param.multisim_replay_time_limit > 0 and t > param.multisim_replay_time_limit){
            return;
        }

        obstacle_generator.update(t);
        publish_agent_trajectories(t);
        publish_obstacle_trajectories(t);
        publish_collision_model();
        publish_start_goal_points(); //TODO: save goal point and replay it
//        publish_safety_margin(t);
        publish_collision_alert();
        publish_agent_vel_acc(t);
        publish_communication_group();

        if(has_global_map){
            fake_agent->publishMap();
        }
    }

    void MultiSyncReplayer::publish_agent_trajectories(double t){
        if(t > makeSpan){
            pub_agent_trajectories.publish(msg_agent_trajectories_replay);
            return;
        }

        for(int qi = 0; qi < mission.qn; qi++) {
            int idx = static_cast<int>(t / timeStep);
            double alpha = t/timeStep - idx;
            geometry_msgs::Point current_point;
            if(idx < agent_state_history[qi].size() - 1){
                point3d p1 = agent_state_history[qi][idx].position;
                point3d p2 = agent_state_history[qi][idx + 1].position;
                point3d p_new = p1 * (1.0 - alpha) + p2 * alpha;
                current_point = point3DToPointMsg(p_new);
            }
            else{
                current_point = point3DToPointMsg(agent_state_history[qi].back().position);
            }

            msg_agent_trajectories_replay.markers[qi].header.frame_id = param.world_frame_id;
            msg_agent_trajectories_replay.markers[qi].type = visualization_msgs::Marker::LINE_STRIP;
            msg_agent_trajectories_replay.markers[qi].action = visualization_msgs::Marker::ADD;
            msg_agent_trajectories_replay.markers[qi].pose.orientation = defaultQuaternion();
            msg_agent_trajectories_replay.markers[qi].scale.x = 0.07;
            msg_agent_trajectories_replay.markers[qi].id = qi;

            msg_agent_trajectories_replay.markers[qi].color = mission.color[qi];
            msg_agent_trajectories_replay.markers[qi].color.a = 0.75;
            msg_agent_trajectories_replay.markers[qi].points.emplace_back(current_point);
        }
        pub_agent_trajectories.publish(msg_agent_trajectories_replay);
    }

    void MultiSyncReplayer::publish_obstacle_trajectories(double t) {
        if(t > makeSpan) {
            pub_obstacle_trajectories.publish(msg_obstacle_trajectories_replay);
            return;
        }

        for(size_t oi = 0; oi < mission.on; oi++){
            int idx = static_cast<int>(t / timeStep);
            double alpha = t/timeStep - idx;
            geometry_msgs::Point current_point;
            if(idx < obstacle_state_history[oi].size() - 1){
                point3d p1 = obstacle_state_history[oi][idx].position;
                point3d p2 = obstacle_state_history[oi][idx + 1].position;
                point3d p_new = p1 * (1.0 - alpha) + p2 * alpha;
                current_point = point3DToPointMsg(p_new);
            }

            msg_obstacle_trajectories_replay.markers[oi].header.frame_id = param.world_frame_id;
            msg_obstacle_trajectories_replay.markers[oi].type = visualization_msgs::Marker::LINE_STRIP;
            msg_obstacle_trajectories_replay.markers[oi].action = visualization_msgs::Marker::ADD;
            msg_obstacle_trajectories_replay.markers[oi].scale.x = 0.07;
            msg_obstacle_trajectories_replay.markers[oi].id = oi;

            msg_obstacle_trajectories_replay.markers[oi].color.a = 0.75;
            msg_obstacle_trajectories_replay.markers[oi].color.r = 0;
            msg_obstacle_trajectories_replay.markers[oi].color.g = 0;
            msg_obstacle_trajectories_replay.markers[oi].color.b = 0;
            msg_obstacle_trajectories_replay.markers[oi].points.emplace_back(current_point);
        }
        pub_obstacle_trajectories.publish(msg_obstacle_trajectories_replay);
    }

    void MultiSyncReplayer::publish_collision_model() {
        visualization_msgs::MarkerArray msg_collision_model;
        msg_collision_model.markers.resize(mission.qn + mission.on);

        for(size_t qi = 0; qi < mission.qn; qi++) {
            visualization_msgs::Marker marker_agent;
            marker_agent.header.frame_id = param.world_frame_id;
            marker_agent.type = visualization_msgs::Marker::SPHERE;
            marker_agent.action = visualization_msgs::Marker::ADD;
            marker_agent.color = mission.color[qi];
            marker_agent.color.a = 0.8;
            marker_agent.scale.x = 2 * mission.agents[qi].radius;
            marker_agent.scale.y = 2 * mission.agents[qi].radius;
            marker_agent.scale.z = 2 * mission.agents[qi].radius * mission.agents[qi].downwash;

            marker_agent.id = qi;
            marker_agent.pose.position = msg_agent_trajectories_replay.markers[qi].points.back();
            marker_agent.pose.orientation = defaultQuaternion();
            msg_collision_model.markers[qi] = marker_agent;
        }

        for(size_t oi = 0; oi < mission.on; oi++){
            visualization_msgs::Marker marker_obstacle;
            marker_obstacle.header.frame_id = param.world_frame_id;
            marker_obstacle.type = visualization_msgs::Marker::SPHERE;
            marker_obstacle.action = visualization_msgs::Marker::ADD;
            marker_obstacle.color.a = 0.75;
            marker_obstacle.color.r = 0;
            marker_obstacle.color.g = 0;
            marker_obstacle.color.b = 0;

            marker_obstacle.scale.x = 2 * mission.obstacles[oi]->getRadius();
            marker_obstacle.scale.y = 2 * mission.obstacles[oi]->getRadius();
            marker_obstacle.scale.z = 2 * mission.obstacles[oi]->getRadius()
                                        * mission.obstacles[oi]->getDownwash();

            marker_obstacle.id = mission.qn + oi;
            marker_obstacle.pose.position = msg_obstacle_trajectories_replay.markers[oi].points.back();
            marker_obstacle.pose.orientation = defaultQuaternion();
            msg_collision_model.markers[mission.qn + oi] = marker_obstacle;
        }

        pub_collision_model.publish(msg_collision_model);
    }

    void MultiSyncReplayer::publish_start_goal_points(){
        visualization_msgs::MarkerArray msg_start_goal_points;
        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.action = visualization_msgs::Marker::ADD;

        double start_size = 0.1;
        double goal_size = 0.1;

        for(size_t qi = 0; qi < mission.qn; qi++) {
            marker.color = mission.color[qi];
            marker.color.a = 0.7;

            marker.scale.x = start_size;
            marker.scale.y = start_size;
            marker.scale.z = start_size;

            marker.ns = "start";
            marker.id = qi;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.pose.position = point3DToPointMsg(mission.agents[qi].start_point);
            marker.pose.orientation = defaultQuaternion();
            msg_start_goal_points.markers.emplace_back(marker);

            marker.ns = "goal";
            marker.id = qi;

            marker.scale.x = goal_size;
            marker.scale.y = goal_size;
            marker.scale.z = goal_size;

            marker.type = visualization_msgs::Marker::CUBE;
            marker.pose.position = point3DToPointMsg(agent_state_history[qi].back().position);
            marker.pose.orientation = defaultQuaternion();

            msg_start_goal_points.markers.emplace_back(marker);
        }
        pub_start_goal_points.publish(msg_start_goal_points);
    }

//    void MultiSyncReplayer::publish_safety_margin(double t) {
//        if(t > makeSpan) {
//            return;
//        }
//
//        for(int qi = 0; qi < mission.qn; qi++) {
//            int idx = static_cast<int>(t / timeStep);
//            double alpha = t/timeStep - idx;
//            geometry_msgs::Point current_point;
//            if(idx < safety_margin_to_agents_history[qi].size() - 1){
//                safety_margin_to_agents_replay.data[qi] = safety_margin_to_agents_history[qi][idx] * (1.0 - alpha)
//                                                          + safety_margin_to_agents_history[qi][idx + 1] * alpha;
//            }
//            if(idx < safety_margin_to_obstacles_history[qi].size() - 1){
//                safety_margin_to_obstacles_replay.data[qi] = safety_margin_to_obstacles_history[qi][idx] * (1.0 - alpha)
//                                                             + safety_margin_to_obstacles_history[qi][idx + 1] * alpha;
//            }
//        }
//        pub_safety_margin_to_agents.publish(safety_margin_to_agents_replay);
//        pub_safety_margin_to_obstacles.publish(safety_margin_to_obstacles_replay);
//    }

    void MultiSyncReplayer::publish_collision_alert(){
//        visualization_msgs::MarkerArray msg_collision_alert;
//        visualization_msgs::Marker marker;
//        marker.header.frame_id = "world";
//        marker.type = visualization_msgs::Marker::CUBE;
//        marker.action = visualization_msgs::Marker::ADD;
//
//        marker.pose.position = defaultPoint();
//        marker.pose.orientation = defaultQuaternion();
//
//        marker.scale.x = 20;
//        marker.scale.y = 20;
//        marker.scale.z = 20;
//
//        marker.color.a = 0.0;
////        for(int qi = 0; qi < mission.qn; qi++){
////            if(safety_margin_to_agents_replay.data[qi] < 1 or safety_margin_to_obstacles_replay.data[qi] < 1){
////                marker.color.a = 0.3;
////                break;
////            }
////        }
//        marker.color.r = 1;
//        marker.color.g = 0;
//        marker.color.b = 0;
//
//        msg_collision_alert.markers.emplace_back(marker);
//        pub_world_boundary.publish(msg_collision_alert);

        visualization_msgs::MarkerArray msg_world_boundary;
        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        std::vector<double> world_boundary{mission.world_min.x(), mission.world_min.y(), mission.world_min.z(),
                                           mission.world_max.x(), mission.world_max.y(), mission.world_max.z()};

        marker.pose.position = defaultPoint();
        marker.pose.orientation = defaultQuaternion();

        marker.scale.x = 0.03;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

        geometry_msgs::Point point_i, point_j;
        std::vector<int> index = {0, 1, 2,
                                  3, 4, 2,
                                  3, 1, 5,
                                  0, 4, 5};
        int offset = 0;
        for (int iter = 0; iter < 4; iter++) {
            point_i.x = world_boundary[index[offset + 0]];
            point_i.y = world_boundary[index[offset + 1]];
            point_i.z = world_boundary[index[offset + 2]];

            for (int i = 0; i < 3; i++) {
                point_j.x = world_boundary[(index[offset + 0] + 3 * (i == 0)) % 6];
                point_j.y = world_boundary[(index[offset + 1] + 3 * (i == 1)) % 6];
                point_j.z = world_boundary[(index[offset + 2] + 3 * (i == 2)) % 6];

                marker.points.emplace_back(point_i);
                marker.points.emplace_back(point_j);
            }
            offset += 3;
        }
        msg_world_boundary.markers.emplace_back(marker);
        pub_world_boundary.publish(msg_world_boundary);
    }

    void MultiSyncReplayer::publish_agent_vel_acc(double t) {
        if(t > makeSpan) {
            return;
        }

        std_msgs::Float64MultiArray msg_agent_velocities_x;
        std_msgs::Float64MultiArray msg_agent_velocities_y;
        std_msgs::Float64MultiArray msg_agent_velocities_z;
        msg_agent_velocities_x.data.resize(mission.qn);
        msg_agent_velocities_y.data.resize(mission.qn);
        msg_agent_velocities_z.data.resize(mission.qn);

        std_msgs::Float64MultiArray msg_agent_acceleration_x;
        std_msgs::Float64MultiArray msg_agent_acceleration_y;
        std_msgs::Float64MultiArray msg_agent_acceleration_z;
        msg_agent_acceleration_x.data.resize(mission.qn);
        msg_agent_acceleration_y.data.resize(mission.qn);
        msg_agent_acceleration_z.data.resize(mission.qn);

        std_msgs::Float64MultiArray msg_agent_vel_limits;
        std_msgs::Float64MultiArray msg_agent_acc_limits;
        msg_agent_vel_limits.data.resize(2);
        msg_agent_acc_limits.data.resize(2);

        for(size_t qi = 0; qi < mission.qn; qi++) {
            int idx = static_cast<int>(t / timeStep);
            double alpha = t/timeStep - idx;
            geometry_msgs::Point current_point;
            if(idx < agent_state_history[qi].size() - 1){
                point3d current_velocity = agent_state_history[qi][idx].velocity * (1.0 - alpha)
                                           + agent_state_history[qi][idx + 1].velocity * alpha;
                point3d current_acceleration = agent_state_history[qi][idx].acceleration * (1.0 - alpha)
                                               + agent_state_history[qi][idx + 1].acceleration * alpha;

                msg_agent_velocities_x.data[qi] = current_velocity.x();
                msg_agent_velocities_y.data[qi] = current_velocity.y();
                msg_agent_velocities_z.data[qi] = current_velocity.z();

                msg_agent_acceleration_x.data[qi] = current_acceleration.x();
                msg_agent_acceleration_y.data[qi] = current_acceleration.y();
                msg_agent_acceleration_z.data[qi] = current_acceleration.z();
            }
        }

        msg_agent_vel_limits.data[0] = mission.agents[0].max_vel;
        msg_agent_vel_limits.data[1] = -mission.agents[0].max_vel;
        msg_agent_acc_limits.data[0] = mission.agents[0].max_acc;
        msg_agent_acc_limits.data[1] = -mission.agents[0].max_acc;

        pub_agent_velocities_x.publish(msg_agent_velocities_x);
        pub_agent_velocities_y.publish(msg_agent_velocities_y);
        pub_agent_velocities_z.publish(msg_agent_velocities_z);
        pub_agent_accelerations_x.publish(msg_agent_acceleration_x);
        pub_agent_accelerations_y.publish(msg_agent_acceleration_y);
        pub_agent_accelerations_z.publish(msg_agent_acceleration_z);
        pub_agent_vel_limits.publish(msg_agent_vel_limits);
        pub_agent_acc_limits.publish(msg_agent_acc_limits);
    }

    void MultiSyncReplayer::publish_communication_group(){
        if(param.communication_range < 0){
            return;
        }

        visualization_msgs::MarkerArray msg_communication_group;
        visualization_msgs::Marker marker;
        marker.header.frame_id = param.world_frame_id;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "communication_route";
        marker.id = 0;
        marker.pose.orientation = defaultQuaternion();
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.2;
        marker.scale.x = 0.03;

        for(size_t qi = 0; qi < mission.qn; qi++) {
            point3d position_i = pointMsgToPoint3d(msg_agent_trajectories_replay.markers[qi].points.back());
            for(size_t qj = qi + 1; qj < mission.qn; qj++) {
                point3d position_j = pointMsgToPoint3d(msg_agent_trajectories_replay.markers[qj].points.back());
                double dist = LInfinityDistance(position_i, position_j);
                if(dist < param.communication_range){
                    marker.points.emplace_back(point3DToPointMsg(position_i));
                    marker.points.emplace_back(point3DToPointMsg(position_j));
                }
            }
        }
        msg_communication_group.markers.emplace_back(marker);

        pub_communication_group.publish(msg_communication_group);
    }

    std::vector<std::string> MultiSyncReplayer::csv_read_row(std::istream& in, char delimiter){
        std::stringstream ss;
        bool inquotes = false;
        std::vector<std::string> row;//relying on RVO
        while(in.good())
        {
            char c = in.get();
            if (!inquotes && c=='"') //beginquotechar
            {
                inquotes=true;
            }
            else if (inquotes && c=='"') //quotechar
            {
                if ( in.peek() == '"')//2 consecutive quotes resolve to 1
                {
                    ss << (char)in.get();
                }
                else //endquotechar
                {
                    inquotes=false;
                }
            }
            else if (!inquotes && c==delimiter) //end of field
            {
                row.push_back( ss.str() );
                ss.str("");
            }
            else if (!inquotes && (c=='\r' || c=='\n') )
            {
                if(in.peek()=='\n') { in.get(); }
                row.push_back( ss.str() );
                return row;
            }
            else
            {
                ss << c;
            }
        }
    }
}