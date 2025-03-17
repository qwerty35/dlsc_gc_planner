#include <mission.hpp>

namespace MATP {
    Mission::Mission(const ros::NodeHandle &nh) {
        qn = 0;
        on = 0;

        std::string mission_file_name, world_file_name;
        nh.param<std::string>("mission", mission_file_name, "default.json");
        nh.param<std::string>("world/file_name", world_file_name, "default.bt");

        std::string package_path = ros::package::getPath("dlsc_gc_planner");
        if (mission_file_name.find(".json") != std::string::npos) {
            mission_file_names.emplace_back(package_path + "/missions/" + mission_file_name);
        } else {
            std::set<std::string> mission_set; // Use set to sort mission by file names
            std::string current_path = package_path + "/missions/" + mission_file_name;
            for (const auto &entry: fs::recursive_directory_iterator(current_path)) {
                if (entry.path().string().find(".json") != std::string::npos) {
                    mission_set.emplace(entry.path().string());
                }
            }

            for (const auto &file_name: mission_set) {
                mission_file_names.emplace_back(file_name);
            }
        }

        if (world_file_name.find(".bt") != std::string::npos or world_file_name.find(".csv") != std::string::npos) {
            world_file_names.emplace_back(package_path + "/world/" + world_file_name);
        } else {
            std::set<std::string> world_set; // Use set to sort world by file names
            std::string current_path = package_path + "/world/" + world_file_name;
            for (const auto &entry: fs::recursive_directory_iterator(current_path)) {
                if (entry.path().string().find(".bt") != std::string::npos or
                    entry.path().string().find(".csv") != std::string::npos) {
                    world_set.emplace(entry.path().string());
                }
            }

            for (const auto &file_name: world_set) {
                world_file_names.emplace_back(file_name);
            }
        }
    }

    Document Mission::parseMissionFile(const std::string& file_name) {
        std::ifstream ifs(file_name);
        IStreamWrapper isw(ifs);
        Document document;
        if (document.ParseStream(isw).HasParseError()) {
            throw std::invalid_argument("There is no such mission file " + file_name + "\n");
        }

        return document;
    }

    bool Mission::changeMission(const std::string& new_mission, double max_noise, int world_dimension, double world_z_2d){
        std::string package_path = ros::package::getPath("dlsc_gc_planner");
        std::string new_mission_file_name = package_path + "/missions/" + new_mission;

        try{
            Document document = parseMissionFile(new_mission_file_name);
            const Value &agents_list = document["agents"];
            size_t new_qn = agents_list.Size();
            if(qn != new_qn){
                ROS_ERROR_STREAM("The number of agents is not matched, prev: " << qn << ", new: " << new_qn);
                return false;
            }
        }
        catch(...){
            ROS_ERROR_STREAM("There is no such mission file " + new_mission_file_name);
            return false;
        }

        current_mission_file_name = new_mission_file_name;
        current_world_file_name = world_file_names[0];
        return readMissionFile(max_noise, world_dimension, world_z_2d);
    }


    bool Mission::loadMission(double max_noise, int world_dimension, double world_z_2d, int mission_idx) {
        current_mission_file_name = mission_file_names[mission_idx];
        if(mission_file_names.size() == world_file_names.size()){
            current_world_file_name = world_file_names[mission_idx];
        }
        else{
            current_world_file_name = world_file_names[0];
        }

        return readMissionFile(max_noise, world_dimension, world_z_2d);
    }

    bool Mission::readMissionFile(double max_noise, int world_dimension, double world_z_2d){
        Document document;
        try{
            document = parseMissionFile(current_mission_file_name);
        }
        catch(...){
            ROS_ERROR_STREAM("There is no such mission file " + current_mission_file_name);
            return false;
        }

        // World
        const Value &world_list = document["world"];
        if(world_list.Size() != 1){
            ROS_ERROR("[Mission] World must have one element");
            return false;
        }
        const Value &dimension = world_list[0].GetObject()["dimension"];
        world_min = point3d(dimension[0].GetFloat(), dimension[1].GetFloat(), dimension[2].GetFloat());
        world_max = point3d(dimension[3].GetFloat(), dimension[4].GetFloat(), dimension[5].GetFloat());

        // Quadrotors
        const Value& quadrotor_list = document["quadrotors"];
        for(Value::ConstMemberIterator itr = quadrotor_list.MemberBegin(); itr != quadrotor_list.MemberEnd(); ++itr){
            std::string quad_name = itr->name.GetString();
            Agent quadrotor;

            const Value &maxVel = itr->value.GetObject()["max_vel"];
            quadrotor.max_vel = maxVel[0].GetDouble();

            const Value &maxAcc = itr->value.GetObject()["max_acc"];
            quadrotor.max_acc = maxAcc[0].GetDouble();

            quadrotor.radius = itr->value.GetObject()["radius"].GetDouble();
            quadrotor.downwash = itr->value.GetObject()["downwash"].GetDouble();
            quadrotor.nominal_velocity = itr->value.GetObject()["nominal_velocity"].GetDouble();

            quadrotor_map.insert({quad_name, quadrotor});
        }

        // Agents
        const Value &agents_list = document["agents"];
        qn = agents_list.Size();
        agents.resize(qn);
        for (SizeType qi = 0; qi < qn; qi++) {
            // type
            if(agents_list[qi].GetObject()["type"].Empty()){
                ROS_ERROR("[Mission] Agent must have type element");
                return false;
            }
            std::string type = agents_list[qi].GetObject()["type"].GetString();
            agents[qi] = quadrotor_map[type];

            // id
            agents[qi].id = qi;

            // crazyflie id
            if(agents_list[qi].GetObject()["cid"].Empty()){
                agents[qi].cid = qi;
            } else {
                agents[qi].cid = agents_list[qi].GetObject()["cid"].GetInt();
            }

            // start
            if(agents_list[qi].GetObject()["start"].Empty()){
                ROS_ERROR("[Mission] Agent must have start element");
                return false;
            }
            const Value &start = agents_list[qi].GetObject()["start"];
            if(world_dimension == 2){
                agents[qi].start_point = point3d(start[0].GetFloat(),
                                                 start[1].GetFloat(),
                                                 world_z_2d);
            }
            else{
                agents[qi].start_point = point3d(start[0].GetFloat(),
                                                 start[1].GetFloat(),
                                                 start[2].GetFloat());
            }


            // goal
            if(agents_list[qi].GetObject()["goal"].Empty()){
                ROS_ERROR("[Mission] Agent must have goal element");
                return false;
            }
            const Value &goal = agents_list[qi].GetObject()["goal"];
            if(world_dimension == 2){
                agents[qi].desired_goal_point = point3d(goal[0].GetFloat(),
                                                        goal[1].GetFloat(),
                                                        world_z_2d);
            }
            else{
                agents[qi].desired_goal_point = point3d(goal[0].GetFloat(),
                                                        goal[1].GetFloat(),
                                                        goal[2].GetFloat());
            }

            // radius
            if(not agents_list[qi].GetObject()["size"].Empty()){
                agents[qi].radius = agents_list[qi].GetObject()["radius"].GetDouble();
            }

            //downwash
            if(not agents_list[qi].GetObject()["downwash"].Empty()){
                agents[qi].downwash = agents_list[qi].GetObject()["downwash"].GetDouble();
            }

            // speed
            if(not agents_list[qi].GetObject()["nominal_velocity"].Empty()){
                agents[qi].nominal_velocity = agents_list[qi].GetObject()["nominal_velocity"].GetDouble();
            }
        }

        initializeAgentColor();

        const Value &obstacle_list = document["obstacles"];
        on = obstacle_list.Size();
        obstacles.resize(on);
        for (SizeType oi = 0; oi < on; oi++) {
            std::string type = obstacle_list[oi].GetObject()["type"].GetString();

            if (type == "spin") {
                geometry_msgs::PoseStamped obs_axis;
                const Value &axis_position = obstacle_list[oi].GetObject()["axis_position"];
                obs_axis.pose.position.x = axis_position[0].GetDouble();
                obs_axis.pose.position.y = axis_position[1].GetDouble();
                obs_axis.pose.position.z = axis_position[2].GetDouble();

                const Value &axis_ori = obstacle_list[oi].GetObject()["axis_ori"];
                obs_axis.pose.orientation.x = axis_ori[0].GetDouble();
                obs_axis.pose.orientation.y = axis_ori[1].GetDouble();
                obs_axis.pose.orientation.z = axis_ori[2].GetDouble();

                geometry_msgs::Point obs_start;
                const Value &start = obstacle_list[oi].GetObject()["start"];
                obs_start.x = start[0].GetDouble();
                obs_start.y = start[1].GetDouble();
                obs_start.z = start[2].GetDouble();

                double obs_size = obstacle_list[oi].GetObject()["size"].GetDouble();
                double obs_speed = obstacle_list[oi].GetObject()["speed"].GetDouble();
                double obs_max_acc = obstacle_list[oi].GetObject()["max_acc"].GetDouble();
                double obs_downwash = obstacle_list[oi].GetObject()["downwash"].GetDouble();
                if (obs_downwash == 0) {
                    obs_downwash = 1;
                }

                obstacles[oi] = std::make_shared<SpinObstacle>(obs_axis, obs_start, obs_size, obs_speed, obs_max_acc,
                                                               obs_downwash);
            } else if (type == "straight") {
                const Value &start = obstacle_list[oi].GetObject()["start"];
                point3d obs_start(start[0].GetDouble(), start[1].GetDouble(), start[2].GetDouble());

                const Value &goal = obstacle_list[oi].GetObject()["goal"];
                point3d obs_goal(goal[0].GetDouble(), goal[1].GetDouble(), goal[2].GetDouble());

                double obs_size = obstacle_list[oi].GetObject()["size"].GetDouble();
                double obs_speed = obstacle_list[oi].GetObject()["speed"].GetDouble();
                double obs_max_acc = obstacle_list[oi].GetObject()["max_acc"].GetDouble();
                double obs_downwash = obstacle_list[oi].GetObject()["downwash"].GetDouble();
                if (obs_downwash == 0) {
                    obs_downwash = 1;
                }

                obstacles[oi] = std::make_shared<StraightObstacle>(obs_start, obs_goal, obs_size, obs_speed,
                                                                   obs_max_acc, obs_downwash);
            } else if (type == "patrol") {
                point3ds obs_waypoints;
                const Value &waypoints = obstacle_list[oi].GetObject()["waypoints"];
                for (int i_waypoint = 0; i_waypoint < waypoints.Size(); i_waypoint++) {
                    const Value &waypoint = waypoints[i_waypoint].GetObject()["waypoint"];
                    point3d obs_waypoint(waypoint[0].GetDouble(),
                                         waypoint[1].GetDouble(),
                                         waypoint[2].GetDouble());
                    obs_waypoints.emplace_back(obs_waypoint);
                }

                double obs_size = obstacle_list[oi].GetObject()["size"].GetDouble();
                double obs_speed = obstacle_list[oi].GetObject()["speed"].GetDouble();
                double obs_max_acc = obstacle_list[oi].GetObject()["max_acc"].GetDouble();
                double obs_downwash = obstacle_list[oi].GetObject()["downwash"].GetDouble();
                if (obs_downwash == 0) {
                    obs_downwash = 1;
                }

                obstacles[oi] = std::make_shared<PatrolObstacle>(obs_waypoints, obs_size, obs_speed, obs_max_acc,
                                                                 obs_downwash);
            } else if (type == "chasing") {
                Obstacle obs_start_state;
                const Value &start = obstacle_list[oi].GetObject()["start"];
                obs_start_state.position.x() = start[0].GetDouble();
                obs_start_state.position.y() = start[1].GetDouble();
                obs_start_state.position.z() = start[2].GetDouble();

                double obs_size = obstacle_list[oi].GetObject()["size"].GetDouble();
                double obs_max_vel = obstacle_list[oi].GetObject()["max_vel"].GetDouble();
                double obs_max_acc = obstacle_list[oi].GetObject()["max_acc"].GetDouble();
                double obs_gamma_target = obstacle_list[oi].GetObject()["gamma_target"].GetDouble();
                double obs_gamma_obs = obstacle_list[oi].GetObject()["gamma_obs"].GetDouble();
                double obs_downwash = obstacle_list[oi].GetObject()["downwash"].GetDouble();
                if (obs_downwash == 0) {
                    obs_downwash = 1;
                }

                obstacles[oi] = std::make_shared<ChasingObstacle>(obs_start_state, obs_size, obs_max_vel, obs_max_acc,
                                                                  obs_gamma_target, obs_gamma_obs,
                                                                  obs_downwash);
            } else if(type == "gaussian"){
                point3d obs_start;
                const Value &start = obstacle_list[oi].GetObject()["start"];
                obs_start.x() = start[0].GetDouble();
                obs_start.y() = start[1].GetDouble();
                obs_start.z() = start[2].GetDouble();

                point3d obs_initial_vel;
                const Value &initial_vel = obstacle_list[oi].GetObject()["initial_vel"];
                obs_initial_vel.x() = initial_vel[0].GetDouble();
                obs_initial_vel.y() = initial_vel[1].GetDouble();
                obs_initial_vel.z() = initial_vel[2].GetDouble();

                double obs_size = obstacle_list[oi].GetObject()["size"].GetDouble();
                double obs_max_vel = obstacle_list[oi].GetObject()["max_vel"].GetDouble();
                double obs_stddev_acc = obstacle_list[oi].GetObject()["stddev_acc"].GetDouble();
                double obs_max_acc = obstacle_list[oi].GetObject()["max_acc"].GetDouble();
                double obs_acc_update_cycle = obstacle_list[oi].GetObject()["acc_update_cycle"].GetDouble();
                double obs_downwash = obstacle_list[oi].GetObject()["downwash"].GetDouble();
                if(obs_acc_update_cycle == 0){
                    obs_acc_update_cycle = 0.1;
                }
                if (obs_downwash == 0) {
                    obs_downwash = 1;
                }
                obstacles[oi] = std::make_shared<GaussianObstacle>(obs_start, obs_size, obs_initial_vel, obs_max_vel,
                                                                   obs_stddev_acc, obs_max_acc, obs_acc_update_cycle,
                                                                   obs_downwash);
            }
//            else if(type == "bernstein"){
//                std::string traj_csv_path = obstacle_list[oi].GetObject()["traj_csv_path"].GetString();
//                int obs_traj_n = obstacle_list[oi].GetObject()["n"].GetInt();
//                int obs_cf_id = obstacle_list[oi].GetObject()["cf_id"].GetInt();
//                double obs_size = obstacle_list[oi].GetObject()["size"].GetDouble();
//                double obs_max_acc = obstacle_list[oi].GetObject()["max_acc"].GetDouble();
//                double obs_downwash = obstacle_list[oi].GetObject()["downwash"].GetDouble();
//
//                std::vector<point3ds> total_control_points;
//                std::vector<double> time_segments;
//                time_segments.emplace_back(0.0);
//
//                std::ifstream traj_csv(traj_csv_path);
//                int dim = 2;
//                for(auto& row: CSVRange(traj_csv))
//                {
//                    if(row.size() < 2){
//                        break;
//                    }
//                    point3ds segment_control_points;
//                    segment_control_points.resize(obs_traj_n + 1);
//                    for(int k = 0; k < dim; k++){
//                        for(int i = 0; i < obs_traj_n + 1; i++){
//                            std::string token = std::string(row[2 + (obs_traj_n + 1) * k + i]);
//                            segment_control_points[i](k) = std::stod(token);
//                        }
//                    }
//                    total_control_points.emplace_back(segment_control_points);
//                    time_segments.emplace_back(std::stod(row[1].data()));
//                }
//                obstacles[oi] = std::make_shared<BernsteinObstacle>(total_control_points, time_segments, obs_size, obs_max_acc, obs_downwash);
//            }
            else if(type == "real"){
                double obs_size = obstacle_list[oi].GetObject()["size"].GetDouble();
                double obs_speed = obstacle_list[oi].GetObject()["speed"].GetDouble();
                double obs_max_acc = obstacle_list[oi].GetObject()["max_acc"].GetDouble();
                double obs_downwash = obstacle_list[oi].GetObject()["downwash"].GetDouble();
                if (obs_downwash == 0) {
                    obs_downwash = 1;
                }

                obstacles[oi] = std::make_shared<RealObstacle>(obs_size, obs_max_acc, obs_downwash);
            }
            else {
                return false;
            }
        }

        if(max_noise > 0){
            addNoise(max_noise, world_dimension);
        }
        return true;
    }

    void Mission::addAgent(const Agent &agent) {
        agents.emplace_back(agent);
        qn++;
        initializeAgentColor();
    }

    void Mission::addObstacle(const std::shared_ptr<ObstacleBase> &obstacle_ptr) {
        obstacles.emplace_back(obstacle_ptr);
        on++;
    }

    void Mission::addNoise(double max_noise, int dimension) {
        ROS_INFO_STREAM("[Mission] Add noise " << max_noise);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(0, 1);
        for (size_t qi = 0; qi < qn; qi++) {
            for (int k = 0; k < dimension; k++) {
                agents[qi].desired_goal_point(k) += (float)(dis(gen) * max_noise);
            }
        }
    }

    // set color using HSV Colormap
    void Mission::initializeAgentColor() {
        color = getHSVColorMap(qn);
    }

    void Mission::saveMission(const std::string& file_addr) const {
        Document document = parseMissionFile(current_mission_file_name);

        Value& agents_list = document["agents"];
        Document::AllocatorType& allocator = document.GetAllocator();
        if(agents_list.Empty()){
            for(SizeType qi = 0; qi < qn; qi++){
                Value agent(kObjectType);
                Value start(kArrayType);
                Value goal(kArrayType);
                agent.AddMember("type", "crazyflie", allocator);
                for(int i = 0; i < 3; i++){
                    start.PushBack(Value().SetFloat(agents[qi].start_point(i)), allocator);
                    goal.PushBack(Value().SetFloat(agents[qi].desired_goal_point(i)), allocator);
                }
                agent.AddMember("start", start, allocator);
                agent.AddMember("goal", goal, allocator);

                agents_list.PushBack(agent, allocator);
            }
        }
        else{
            for (SizeType qi = 0; qi < qn; qi++) {
                Value& start = agents_list[qi].GetObject()["start"];
                Value& goal = agents_list[qi].GetObject()["goal"];
                for(int i = 0; i < 3; i++){
                    start[i].SetFloat(agents[qi].start_point(i));
                    goal[i].SetFloat(agents[qi].desired_goal_point(i));
                }
            }
        }

        FILE* fp = fopen(file_addr.c_str(), "w");

        char writeBuffer[65536];
        FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));

        Writer<FileWriteStream> writer(os);
        document.Accept(writer);

        fclose(fp);
    }
}