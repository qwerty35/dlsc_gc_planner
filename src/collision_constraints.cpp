#define CONVHULL_3D_ENABLE
#include <collision_constraints.hpp>

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

namespace MATP {
    LSC::LSC(const point3d &_obs_control_point,
             const point3d &_normal_vector,
             double _d)
            : obs_control_point(_obs_control_point), normal_vector(_normal_vector), d(_d) {}

    visualization_msgs::Marker LSC::convertToMarker(double agent_radius, const std::string &world_frame_id) const {
        visualization_msgs::Marker msg_marker;
        msg_marker.header.frame_id = world_frame_id;
        msg_marker.type = visualization_msgs::Marker::CUBE;
        msg_marker.action = visualization_msgs::Marker::ADD;

        double box_scale = 40;
        msg_marker.scale.x = box_scale;
        msg_marker.scale.y = box_scale;
        msg_marker.scale.z = box_scale;

        double distance = -(d - agent_radius) + box_scale / 2;
        Eigen::Vector3d V3d_normal_vector(normal_vector.x(), normal_vector.y(), normal_vector.z());
        Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
        Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, -V3d_normal_vector);

        msg_marker.pose.position = point3DToPointMsg(obs_control_point - normal_vector * distance);
        msg_marker.pose.orientation = quaternionToQuaternionMsg(q);

        return msg_marker;
    }

    bool LSC::isPointInLSC(const point3d &point) const {
        return (point - obs_control_point).dot(normal_vector) - d > -SP_EPSILON;
    }

    void DynamicConstraints::initialize(int N_obs_, int M, int n){
        N_obs = N_obs_;

        types.clear();
        constraints.clear();
        obs_positions.clear();

        types.resize(N_obs);
        obs_positions.resize(N_obs);
        constraints.resize(N_obs);
        for(int oi = 0; oi < N_obs; oi++){
            constraints[oi].resize(M);
            for(int m = 0; m < M; m++){
                constraints[oi][m].resize(n + 1);
            }
        }
    }

    LSC DynamicConstraints::getLSC(int oi, int m, int i) {
        return constraints[oi][m][i];
    }

    Box::Box(const point3d &_box_min, const point3d &_box_max) {
        box_min = _box_min;
        box_max = _box_max;
    }

    LSCs Box::convertToLSCs(int dim) const {
        point3d normal_vector_min, normal_vector_max;
        point3d zero_point = point3d(0, 0, 0);
        double d_min, d_max;

        LSCs lscs(2 * dim);
        for (int i = 0; i < dim; i++) {
            normal_vector_min = zero_point;
            normal_vector_max = zero_point;
            normal_vector_min(i) = 1;
            normal_vector_max(i) = -1;
            d_min = box_min(i);
            d_max = -box_max(i);

            LSC lsc_min(zero_point, normal_vector_min, d_min);
            LSC lsc_max(zero_point, normal_vector_max, d_max);
            lscs[2 * i] = lsc_min;
            lscs[2 * i + 1] = lsc_max;
        }

        return lscs;
    }

    visualization_msgs::Marker Box::convertToMarker(double agent_radius, const std::string &world_frame_id) const {
        visualization_msgs::Marker msg_marker;
        msg_marker.header.frame_id = world_frame_id;
        msg_marker.type = visualization_msgs::Marker::LINE_LIST;
        msg_marker.action = visualization_msgs::Marker::ADD;
        msg_marker.pose.position = defaultPoint();
        msg_marker.pose.orientation = defaultQuaternion();
        msg_marker.scale.x = 0.03;

        point3d inflation_vector(agent_radius, agent_radius, agent_radius);
        Box inflated_box = Box(box_min - inflation_vector, box_max + inflation_vector);
        lines_t edges = inflated_box.getEdges();
        for (const auto &edge: edges) {
            msg_marker.points.emplace_back(point3DToPointMsg(edge.start_point));
            msg_marker.points.emplace_back(point3DToPointMsg(edge.end_point));
        }

        return msg_marker;
    }

    bool Box::isPointInBox(const point3d &point) const {
        return point.x() > box_min.x() - SP_EPSILON_FLOAT &&
               point.y() > box_min.y() - SP_EPSILON_FLOAT &&
               point.z() > box_min.z() - SP_EPSILON_FLOAT &&
               point.x() < box_max.x() + SP_EPSILON_FLOAT &&
               point.y() < box_max.y() + SP_EPSILON_FLOAT &&
               point.z() < box_max.z() + SP_EPSILON_FLOAT;
    }

    bool Box::isPointInBoxStrictly(const point3d &point) const {
        return point.x() > box_min.x() + 0.0001 &&
               point.y() > box_min.y() + 0.0001 &&
               point.z() > box_min.z() + 0.0001 &&
               point.x() < box_max.x() - 0.0001 &&
               point.y() < box_max.y() - 0.0001 &&
               point.z() < box_max.z() - 0.0001;
    }

    bool Box::isLineInBox(const Line &line) const {
        return isPointInBox(line.start_point) && isPointInBox(line.end_point);
    }

    bool Box::isSegmentInBox(const Segment<point3d> &segment) const {
        for (const auto &control_point: segment.control_points) {
            if (not isPointInBox(control_point)) {
                return false;
            }
        }

        return true;
    }

    bool Box::isSegmentInBox(const Segment<point3d> &segment, double margin) const {
        point3d delta = point3d(1, 1, 1) * margin;
        Box sfc_inflated(box_min - delta, box_max + delta);
        for (const auto &control_point: segment.control_points) {
            if (not sfc_inflated.isPointInBox(control_point)) {
                return false;
            }
        }

        return true;
    }

    bool Box::isInOtherBox(const point3d &world_min, const point3d &world_max,
                           double margin) const {
        return box_min.x() > world_min.x() + margin - SP_EPSILON &&
               box_min.y() > world_min.y() + margin - SP_EPSILON &&
               box_min.z() > world_min.z() + margin - SP_EPSILON &&
               box_max.x() < world_max.x() - margin + SP_EPSILON &&
               box_max.y() < world_max.y() - margin + SP_EPSILON &&
               box_max.z() < world_max.z() - margin + SP_EPSILON;
    }

    bool Box::isSuperSetOfConvexHull(const point3ds &convex_hull) const {
        float min_value, max_value;
        for (int i = 0; i < 3; i++) {
            std::vector<float> points_i;
            for (auto point: convex_hull) {
                points_i.emplace_back(point(i));
            }
            min_value = *std::min_element(points_i.begin(), points_i.end());
            max_value = *std::max_element(points_i.begin(), points_i.end());
            if (min_value < box_min(i) - SP_EPSILON_FLOAT || max_value > box_max(i) + SP_EPSILON_FLOAT) {
                return false;
            }
        }

        return true;
    }

    bool Box::intersectWith(const Box &other_box) const {
        Box inter_sfc = intersection(other_box);
        for (int i = 0; i < 3; i++) {
            if (inter_sfc.box_min(i) > inter_sfc.box_max(i) - SP_EPSILON_FLOAT) {
                return false;
            }
        }
        return true;
    }

    bool Box::tangentWith(const Box &other_box) const {
        Box inter_sfc = intersection(other_box);
        int count = 0;
        for (int i = 0; i < 3; i++) {
            if (inter_sfc.box_min(i) > inter_sfc.box_max(i) + SP_EPSILON_FLOAT) {
                return false;
            } else if (inter_sfc.box_min(i) > inter_sfc.box_max(i) - SP_EPSILON_FLOAT) {
                count++;
            }
        }

        return count == 1;
    }

    bool Box::include(const Box &other_box) const {
        return isPointInBox(other_box.box_min) and isPointInBox(other_box.box_max);
    }

    Box Box::unify(const Box &other_box) const {
        Box unified_box;
        for (int i = 0; i < 3; i++) {
            unified_box.box_min(i) = std::min(box_min(i), other_box.box_min(i));
            unified_box.box_max(i) = std::max(box_max(i), other_box.box_max(i));
        }
        return unified_box;
    }

    Box Box::intersection(const Box &other_box) const {
        Box inter_box;
        for (int i = 0; i < 3; i++) {
            inter_box.box_min(i) = std::max(box_min(i), other_box.box_min(i));
            inter_box.box_max(i) = std::min(box_max(i), other_box.box_max(i));
        }
        return inter_box;
    }

    point3d Box::closestPoint(const point3d &point) const {
        point3d closest_point = point;
        for (int i = 0; i < 3; i++) {
            if (point(i) < box_min(i)) {
                closest_point(i) = box_min(i);
            } else if (point(i) > box_max(i)) {
                closest_point(i) = box_max(i);
            }
        }

        return closest_point;
    }

    double Box::distanceToPoint(const point3d &point) const {
        if (isPointInBox(point)) {
            return 0;
        }

        point3d closest_point = point;
        for (int i = 0; i < 3; i++) {
            if (point(i) < box_min(i)) {
                closest_point(i) = box_min(i);
            } else if (point(i) > box_max(i)) {
                closest_point(i) = box_max(i);
            }
        }

        return (point - closest_point).norm();
    }

    double Box::distanceToInnerPoint(const point3d &point) const {
        if (not isPointInBox(point)) {
            return -1;
        }

        double dist, min_dist = SP_INFINITY;
        for (int i = 0; i < 3; i++) {
            dist = abs(point(i) - box_min(i));
            if (dist < min_dist) {
                min_dist = dist;
            }

            dist = abs(point(i) - box_max(i));
            if (dist < min_dist) {
                min_dist = dist;
            }
        }

        return min_dist;
    }

    double Box::raycastFromInnerPoint(const point3d &inner_point, const point3d &direction) const {
        point3d surface_direction;
        return raycastFromInnerPoint(inner_point, direction, surface_direction);
    }

    double Box::raycastFromInnerPoint(const point3d &inner_point,
                                      const point3d &direction,
                                      point3d &surface_direction) const {
        if (not isPointInBox(inner_point)) {
            point3d delta = point3d(1, 1, 1) * 0.1;
            Box sfc_inflated(box_min - delta, box_max + delta);
            return sfc_inflated.raycastFromInnerPoint(inner_point, direction, surface_direction);
        }

        double a, b, k, min_dist = SP_INFINITY;
        for (int i = 0; i < 3; i++) {
            point3d n_surface;
            n_surface(i) = 1;
            a = (box_min - inner_point).dot(n_surface);
            b = direction.dot(n_surface);
            if (abs(b) < SP_EPSILON_FLOAT) {
                continue;
            }

            k = a / b;
            if (k > 0 and k < min_dist) {
                min_dist = k;
                surface_direction = n_surface;
            }
        }

        for (int i = 0; i < 3; i++) {
            point3d n_surface;
            n_surface(i) = -1;
            a = (box_max - inner_point).dot(n_surface);
            b = direction.dot(n_surface);
            if (abs(b) < SP_EPSILON_FLOAT) {
                continue;
            }

            k = a / b;
            if (k >= 0 and k < min_dist) {
                min_dist = k;
                surface_direction = n_surface;
            }
        }

        return min_dist;
    }

    point3ds Box::getVertices(int dim, double z_2d) const {
        point3ds vertices;
        if (dim == 2) {
            point3d point1 = box_min;
            point3d point2 = box_max;
            point1.z() = z_2d;
            point2.z() = z_2d;
            vertices.emplace_back(point1);
            vertices.emplace_back(point2);
            point1.x() = box_max.x();
            point2.x() = box_min.x();
            vertices.emplace_back(point1);
            vertices.emplace_back(point2);
        } else {
            vertices.emplace_back(box_min);
            vertices.emplace_back(box_max);
            for (int i = 0; i < dim; i++) {
                point3d point1 = box_min;
                point3d point2 = box_max;
                point1(i) = box_max(i);
                point2(i) = box_min(i);
                vertices.emplace_back(point1);
                vertices.emplace_back(point2);
            }
        }

        return vertices;
    }

    lines_t Box::getEdges() const {
        lines_t edges;

        // find edges
        point3d vertex1, vertex2, vertex3;

        vertex1 = box_min;
        for (int i = 0; i < 3; i++) {
            vertex2 = box_min;
            vertex2(i) = box_max(i);
            edges.emplace_back(Line(vertex1, vertex2));

            for (int j = 0; j < 3; j++) {
                if (i == j) continue;
                vertex3 = vertex2;
                vertex3(j) = box_max(j);
                edges.emplace_back(Line(vertex2, vertex3));
            }
        }

        vertex2 = box_max;
        for (int i = 0; i < 3; i++) {
            vertex1 = box_max;
            vertex1(i) = box_min(i);
            edges.emplace_back(Line(vertex1, vertex2));
        }

        return edges;
    }

    point3ds Box::getSurfacePoints(double resolution) const {
        point3ds surface_points;

        std::array<int, 3> max_iter = {0, 0, 0};
        for (int i = 0; i < 3; i++) {
            max_iter[i] =
                    (int) floor((box_max(i) - box_min(i) + SP_EPSILON_FLOAT) / resolution) + 1;
        }

        std::array<size_t, 3> iter = {0, 0, 0};
        for (iter[0] = 0; iter[0] < max_iter[0]; iter[0]++) {
            for (iter[1] = 0; iter[1] < max_iter[1]; iter[1]++) {
                for (iter[2] = 0; iter[2] < max_iter[2]; iter[2]++) {
                    // Skip non-surface points
                    if (iter[0] > 0 && iter[0] < max_iter[0] - 1 &&
                        iter[1] > 0 && iter[1] < max_iter[1] - 1 &&
                        iter[2] > 0 && iter[2] < max_iter[2] - 1) {
                        continue;
                    }

                    point3d surface_point;
                    for (int i = 0; i < 3; i++) {
                        surface_point(i) = box_min(i) + iter[i] * resolution;
                    }

                    surface_points.emplace_back(surface_point);
                }
            }
        }

        return surface_points;
    }

    bool Box::operator==(Box &other_sfc) const {
        return box_min.distance(other_sfc.box_min) < SP_EPSILON_FLOAT and
               box_max.distance(other_sfc.box_max) < SP_EPSILON_FLOAT;
    }

    bool Box::operator!=(Box &other_sfc) const {
        return box_min.distance(other_sfc.box_min) > SP_EPSILON_FLOAT and
               box_max.distance(other_sfc.box_max) > SP_EPSILON_FLOAT;
    }

    CollisionConstraints::CollisionConstraints(const Param &_param,
                                               const Mission &_mission,
                                               double _agent_radius,
                                               double _agent_max_vel)
            : param(_param), mission(_mission), agent_radius(_agent_radius), agent_max_vel(_agent_max_vel) {}

    void CollisionConstraints::initializeSFC(const point3d &agent_position) {
        sfcs.resize(param.M);

        Box sfc_initial, sfc_expand;
        for (size_t k = 0; k < 3; k++) {
            sfc_initial.box_min(k) = floor(agent_position(k) / param.world_resolution) * param.world_resolution;
            sfc_initial.box_max(k) = ceil(agent_position(k) / param.world_resolution) * param.world_resolution;
        }

        bool success = expandSFCIncrementally(sfc_initial, agent_radius, sfc_expand);
        if (not success) {
            throw std::invalid_argument("[CollisionConstraints] Invalid initial SFC");
        }

        for (int m = 0; m < param.M; m++) {
            sfcs[m] = sfc_expand;
        }
    }

    void CollisionConstraints::initializeLSC(const Obstacles &obstacles) {
        size_t N_obs = obstacles.size();
        lscs.initialize(N_obs, param.M, param.n);
        for (size_t oi = 0; oi < N_obs; oi++) {
            lscs.types[oi] = obstacles[oi].type;
            lscs.obs_positions[oi] = obstacles[oi].position;
        }
    }

    void CollisionConstraints::constructSFCFromPoint(const point3d &point,
                                                     const point3d &goal_point) {
        // Update sfc for segments m < M-1 from previous sfc
        for (int m = 0; m < param.M - 1; m++) {
            sfcs[m] = sfcs[m + 1];
        }

        Box sfc_update;
        bool success = expandSFCFromPoint(point, goal_point, sfcs[param.M - 1], sfc_update);
        if (not success) {
            ROS_WARN("[CollisionConstraints] Cannot find proper SFC, use previous one");
            sfc_update = sfcs[param.M - 1]; // Reuse previous one
        }

        sfcs[param.M - 1] = sfc_update;
    }

    void CollisionConstraints::constructSFCFromConvexHull(const point3ds &convex_hull,
                                                          const point3d &next_waypoint) {
        // Update sfc for segments m < M-1 from previous sfc
        for (int m = 0; m < param.M - 1; m++) {
            sfcs[m] = sfcs[m + 1];
        }

        Box sfc_update;
        point3ds convex_hull_greedy = convex_hull;
        convex_hull_greedy.emplace_back(next_waypoint);
        bool success = expandSFCFromConvexHull(convex_hull_greedy, sfc_update);
        if (not success) {
            success = expandSFCFromConvexHull(convex_hull, sfcs[param.M - 1], sfc_update);
            if (not success) {
                ROS_WARN("[CollisionConstraints] Cannot find proper SFC, use previous one");
                sfc_update = sfcs[param.M - 1]; // Reuse previous one
            }
        }

        sfcs[param.M - 1] = sfc_update;
    }

    void CollisionConstraints::constructSFCFromInitialTraj(const traj_t &initial_traj,
                                                           const point3d &current_goal_point,
                                                           const point3d &next_waypoint) {
        // Update sfc for segments m < M-1 from previous sfc
        for (int m = 0; m < param.M - 1; m++) {
            sfcs[m] = sfcs[m + 1];
        }

        // Minor refinement
        for(int m = 0; m < param.M - 2; m++) {
            point3ds control_points = initial_traj[m].control_points;
            if(sfcs[m + 1].isSuperSetOfConvexHull(control_points)){
                sfcs[m] = sfcs[m + 1];
            }
        }

        point3ds convex_hull;
        convex_hull.emplace_back(initial_traj.lastPoint());
        convex_hull.emplace_back(current_goal_point);

        point3ds convex_hull_greedy = convex_hull;
        convex_hull_greedy.emplace_back(next_waypoint);

        Box sfc_update;
        bool success = expandSFCFromConvexHull(convex_hull_greedy, sfc_update);
        if (not success) {
            success = expandSFCFromConvexHull(convex_hull, sfcs[param.M - 1], sfc_update);
            if (not success) {
                ROS_WARN("[CollisionConstraints] Cannot find proper SFC, use previous one");
                sfc_update = sfcs[param.M - 1]; // Reuse previous one
            }
        }

        sfcs[param.M - 1] = sfc_update;
    }

    void CollisionConstraints::constructCommunicationRange(const point3d &next_waypoint) {
        if(param.communication_range > 0){
            point3d delta(0.5 * param.communication_range,
                          0.5 * param.communication_range,
                          0.5 * param.communication_range);
            communication_range.box_min = next_waypoint - delta;
            communication_range.box_max = next_waypoint + delta;
        }
    }

//    std::vector<SFCs> CollisionConstraints::findSFCsCandidates(const std::vector<SFCs> &valid_sfcs) {
//        Box temp;
//        std::vector<SFCs> sfc_candidates = findSFCsCandidates(valid_sfcs, temp, 0);
//        for (auto &sfc_cand: sfc_candidates) {
//            std::reverse(sfc_cand.begin(), sfc_cand.end());
//        }
//
//        return sfc_candidates;
//    }
//
//    std::vector<SFCs> CollisionConstraints::findSFCsCandidates(const std::vector<SFCs> &valid_sfcs,
//                                                               const Box &sfc_prev,
//                                                               int m) {
//        std::vector<SFCs> sfcs_candidates;
//        if (m == param.M) {
//            SFCs sfcs_cand;
//            sfcs_candidates.emplace_back(sfcs_cand);
//            return sfcs_candidates;
//        }
//
//        for (const auto &sfc_curr: valid_sfcs[m]) {
//            if (m == 0 or sfc_curr.intersectWith(sfc_prev)) {
//                std::vector<SFCs> sfcs_candidates_child = findSFCsCandidates(valid_sfcs, sfc_curr, m + 1);
//                for (const auto &sfcs_cand_child: sfcs_candidates_child) {
//                    SFCs sfcs_cand_curr = sfcs_cand_child;
//                    sfcs_cand_curr.emplace_back(sfc_curr);
//                    sfcs_candidates.emplace_back(sfcs_cand_curr);
//                }
//            }
//        }
//
//        return sfcs_candidates;
//    }

    bool CollisionConstraints::isDynamicObstacle(int oi) const {
        return lscs.types[oi] != ObstacleType::AGENT;
    }

    bool CollisionConstraints::isPointInFeasibleRegion(const point3d &point, int m, int i) const {
        for(int oi = 0; oi < lscs.N_obs; oi++){
            if(lscs.types[oi] == ObstacleType::AGENT and !lscs.constraints[oi][m][i].isPointInLSC(point)){
                return false;
            }
        }

        if(param.world_use_octomap && !sfcs[m].isPointInBox(point)){
            return false;
        }

        return communication_range.isPointInBox(point);
    }

    LSC CollisionConstraints::getLSC(int oi, int m, int i) const {
        return lscs.constraints[oi][m][i];
    }

    Box CollisionConstraints::getSFC(int m) const {
        return sfcs[m];
    }

    size_t CollisionConstraints::getObsSize() const {
        return lscs.N_obs;
    }

    point3d CollisionConstraints::getObsPosition(int oi) const {
        return lscs.obs_positions[oi];
    }

    void CollisionConstraints::setDistmap(std::shared_ptr<DynamicEDTOctomap> distmap_ptr_) {
        distmap_ptr = distmap_ptr_;
    }

    void CollisionConstraints::setOctomap(std::shared_ptr<octomap::OcTree> octree_ptr_) {
        octree_ptr = octree_ptr_;
    }

    void CollisionConstraints::setLSC(int oi, int m, int i, const LSC &lsc) {
        lscs.constraints[oi][m][i] = lsc;
    }

    void CollisionConstraints::setSFC(int m, const Box &sfc) {
        sfcs[m] = sfc;
    }

//    visualization_msgs::MarkerArray CollisionConstraints::convertToMarkerArrayMsg(
//            const Obstacles &obstacles,
//            const Colors &colors,
//            int agent_id, double agent_radius) const {
//        visualization_msgs::MarkerArray msg1 = convertLSCsToMarkerArrayMsg(obstacles, colors, agent_radius);
//        visualization_msgs::MarkerArray msg2 = convertSFCsToMarkerArrayMsg(colors[agent_id], agent_radius);
//        msg1.markers.insert(msg1.markers.end(), msg2.markers.begin(), msg2.markers.end());
//        return msg1;
//    }

    visualization_msgs::MarkerArray CollisionConstraints::convertLSCsToMarkerArrayMsg(
            const Obstacles &obstacles,
            const Colors &colors) const {
        visualization_msgs::MarkerArray msg_marker_array;

        if (lscs.constraints.empty()) {
            return msg_marker_array;
        }

        size_t N_obs = obstacles.size();
        msg_marker_array.markers.clear();
        for (size_t oi = 0; oi < N_obs; oi++) {
            std_msgs::ColorRGBA marker_color;
            if (obstacles[oi].type == AGENT) {
                marker_color = colors[obstacles[oi].id];
                marker_color.a = 0.1;
            } else {
                marker_color.r = 0.0;
                marker_color.g = 0.0;
                marker_color.b = 0.0;
                marker_color.a = 0.1;
            }

            for (size_t m = 0; m < lscs.constraints[oi].size(); m++) {
//                visualization_msgs::Marker msg_marker = lscs[oi][m][0].convertToMarker(agent_radius);
                visualization_msgs::Marker msg_marker = lscs.constraints[oi][m][0].convertToMarker(0, param.world_frame_id);
                if (obstacles[oi].type == AGENT) {
                    msg_marker.ns = "agent" + std::to_string(m);
                } else {
                    msg_marker.ns = "dynamic_obstacle" + std::to_string(m);
                }
                msg_marker.id = m * N_obs + oi;
                msg_marker.color = marker_color;

                msg_marker_array.markers.emplace_back(msg_marker);
            }
        }

        return msg_marker_array;
    }

    visualization_msgs::MarkerArray CollisionConstraints::convertSFCsToMarkerArrayMsg(const std_msgs::ColorRGBA &color) const {
        visualization_msgs::MarkerArray msg_marker_array;

        if (sfcs.empty()) {
            return msg_marker_array;
        }

        msg_marker_array.markers.clear();
        for (size_t m = 0; m < sfcs.size(); m++) {
//            visualization_msgs::Marker msg_marker = sfcs[m].box.convertToMarker(agent_radius);
            visualization_msgs::Marker msg_marker = sfcs[m].convertToMarker(0, param.world_frame_id);
            msg_marker.id = m;
            msg_marker.ns = "SFC" + std::to_string(m);
            msg_marker.color = color;
            msg_marker.color.a = 1.0;
            msg_marker_array.markers.emplace_back(msg_marker);
        }

        return msg_marker_array;
    }

    visualization_msgs::MarkerArray CollisionConstraints::feasibleRegionToMarkerArrayMsg(
            int agent_id,
            const std_msgs::ColorRGBA &color) const {

        visualization_msgs::MarkerArray msg_marker_array;
        if (lscs.constraints.empty() and sfcs.empty()) {
            return msg_marker_array;
        }

        for (int m = 0; m < param.M + 1; m++) {
            point3ds vertices;
            if(m < param.M){
                vertices = findFeasibleVertices(m, 0);
            } else {
                vertices = findFeasibleVertices(param.M - 1, param.n);
            }

            visualization_msgs::Marker msg_marker = convexHullToMarkerMsg(vertices, color, param.world_frame_id);

            msg_marker.id = agent_id;
            msg_marker.ns = std::to_string(m);
            msg_marker.color = color;
            msg_marker.color.a = 0.4;
            msg_marker_array.markers.emplace_back(msg_marker);

//            int count = 0;
//            for(const auto& vertex: vertices){
//                visualization_msgs::Marker msg_marker;
//                msg_marker.ns = std::to_string(m);
//                msg_marker.id = count;
//                msg_marker.color = color;
//                msg_marker.color.a = 1.0;
//
//                msg_marker.header.frame_id = param.world_frame_id;
//                msg_marker.type = visualization_msgs::Marker::SPHERE;
//                msg_marker.action = visualization_msgs::Marker::ADD;
//                msg_marker.pose.position = point3DToPointMsg(vertex);
//                msg_marker.pose.orientation = defaultQuaternion();
//                msg_marker.scale.x = 0.2;
//                msg_marker.scale.y = 0.2;
//                msg_marker.scale.z = 0.2;
//
//                msg_marker_array.markers.emplace_back(msg_marker);
//                count++;
//            }
        }

        return msg_marker_array;
    }

    bool CollisionConstraints::expandSFCFromPoint(const point3d &point, const point3d &goal_point,
                                                  const Box &sfc_prev, Box &sfc_expand) {
        // Initialize initial_box
        Box sfc_initial;
        for (size_t k = 0; k < 3; k++) {
            sfc_initial.box_min(k) = floor(point(k) / param.world_resolution) * param.world_resolution;
            sfc_initial.box_max(k) = ceil(point(k) / param.world_resolution) * param.world_resolution;
        }
        if (not sfc_prev.include(sfc_initial)) {
            sfc_initial = sfc_prev.intersection(sfc_initial);
            for (size_t k = 0; k < 3; k++) {
                sfc_initial.box_min(k) = ceil((sfc_initial.box_min(k) - SP_EPSILON_FLOAT) / param.world_resolution) *
                                         param.world_resolution;
                sfc_initial.box_max(k) = floor((sfc_initial.box_max(k) + SP_EPSILON_FLOAT) / param.world_resolution) *
                                         param.world_resolution;
            }
        }

        bool success = expandSFC(sfc_initial, goal_point, agent_radius, sfc_expand);
        if (not success or not sfc_expand.isPointInBox(point)) {
            ROS_ERROR("????");
//            success = expandSFC(sfc_initial, sfc_expand, agent_radius);
        }

        return success;
    }

    bool CollisionConstraints::expandSFCFromConvexHull(const point3ds &convex_hull,
                                                       Box &sfc_expand) {
        if (convex_hull.empty()) {
            return false;
        }

        // Initialize initial_box
        Box sfc_initial;
        sfc_initial.box_min = convex_hull[0];
        sfc_initial.box_max = convex_hull[0];
        for (const auto &point: convex_hull) {
            for (int k = 0; k < 3; k++) {
                if (point(k) < sfc_initial.box_min(k)) {
                    sfc_initial.box_min(k) = point(k);
                }
                if (point(k) > sfc_initial.box_max(k)) {
                    sfc_initial.box_max(k) = point(k);
                }
            }
        }

        // Align initial SFC to grid
        for (int k = 0; k < 3; k++) {
            sfc_initial.box_min(k) = round(sfc_initial.box_min(k) / param.world_resolution) * param.world_resolution;
            sfc_initial.box_max(k) = round(sfc_initial.box_max(k) / param.world_resolution) * param.world_resolution;
        }

        // Expand initial SFC
        bool success = expandSFCIncrementally(sfc_initial, agent_radius, sfc_expand);
        if (success and not sfc_expand.isSuperSetOfConvexHull(convex_hull)) {
            success = false;
        }

        return success;
    }

    bool CollisionConstraints::expandSFCFromConvexHull(const point3ds &convex_hull,
                                                       const Box &sfc_prev,
                                                       Box &sfc_expand) {
        if (convex_hull.empty()) {
            return false;
        }

        // Initialize initial_box
        Box sfc_initial;
        sfc_initial.box_min = convex_hull[0];
        sfc_initial.box_max = convex_hull[0];
        for (const auto &point: convex_hull) {
            for (int k = 0; k < 3; k++) {
                if (point(k) < sfc_initial.box_min(k)) {
                    sfc_initial.box_min(k) = point(k);
                }
                if (point(k) > sfc_initial.box_max(k)) {
                    sfc_initial.box_max(k) = point(k);
                }
            }
        }

        // Align initial SFC to grid
        for (int k = 0; k < 3; k++) {
            sfc_initial.box_min(k) = floor(sfc_initial.box_min(k) / param.world_resolution) * param.world_resolution;
            sfc_initial.box_max(k) = ceil(sfc_initial.box_max(k) / param.world_resolution) * param.world_resolution;
        }
        if (not sfc_prev.include(sfc_initial)) {
            sfc_initial = sfc_prev.intersection(sfc_initial);
            for (size_t k = 0; k < 3; k++) {
                sfc_initial.box_min(k) = ceil((sfc_initial.box_min(k) - SP_EPSILON_FLOAT) / param.world_resolution) *
                                         param.world_resolution;
                sfc_initial.box_max(k) = floor((sfc_initial.box_max(k) + SP_EPSILON_FLOAT) / param.world_resolution) *
                                         param.world_resolution;
            }
        }

        bool success = expandSFCIncrementally(sfc_initial, agent_radius, sfc_expand);
        if (not success or not sfc_expand.isSuperSetOfConvexHull(convex_hull)) {
            ROS_ERROR("????");
        }

        return success;
    }

    bool CollisionConstraints::isObstacleInSFC(const Box &sfc, double margin) {
        point3d delta(0.5 * param.world_resolution, 0.5 * param.world_resolution, 0.5 * param.world_resolution);
        std::array<int, 3> max_iter = {0, 0, 0};
        for (int i = 0; i < 3; i++) {
            max_iter[i] =
                    (int) floor((sfc.box_max(i) - sfc.box_min(i) + SP_EPSILON_FLOAT) / param.world_resolution) + 1;
        }

        std::array<size_t, 3> iter = {0, 0, 0};
        for (iter[0] = 0; iter[0] < max_iter[0]; iter[0]++) {
            for (iter[1] = 0; iter[1] < max_iter[1]; iter[1]++) {
                for (iter[2] = 0; iter[2] < max_iter[2]; iter[2]++) {
                    float dist;
                    point3d search_point, closest_point;
                    for (int i = 0; i < 3; i++) {
                        search_point(i) = sfc.box_min(i) + iter[i] * param.world_resolution;
                    }

                    distmap_ptr->getDistanceAndClosestObstacle(search_point, dist, closest_point);
                    Box closest_cell(closest_point - delta, closest_point + delta);
                    closest_point = closest_cell.closestPoint(search_point);
                    double dist_to_obs = LInfinityDistance(closest_point, search_point);
                    if (dist < 1 and dist_to_obs < margin + SP_EPSILON_FLOAT) {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    bool CollisionConstraints::isSFCInBoundary(const Box &sfc, double margin) {
        return sfc.box_min.x() > mission.world_min.x() + margin - SP_EPSILON_FLOAT &&
               sfc.box_min.y() > mission.world_min.y() + margin - SP_EPSILON_FLOAT &&
               sfc.box_min.z() > mission.world_min.z() + margin - SP_EPSILON_FLOAT &&
               sfc.box_max.x() < mission.world_max.x() - margin + SP_EPSILON_FLOAT &&
               sfc.box_max.y() < mission.world_max.y() - margin + SP_EPSILON_FLOAT &&
               sfc.box_max.z() < mission.world_max.z() - margin + SP_EPSILON_FLOAT;
    }

//    Boxes CollisionConstraints::findSurfaceBoxes(const Box &sfc) {
//        Boxes surface_boxes;
//
//        point3ds surface_points = sfc.getSurfacePoints(param.world_resolution);
//        for (const auto& surface_point : surface_points) {
//            float dist;
//            point3d closest_point;
//            point3d delta(0.5 * param.world_resolution, 0.5 * param.world_resolution, 0.5 * param.world_resolution);
//            point3d unit_cubic(1, 1, 1);
//
//            distmap_ptr->getDistanceAndClosestObstacle(surface_point, dist, closest_point);
//            Box closest_cell(closest_point - delta, closest_point + delta);
//            closest_point = closest_cell.closestPoint(surface_point);
//            double dist_to_obs = LInfinityDistance(closest_point, surface_point);
//            double box_size =
//                    floor((dist_to_obs + SP_EPSILON_FLOAT) / param.world_resolution) * param.world_resolution;
//
//            surface_boxes.emplace_back(Box(surface_point - unit_cubic * box_size,
//                                           surface_point + unit_cubic * box_size));
//        }
//
//        return surface_boxes;
//    }
//
//    Box CollisionConstraints::unifySurfaceBoxes(const Boxes &surface_boxes){
//        Box union_box;
//
//        if(not surface_boxes.empty()){
//            union_box = surface_boxes.front();
//            for(const auto& surface_box : surface_boxes){
//                union_box = union_box.unify(surface_box);
//            }
//        }
//
//        return union_box;
//    }
//
//    Box CollisionConstraints::shrinkUnionBox(const Box &union_box,
//                                             const Box &sfc_initial,
//                                             const Boxes &surface_boxes){
//        Box shrink_box = union_box;
//
//        std::set<int> shrink_directions; // 1: +x 2: +y, 3: +z, -1: -x -2: -y, -3: -z
//        do {
//            shrink_directions.clear();
//            point3ds surface_points = shrink_box.getSurfacePoints(param.world_resolution);
//
//            for (const auto &surface_point: surface_points) {
//                bool valid_point = false;
//                for (const auto &surface_box: surface_boxes) {
//                    if (surface_box.isPointInBox(surface_point)) {
//                        valid_point = true;
//                        break;
//                    }
//                }
//
//                if (not valid_point) {
//                    for(int i = 0; i < 3; i++){
//                        bool shrink_direction_found = true;
//                        if(surface_point(i) < shrink_box.box_min(i) + SP_EPSILON_FLOAT &&
//                           surface_point(i) < sfc_initial.box_min(i) - SP_EPSILON_FLOAT){
//                            shrink_directions.insert(-i-1);
//                        } else if(surface_point(i) > shrink_box.box_max(i) - SP_EPSILON_FLOAT &&
//                                  surface_point(i) > sfc_initial.box_max(i) + SP_EPSILON_FLOAT){
//                            shrink_directions.insert(i+1);
//                        } else {
//                            shrink_direction_found = false;
//                        }
//
//                        if(shrink_direction_found){
//                            break;
//                        }
//                    }
//                }
//            }
//
//            for (const auto &shrink_direction: shrink_directions) {
//                if(shrink_direction < 0){
//                    shrink_box.box_min(-shrink_direction - 1) += param.world_resolution;
//                } else {
//                    shrink_box.box_max(shrink_direction - 1) -= param.world_resolution;
//                }
//            }
//        } while(not shrink_directions.empty());
//
//        return shrink_box;
//    }

//    bool CollisionConstraints::expandSFCUniformly(const Box &sfc_initial, double margin, Box &sfc_expand) {
//        if (isObstacleInSFC(sfc_initial, margin)) {
//            return false;
//        }
//
//        point3d unit_vector(1, 1, 1);
//        point3d delta = unit_vector * 0.5 * param.world_resolution;
//        sfc_expand = sfc_initial;
//        bool iteration = true;
//
//        while(iteration) {
//            point3ds vertices = sfc_expand.getVertices(param.world_dimension, param.world_z_2d);
//            double box_length = LInfinityDistance(sfc_expand.box_min, sfc_expand.box_max);
//            for (const auto& vertex : vertices) {
//                float dist;
//                point3d closest_point;
//                distmap_ptr->getDistanceAndClosestObstacle(vertex, dist, closest_point);
//                Box closest_cell(closest_point - delta, closest_point + delta);
//                closest_point = closest_cell.closestPoint(vertex);
//                double dist_to_obs = LInfinityDistance(closest_point, vertex);
//                if (dist_to_obs > margin + box_length + SP_EPSILON_FLOAT) {
//                    //update sfc
//                    sfc_expand = Box(vertex - unit_vector * (dist_to_obs - margin), vertex + unit_vector * (dist_to_obs - margin));
//                    continue;
//                }
//                iteration = false;
//            }
//        }
//
//        return true;
//    }

    bool CollisionConstraints::expandSFCIncrementally(const Box &sfc_initial, double margin, Box &sfc_expand) {
        if (isObstacleInSFC(sfc_initial, margin)) {
            return false;
        }

        Box sfc, sfc_cand, sfc_update;
        std::vector<int> axis_cand = {0, 1, 2, 3, 4, 5}; // -x, -y, -z, +x, +y, +z

        int max_iter = (int)round(std::max(2 * param.grid_resolution, agent_max_vel * param.dt) / param.world_resolution) + 1;
//        int max_iter = SP_INFINITY;
        std::vector<int> axis_iter = {0, 0, 0, 0, 0, 0};

        int i = -1;
        int axis;
        sfc = sfc_initial;
        while (!axis_cand.empty()) {
            // initialize boxes
            sfc_cand = sfc;
            sfc_update = sfc;

            //check collision update_box only! box_current + box_update = box_cand
            while (isSFCInBoundary(sfc_update, 0) && !isObstacleInSFC(sfc_update, margin)) {
                i++;
                if (i >= axis_cand.size()) {
                    i = 0;
                }
                axis = axis_cand[i];

                //update current box
                sfc = sfc_cand;
                sfc_update = sfc_cand;

                //expand box_cand and get updated part of box (box_update)
                if (axis < 3) {
                    sfc_update.box_max(axis) = sfc_cand.box_min(axis);
                    sfc_cand.box_min(axis) = sfc_cand.box_min(axis) - param.world_resolution;
                    sfc_update.box_min(axis) = sfc_cand.box_min(axis);
                } else {
                    sfc_update.box_min(axis - 3) = sfc_cand.box_max(axis - 3);
                    sfc_cand.box_max(axis - 3) = sfc_cand.box_max(axis - 3) + param.world_resolution;
                    sfc_update.box_max(axis - 3) = sfc_cand.box_max(axis - 3);
                }

                axis_iter[axis] = axis_iter[axis] + 1;
                if(axis_iter[axis] > max_iter){
                    break;
                }
            }
            // if obstacle is in box then do not expand box to the current axis direction
            axis_cand.erase(axis_cand.begin() + i);
            if (i > 0) {
                i--;
            } else {
                i = axis_cand.size() - 1;
            }
        }

        //SFC margin compensation
        double delta = margin - ((int) (margin / param.world_resolution) * param.world_resolution);
        for (int k = 0; k < 3; k++) {
            if (sfc.box_min(k) > mission.world_min(k) + SP_EPSILON_FLOAT) {
                sfc.box_min(k) = sfc.box_min(k) - delta;
            }
            if (sfc.box_max(k) < mission.world_max(k) - SP_EPSILON_FLOAT) {
                sfc.box_max(k) = sfc.box_max(k) + delta;
            }
        }

        sfc_expand = sfc;
        return true;
    }

//    bool CollisionConstraints::expandSFCLInfinity(const Box &sfc_initial, double margin, Box &sfc_expand) {
//        if (isObstacleInSFC(sfc_initial, margin)) {
//            return false;
//        }
//
//        sfc_expand = sfc_initial;
//        Box sfc_prev;
//        while(sfc_expand != sfc_prev){
//            sfc_prev = sfc_expand;
//
//            // Expand surface points until it meats obstacles
//            Boxes surface_boxes = findSurfaceBoxes(sfc_expand);
//
//            // Unify surface boxes
//            Box union_box = unifySurfaceBoxes(surface_boxes);
//            union_box = union_box.intersection(Box(mission.world_min, mission.world_max));
//
//            // Shrink union box until there is no obstacle in it
//            sfc_expand = shrinkUnionBox(union_box, sfc_expand, surface_boxes);
//        }
//
//        return true;
//    }

    bool CollisionConstraints::expandSFC(const Box &sfc_initial, const point3d &goal_point, double margin,
                                         Box &sfc_expand) {
        if (isObstacleInSFC(sfc_initial, margin)) {
            return false;
        }

        Box sfc, sfc_cand, sfc_update;
        std::vector<int> axis_cand = setAxisCand(sfc_initial, goal_point);  // -x, -y, -z, +x, +y, +z
        // {0, 1, 2, 3, 4, 5};

        int i = -1;
        int axis;
        sfc = sfc_initial;
        while (!axis_cand.empty()) {
            // initialize boxes
            sfc_cand = sfc;
            sfc_update = sfc;

            //check collision update_box only! box_current + box_update = box_cand
            while (isSFCInBoundary(sfc_update, 0) && !isObstacleInSFC(sfc_update, margin)) {
                i++;
                if (i >= axis_cand.size()) {
                    i = 0;
                }
                axis = axis_cand[i];

                //update current box
                sfc = sfc_cand;
                sfc_update = sfc_cand;

                //expand box_cand and get updated part of box (box_update)
                if (axis < 3) {
                    sfc_update.box_max(axis) = sfc_cand.box_min(axis);
                    sfc_cand.box_min(axis) = sfc_cand.box_min(axis) - param.world_resolution;
                    sfc_update.box_min(axis) = sfc_cand.box_min(axis);
                } else {
                    sfc_update.box_min(axis - 3) = sfc_cand.box_max(axis - 3);
                    sfc_cand.box_max(axis - 3) = sfc_cand.box_max(axis - 3) + param.world_resolution;
                    sfc_update.box_max(axis - 3) = sfc_cand.box_max(axis - 3);
                }
            }
            // if obstacle is in box then do not expand box to the current axis direction
            axis_cand.erase(axis_cand.begin() + i);
            if (i > 0) {
                i--;
            } else {
                i = axis_cand.size() - 1;
            }
        }

        //SFC margin compensation
        double delta = margin - ((int) (margin / param.world_resolution) * param.world_resolution);
        for (int k = 0; k < 3; k++) {
            if (sfc.box_min(k) > mission.world_min(k) + SP_EPSILON_FLOAT) {
                sfc.box_min(k) = sfc.box_min(k) - delta;
            }
            if (sfc.box_max(k) < mission.world_max(k) - SP_EPSILON_FLOAT) {
                sfc.box_max(k) = sfc.box_max(k) + delta;
            }
        }

        sfc_expand = sfc;
        return true;
    }

    point3ds CollisionConstraints::findFeasibleVertices(int m, int control_point_idx) const {
        // Use brute force method to find vertices of feasible region
        LSCs lscs_f;

        // LSC
        for (size_t oi = 0; oi < lscs.N_obs; oi++) {
            if (isDynamicObstacle(oi)) {
                continue;
            }
            lscs_f.emplace_back(lscs.constraints[oi][m][control_point_idx]);
        }

        // SFC
        if (param.world_use_octomap) {
            LSCs lsc_sfc = sfcs[m].convertToLSCs(3);
            for (const auto &lsc: lsc_sfc) {
                lscs_f.emplace_back(lsc);
            }
        } else {
            Box sfc_world_boundary(mission.world_min, mission.world_max);
            LSCs lscs_world_boundary = sfc_world_boundary.convertToLSCs(3);
            for (const auto &lsc: lscs_world_boundary) {
                lscs_f.emplace_back(lsc);
            }
        }

        // Communication range
        if(param.communication_range > 0){
            LSCs lscs_communication_range = communication_range.convertToLSCs(3);
            for (const auto &lsc: lscs_communication_range) {
                lscs_f.emplace_back(lsc);
            }
        }

        point3ds vertices;
        size_t N = lscs_f.size();
        for (size_t i = 0; i < N; i++) {
            for (size_t j = i + 1; j < N; j++) {
                for (size_t k = j + 1; k < N; k++) {
                    Eigen::Matrix3f A;
                    A << lscs_f[i].normal_vector(0), lscs_f[i].normal_vector(1), lscs_f[i].normal_vector(2),
                         lscs_f[j].normal_vector(0), lscs_f[j].normal_vector(1), lscs_f[j].normal_vector(2),
                         lscs_f[k].normal_vector(0), lscs_f[k].normal_vector(1), lscs_f[k].normal_vector(2);
                    Eigen::Vector3f b;
                    b << lscs_f[i].obs_control_point.dot(lscs_f[i].normal_vector) + lscs_f[i].d,
                            lscs_f[j].obs_control_point.dot(lscs_f[j].normal_vector) + lscs_f[j].d,
                            lscs_f[k].obs_control_point.dot(lscs_f[k].normal_vector) + lscs_f[k].d;

                    Eigen::Vector3f x = A.householderQr().solve(b);
                    double relative_error = (A * x - b).norm() / b.norm();
                    if (relative_error < SP_EPSILON_FLOAT) {
                        point3d point(x(0), x(1), x(2));
                        bool feasible_point = true;
                        for (size_t ci = 0; ci < N; ci++) {
                            if ((point - lscs_f[ci].obs_control_point).dot(lscs_f[ci].normal_vector) - lscs_f[ci].d <
                                -SP_EPSILON_FLOAT) {
                                feasible_point = false;
                                continue;
                            }
                        }

                        if (feasible_point) {
                            vertices.emplace_back(point);
                        }
                    }
                }
            }
        }

        return vertices;
    }

//    visualization_msgs::Marker CollisionConstraints::convexHullToMarkerMsg(const point3ds& convex_hull,
//                                                                           const std::string& world_frame_id) {
//        visualization_msgs::Marker msg_marker;
//        msg_marker.header.frame_id = world_frame_id;
//        msg_marker.type = visualization_msgs::Marker::LINE_LIST;
//        msg_marker.action = visualization_msgs::Marker::ADD;
//        msg_marker.pose.position = defaultPoint();
//        msg_marker.pose.orientation = defaultQuaternion();
//        msg_marker.scale.x = 0.03;
//
//        if(convex_hull.empty()){
//            return msg_marker;
//        }
//
//        size_t nVertices = convex_hull.size();
//        ch_vertex* vertices;
//        vertices = (ch_vertex*)malloc(nVertices*sizeof(ch_vertex));
//        for (int i = 0; i < nVertices; i++) {
//            vertices[i].x = convex_hull[i].x();
//            vertices[i].y = convex_hull[i].y();
//            vertices[i].z = convex_hull[i].z();
//        }
//
//        int* faceIndices = NULL;
//        int nFaces;
//        convhull_3d_build(vertices, nVertices, &faceIndices, &nFaces);
//        /* Where 'faceIndices' is a flat 2D matrix [nFaces x 3] */
//
//        for(int i = 0; i < nFaces; i++){
//            auto vertex1 = vertices[faceIndices[i * 3]];
//            auto vertex2 = vertices[faceIndices[i * 3 + 1]];
//            auto vertex3 = vertices[faceIndices[i * 3 + 2]];
//
//            point3d point1(vertex1.x, vertex1.y, vertex1.z);
//            point3d point2(vertex2.x, vertex2.y, vertex2.z);
//            point3d point3(vertex3.x, vertex3.y, vertex3.z);
//
//            geometry_msgs::Point pointmsg1 = point3DToPointMsg(point1);
//            geometry_msgs::Point pointmsg2 = point3DToPointMsg(point2);
//            geometry_msgs::Point pointmsg3 = point3DToPointMsg(point3);
//
//            msg_marker.points.emplace_back(pointmsg1);
//            msg_marker.points.emplace_back(pointmsg2);
//
//            msg_marker.points.emplace_back(pointmsg2);
//            msg_marker.points.emplace_back(pointmsg3);
//
//            msg_marker.points.emplace_back(pointmsg3);
//            msg_marker.points.emplace_back(pointmsg1);
//        }
//
//        free(vertices);
//        free(faceIndices);
//        return msg_marker;
//    }

    visualization_msgs::Marker CollisionConstraints::convexHullToMarkerMsg(const point3ds &convex_hull,
                                                                           const std_msgs::ColorRGBA &color,
                                                                           const std::string &world_frame_id) {
        visualization_msgs::Marker msg_marker;
        msg_marker.header.frame_id = world_frame_id;
        msg_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        msg_marker.action = visualization_msgs::Marker::ADD;
        msg_marker.pose.position = defaultPoint();
        msg_marker.pose.orientation = defaultQuaternion();
        msg_marker.scale.x = 1;
        msg_marker.scale.y = 1;
        msg_marker.scale.z = 1;
        std_msgs::ColorRGBA mesh_color = color;
        mesh_color.a = 0.2;


        if (convex_hull.empty()) {
            return msg_marker;
        }

        size_t nVertices = convex_hull.size();
        ch_vertex *vertices;
        vertices = (ch_vertex *) malloc(nVertices * sizeof(ch_vertex));
        for (size_t i = 0; i < nVertices; i++) {
            vertices[i].x = convex_hull[i].x();
            vertices[i].y = convex_hull[i].y();
            vertices[i].z = convex_hull[i].z();
        }

        int *faceIndices = NULL;
        int nFaces;
        convhull_3d_build(vertices, nVertices, &faceIndices, &nFaces);
        /* Where 'faceIndices' is a flat 2D matrix [nFaces x 3] */

        for (int i = 0; i < nFaces; i++) {
            auto vertex1 = vertices[faceIndices[i * 3]];
            auto vertex2 = vertices[faceIndices[i * 3 + 1]];
            auto vertex3 = vertices[faceIndices[i * 3 + 2]];

            point3d point1(vertex1.x, vertex1.y, vertex1.z);
            point3d point2(vertex2.x, vertex2.y, vertex2.z);
            point3d point3(vertex3.x, vertex3.y, vertex3.z);

            geometry_msgs::Point pointmsg1 = point3DToPointMsg(point1);
            geometry_msgs::Point pointmsg2 = point3DToPointMsg(point2);
            geometry_msgs::Point pointmsg3 = point3DToPointMsg(point3);

            msg_marker.points.emplace_back(pointmsg1);
            msg_marker.points.emplace_back(pointmsg2);
            msg_marker.points.emplace_back(pointmsg3);
            msg_marker.colors.emplace_back(mesh_color);
        }

        free(vertices);
        free(faceIndices);
        return msg_marker;
    }

    std::vector<int> CollisionConstraints::setAxisCand(const Box &box, const octomap::point3d &goal_point) {
        std::vector<int> axis_cand(6);

        octomap::point3d mid_point = (box.box_min + box.box_max) * 0.5;
        octomap::point3d delta = goal_point - mid_point;
        std::vector<int> offsets;
        offsets.emplace_back(delta.x() > 0 ? 3 : 0);
        offsets.emplace_back(delta.y() > 0 ? 3 : 0);
        offsets.emplace_back(delta.z() > 0 ? 3 : 0);
        std::vector<double> values;
        values.emplace_back(abs(delta.x()));
        values.emplace_back(abs(delta.y()));
        values.emplace_back(abs(delta.z()));

        std::vector<int> element_order; // 0 is largest, 2 is smallest
        double max_value = -1;
        double min_value = SP_INFINITY;
        for (int i = 0; i < 3; i++) {
            if (values[i] > max_value) {
                element_order.insert(element_order.begin(), i);
                max_value = values[i];
            } else if (values[i] < min_value) {
                element_order.emplace_back(i);
                min_value = values[i];
            } else {
                element_order.insert(element_order.begin() + 1, i);
            }
        }

        for (int i = 0; i < 3; i++) {
            axis_cand[i] = element_order[i] + offsets[element_order[i]];
            axis_cand[5 - i] = element_order[i] + (3 - offsets[element_order[i]]);
        }

        return axis_cand;
    }
}

//#pragma GCC pop_options