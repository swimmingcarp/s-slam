#include "spark_fast_lio.h"

#include "point_ordering.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <omp.h>

namespace spark_fast_lio
{
void SPARKFastLIO2::pointBodyToWorld(PointType const *const input_point,
                                     PointType *const output_point,
                                     const state_ikfom &state)
{
    *output_point = *input_point;
    V3D point_in_body(input_point->x, input_point->y, input_point->z);
    V3D point_in_world(
        state.rot * (state.offset_R_L_I * point_in_body + state.offset_T_L_I) + state.pos);

    output_point->x = point_in_world(0);
    output_point->y = point_in_world(1);
    output_point->z = point_in_world(2);
}

void SPARKFastLIO2::pclPointBodyLidarToIMU(PointType const *const input_point,
                                           PointType *const output_point)
{
    *output_point = *input_point;
    V3D point_in_lidar(input_point->x, input_point->y, input_point->z);
    V3D point_in_imu(latest_state_.offset_R_L_I * point_in_lidar + latest_state_.offset_T_L_I);

    output_point->x = point_in_imu(0);
    output_point->y = point_in_imu(1);
    output_point->z = point_in_imu(2);
}

void SPARKFastLIO2::pclPointBodyLidarToBase(PointType const *const input_point,
                                            PointType *const output_point)
{
    *output_point = *input_point;
    V3D point_in_lidar(input_point->x, input_point->y, input_point->z);
    V3D point_in_base(lidar_rotation_in_base_ * point_in_lidar + lidar_translation_in_base_);

    output_point->x = point_in_base(0);
    output_point->y = point_in_base(1);
    output_point->z = point_in_base(2);
}

void SPARKFastLIO2::discardRemovedPointHistory()
{
    // acquire_removed_points() transfers the tree's deleted-point history and
    // clears it. The points are not needed after a local-map window move.
    PointVector points_history;
    ikd_tree_.acquire_removed_points(points_history);
}
void SPARKFastLIO2::computeMeasurementModel(
    state_ikfom &state,
    esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    selected_points_->clear();
    selected_normals_->clear();
    selected_points_->reserve(downsampled_point_count_);
    selected_normals_->reserve(downsampled_point_count_);
    if (static_cast<int>(point_residuals_.size()) < downsampled_point_count_)
    {
        point_residuals_.resize(downsampled_point_count_, 0.0f);
    }
    if (static_cast<int>(surface_point_selected_.size()) < downsampled_point_count_)
    {
        surface_point_selected_.resize(downsampled_point_count_, 0U);
    }
    total_residual_ = 0.0;
    int nearest_candidate_num = 0;
    int plane_candidate_num   = 0;

    /** closest surface search and residual computation **/
#ifdef MP_ENABLED
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for reduction(+ : nearest_candidate_num, plane_candidate_num)
#endif
    for (int i = 0; i < downsampled_point_count_; ++i)
    {
        PointType &point_body  = feats_down_body_->points[i];
        PointType &point_world = feats_down_world_->points[i];

        /* transform to world frame */
        V3D point_in_body(point_body.x, point_body.y, point_body.z);
        V3D point_in_world(
            state.rot * (state.offset_R_L_I * point_in_body + state.offset_T_L_I) + state.pos);
        point_world.x         = point_in_world(0);
        point_world.y         = point_in_world(1);
        point_world.z         = point_in_world(2);
        point_world.intensity = point_body.intensity;

        auto &points_near = nearest_map_points_[i];
        thread_local std::vector<float> point_search_sq_dis;

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikd_tree_.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, point_search_sq_dis);
            surface_point_selected_[i] = points_near.size() < NUM_MATCH_POINTS        ? false
                                      : point_search_sq_dis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                   : true;
        }

        if (!surface_point_selected_[i])
        {
            continue;
        }
        ++nearest_candidate_num;

        VF(4) plane_coefficients;
        surface_point_selected_[i] = false;
        if (esti_plane(plane_coefficients, points_near, 0.1f))
        {
            ++plane_candidate_num;
            const double point_range = point_in_body.norm();
            if (!std::isfinite(point_range) || point_range <= std::numeric_limits<double>::epsilon())
            {
                continue;
            }
            const float point_to_plane_distance =
                plane_coefficients(0) * point_world.x +
                plane_coefficients(1) * point_world.y +
                plane_coefficients(2) * point_world.z + plane_coefficients(3);
            const float plane_score =
                1 - 0.9f * fabs(point_to_plane_distance) / sqrt(point_range);

            if (plane_score > 0.9f)
            {
                surface_point_selected_[i]   = true;
                point_normals_->points[i].x  = plane_coefficients(0);
                point_normals_->points[i].y  = plane_coefficients(1);
                point_normals_->points[i].z  = plane_coefficients(2);
                point_normals_->points[i].intensity = point_to_plane_distance;
                point_residuals_[i] = abs(point_to_plane_distance);
            }
        }
    }

    effective_feature_count_ = 0;

    for (int i = 0; i < downsampled_point_count_; ++i)
    {
        if (surface_point_selected_[i])
        {
            selected_points_->push_back(feats_down_body_->points[i]);
            selected_normals_->push_back(point_normals_->points[i]);
            total_residual_ += point_residuals_[i];
            ++effective_feature_count_;
        }
    }

    if (effective_feature_count_ < 1)
    {
        ekfom_data.valid = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             1000,
                             "No Effective Points! feats_down=%d nearest_candidates=%d "
                             "plane_candidates=%d tree=%d pos=[%.3f, %.3f, %.3f] "
                             "vel=[%.3f, %.3f, %.3f]",
                             downsampled_point_count_,
                             nearest_candidate_num,
                             plane_candidate_num,
                             ikd_tree_.size(),
                             state.pos[0],
                             state.pos[1],
                             state.pos[2],
                             state.vel[0],
                             state.vel[1],
                             state.vel[2]);
        return;
    }

    mean_residual_ = total_residual_ / effective_feature_count_;
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = Eigen::MatrixXd::Zero(effective_feature_count_, 12);  // 23
    ekfom_data.h.resize(effective_feature_count_);

    for (int i = 0; i < effective_feature_count_; ++i)
    {
        const PointType &selected_point = selected_points_->points[i];
        V3D point_in_lidar(selected_point.x, selected_point.y, selected_point.z);
        M3D lidar_point_cross;
        lidar_point_cross << SKEW_SYM_MATRX(point_in_lidar);
        V3D point_in_imu = state.offset_R_L_I * point_in_lidar + state.offset_T_L_I;
        M3D imu_point_cross;
        imu_point_cross << SKEW_SYM_MATRX(point_in_imu);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &normal_point = selected_normals_->points[i];
        V3D normal(normal_point.x, normal_point.y, normal_point.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D rotated_normal(state.rot.conjugate() * normal);
        V3D position_jacobian(imu_point_cross * rotated_normal);
        if (extrinsic_est_enabled_)
        {
            V3D extrinsic_rotation_jacobian(
                lidar_point_cross * state.offset_R_L_I.conjugate() * rotated_normal);
            ekfom_data.h_x.block<1, 12>(i, 0)
                << normal_point.x, normal_point.y, normal_point.z,
                VEC_FROM_ARRAY(position_jacobian), VEC_FROM_ARRAY(extrinsic_rotation_jacobian),
                VEC_FROM_ARRAY(rotated_normal);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i, 0)
                << normal_point.x, normal_point.y, normal_point.z,
                VEC_FROM_ARRAY(position_jacobian), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -normal_point.intensity;
    }
}

void SPARKFastLIO2::updateLocalMapWindow()
{
    map_boxes_to_remove_.clear();
    const V3D lidar_position = kf_.get_lidar_position();
    if (!local_map_initialized_)
    {
        for (int i = 0; i < 3; ++i)
        {
            local_map_bounds_.vertex_min[i] = lidar_position(i) - local_map_side_length_ / 2.0;
            local_map_bounds_.vertex_max[i] = lidar_position(i) + local_map_side_length_ / 2.0;
        }
        local_map_initialized_ = true;
        return;
    }

    float distance_to_map_edge[3][2];
    bool should_move_window = false;
    for (int i = 0; i < 3; ++i)
    {
        distance_to_map_edge[i][0] = fabs(lidar_position(i) - local_map_bounds_.vertex_min[i]);
        distance_to_map_edge[i][1] = fabs(lidar_position(i) - local_map_bounds_.vertex_max[i]);
        if (distance_to_map_edge[i][0] <= kMapMoveThreshold * detection_range_ ||
            distance_to_map_edge[i][1] <= kMapMoveThreshold * detection_range_)
        {
            should_move_window = true;
        }
    }
    if (!should_move_window)
    {
        return;
    }
    BoxPointType new_map_bounds, removed_bounds;
    new_map_bounds = local_map_bounds_;
    const float move_distance =
        max((local_map_side_length_ - 2.0 * kMapMoveThreshold * detection_range_) * 0.5 * 0.9,
            static_cast<double>(detection_range_ * (kMapMoveThreshold - 1)));
    for (int i = 0; i < 3; ++i)
    {
        removed_bounds = local_map_bounds_;
        if (distance_to_map_edge[i][0] <= kMapMoveThreshold * detection_range_)
        {
            new_map_bounds.vertex_max[i] -= move_distance;
            new_map_bounds.vertex_min[i] -= move_distance;
            removed_bounds.vertex_min[i] = local_map_bounds_.vertex_max[i] - move_distance;
            map_boxes_to_remove_.push_back(removed_bounds);
        }
        else if (distance_to_map_edge[i][1] <= kMapMoveThreshold * detection_range_)
        {
            new_map_bounds.vertex_max[i] += move_distance;
            new_map_bounds.vertex_min[i] += move_distance;
            removed_bounds.vertex_max[i] = local_map_bounds_.vertex_min[i] + move_distance;
            map_boxes_to_remove_.push_back(removed_bounds);
        }
    }
    local_map_bounds_ = new_map_bounds;

    discardRemovedPointHistory();
    if (map_boxes_to_remove_.size() > 0)
    {
        ikd_tree_.Delete_Point_Boxes(map_boxes_to_remove_);
    }
}

void SPARKFastLIO2::insertScanIntoMap(const state_ikfom &state)
{
    PointVector points_to_insert;
    PointVector points_to_insert_without_downsampling;
    points_to_insert.reserve(downsampled_point_count_);
    points_to_insert_without_downsampling.reserve(downsampled_point_count_);

    for (int i = 0; i < downsampled_point_count_; ++i)
    {
        // transform to world frame
        pointBodyToWorld(&(feats_down_body_->points[i]), &(feats_down_world_->points[i]), state);

        // decide if we need to add to map
        if (!nearest_map_points_[i].empty() && is_lio_warmup_complete_)
        {
            const PointVector &points_near = nearest_map_points_[i];
            bool should_insert             = true;

            PointType voxel_center{};
            voxel_center.x = std::floor(feats_down_world_->points[i].x / filter_size_map_min_) *
                                 filter_size_map_min_ +
                             0.5 * filter_size_map_min_;
            voxel_center.y = std::floor(feats_down_world_->points[i].y / filter_size_map_min_) *
                                 filter_size_map_min_ +
                             0.5 * filter_size_map_min_;
            voxel_center.z = std::floor(feats_down_world_->points[i].z / filter_size_map_min_) *
                                 filter_size_map_min_ +
                             0.5 * filter_size_map_min_;

            const float point_to_voxel_center_distance =
                calc_dist(feats_down_world_->points[i], voxel_center);
            if (std::fabs(points_near[0].x - voxel_center.x) > 0.5f * filter_size_map_min_ &&
                std::fabs(points_near[0].y - voxel_center.y) > 0.5f * filter_size_map_min_ &&
                std::fabs(points_near[0].z - voxel_center.z) > 0.5f * filter_size_map_min_)
            {
                points_to_insert_without_downsampling.push_back(feats_down_world_->points[i]);
                continue;
            }

            for (int neighbor_index = 0; neighbor_index < NUM_MATCH_POINTS; ++neighbor_index)
            {
                if (points_near.size() < NUM_MATCH_POINTS)
                {
                    break;
                }
                if (calc_dist(points_near[neighbor_index], voxel_center) <
                    point_to_voxel_center_distance)
                {
                    should_insert = false;
                    break;
                }
            }
            if (should_insert)
            {
                points_to_insert.push_back(feats_down_world_->points[i]);
            }
        }
        else
        {
            // No nearest map points, or the filter has not initialized yet.
            points_to_insert.push_back(feats_down_world_->points[i]);
        }
    }

#ifdef SPARK_FAST_LIO_DETERMINISTIC_MAP_ORDER
    std::sort(
        points_to_insert.begin(), points_to_insert.end(), point_ordering::lessXYZICurvature);
    std::sort(points_to_insert_without_downsampling.begin(),
              points_to_insert_without_downsampling.end(),
              point_ordering::lessXYZICurvature);
#endif

    ikd_tree_.Add_Points(points_to_insert, true);
    ikd_tree_.Add_Points(points_to_insert_without_downsampling, false);
}
}  // namespace spark_fast_lio
