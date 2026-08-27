#include "slam/pose_graph_manager.h"
#include "slam/input_config.hpp"

using namespace kiss_matcher;

PoseGraphManager::PoseGraphManager(const rclcpp::NodeOptions &options)
    : rclcpp::Node("km_sam", options)
{
    double loop_detector_hz;
    double loop_nnsearch_hz;
    double map_update_hz;
    double vis_hz;
    int max_pending_loop_candidates;

    LoopClosureConfig lc_config;
    LoopDetectorConfig ld_config;
    auto &gc = lc_config.gicp_config_;

    map_frame_              = declare_parameter<std::string>("map_frame", "map");
    base_frame_             = declare_parameter<std::string>("base_frame", "base");
    odom_frame_             = declare_parameter<std::string>("odom_frame", "odom");
    publish_map_to_odom_tf_ = declare_parameter<bool>("publish_map_to_odom_tf", false);
    max_sync_interval_      = declare_parameter<double>("input.max_sync_interval", 0.05);
    const auto odom_reliability = parseInputReliability(
        declare_parameter<std::string>("input.odom_qos_reliability", "reliable"),
        "input.odom_qos_reliability");
    const auto odom_qos = makeInputQos(
        declare_parameter<int>("input.odom_qos_depth", 10),
        odom_reliability,
        "input.odom_qos_depth");
    const auto cloud_reliability = parseInputReliability(
        declare_parameter<std::string>("input.cloud_qos_reliability", "reliable"),
        "input.cloud_qos_reliability");
    const auto cloud_qos = makeInputQos(
        declare_parameter<int>("input.cloud_qos_depth", 10),
        cloud_reliability,
        "input.cloud_qos_depth");
    const int sync_queue_size = declare_parameter<int>("input.sync_queue_size", 10);
    declare_parameter<double>("loop_pub_hz", 0.1);
    loop_detector_hz          = declare_parameter<double>("loop_detector_hz", 1.0);
    loop_nnsearch_hz          = declare_parameter<double>("loop_nnsearch_hz", 1.0);
    loop_candidate_cooldown_ = declare_parameter<double>("loop_pub_delayed_time", 60.0);
    max_pending_loop_candidates = declare_parameter<int>("loop.max_pending_candidates", 32);
    map_update_hz             = declare_parameter<double>("map_update_hz", 0.2);
    vis_hz                    = declare_parameter<double>("vis_hz", 0.5);

    if (!std::isfinite(max_sync_interval_) || max_sync_interval_ < 0.0)
    {
        throw std::invalid_argument("input.max_sync_interval must be finite and non-negative");
    }
    const uint32_t input_sync_queue_size =
        inputQueueSize(sync_queue_size, "input.sync_queue_size");
    if (max_pending_loop_candidates <= 0)
    {
        throw std::invalid_argument("loop.max_pending_candidates must be positive");
    }
    loop_candidate_queue_.setCapacity(static_cast<size_t>(max_pending_loop_candidates));

    store_voxelized_scan_           = declare_parameter<bool>("store_voxelized_scan", false);
    lc_config.voxel_res_            = declare_parameter<double>("voxel_resolution", 0.3);
    scan_voxel_res_                 = lc_config.voxel_res_;
    map_voxel_res_                  = declare_parameter<double>("map_voxel_resolution", 1.0);
    save_voxel_res_                 = declare_parameter<double>("save_voxel_resolution", 0.3);
    keyframe_thr_                   = declare_parameter<double>("keyframe.keyframe_threshold", 1.0);
    lc_config.num_submap_keyframes_ = declare_parameter<int>("keyframe.num_submap_keyframes", 5);
    lc_config.verbose_              = declare_parameter<bool>("loop.verbose", false);
    lc_config.is_multilayer_env_    = declare_parameter<bool>("loop.is_multilayer_env", false);
    lc_config.loop_detection_radius_ =
        declare_parameter<double>("loop.loop_detection_radius", 15.0);
    lc_config.loop_detection_timediff_threshold_ =
        declare_parameter<double>("loop.loop_detection_timediff_threshold", 10.0);

    gc.num_threads_               = declare_parameter<int>("local_reg.num_threads", 8);
    gc.correspondence_randomness_ = declare_parameter<int>("local_reg.correspondences_number", 20);
    gc.max_num_iter_              = declare_parameter<int>("local_reg.max_num_iter", 32);
    gc.scale_factor_for_corr_dist_ =
        declare_parameter<double>("local_reg.scale_factor_for_corr_dist", 5.0);
    gc.overlap_threshold_ = declare_parameter<double>("local_reg.overlap_threshold", 90.0);

    lc_config.enable_global_registration_ = declare_parameter<bool>("global_reg.enable", false);
    lc_config.num_inliers_threshold_ =
        declare_parameter<int>("global_reg.num_inliers_threshold", 100);

    save_map_bag_         = declare_parameter<bool>("result.save_map_bag", false);
    save_map_pcd_         = declare_parameter<bool>("result.save_map_pcd", false);
    save_in_kitti_format_ = declare_parameter<bool>("result.save_in_kitti_format", false);
    seq_name_             = declare_parameter<std::string>("result.seq_name", "");

    rclcpp::QoS qos(1);
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    package_path_ = "";

    loop_closure_          = std::make_shared<LoopClosure>(lc_config, this->get_logger());
    loop_detection_radius_ = lc_config.loop_detection_radius_;

    loop_detector_ = std::make_shared<LoopDetector>(ld_config, this->get_logger());

    gtsam::ISAM2Params isam_params_;
    isam_params_.relinearizeThreshold = 0.01;
    isam_params_.relinearizeSkip      = 1;
    isam_handler_                     = std::make_shared<gtsam::ISAM2>(isam_params_);

    odom_path_.header.frame_id      = map_frame_;
    corrected_path_.header.frame_id = map_frame_;

    // NOTE(hlim): To make this node compatible with being launched under different namespaces,
    // I deliberately avoided adding a '/' in front of the topic names.
    path_pub_           = this->create_publisher<nav_msgs::msg::Path>("path/original", qos);
    corrected_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path/corrected", qos);
    map_pub_            = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", qos);
    scan_pub_           = this->create_publisher<sensor_msgs::msg::PointCloud2>("curr_scan", qos);
    loop_detection_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("loop_detection", qos);
    loop_detection_radius_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>("loop_detection_radius", qos);

    // loop_closures_pub_ =
    // this->create_publisher<pose_graph_tools_msgs::msg::PoseGraph>("/hydra_ros_node/external_loop_closures",
    // 10);
    realtime_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_stamped", qos);
    debug_src_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/src", qos);
    debug_tgt_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/tgt", qos);
    debug_coarse_aligned_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/coarse_alignment", qos);
    debug_fine_aligned_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/fine_alignment", qos);
    debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lc/debug_cloud", qos);

    RCLCPP_INFO(get_logger(),
                "Input: odom QoS=%s depth=%zu, cloud QoS=%s depth=%zu, sync queue=%d, "
                "max interval=%.3f s",
                inputReliabilityName(odom_reliability),
                odom_qos.get_rmw_qos_profile().depth,
                inputReliabilityName(cloud_reliability),
                cloud_qos.get_rmw_qos_profile().depth,
                sync_queue_size,
                max_sync_interval_);

    sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry> >(
        this, "odom", odom_qos.get_rmw_qos_profile());
    sub_scan_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2> >(
        this, "cloud", cloud_qos.get_rmw_qos_profile());

    NodeSyncPolicy sync_policy(input_sync_queue_size);
    sync_policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_sync_interval_));
    sub_node_ = std::make_shared<message_filters::Synchronizer<NodeSyncPolicy> >(
        static_cast<const NodeSyncPolicy &>(sync_policy), *sub_odom_, *sub_scan_);
    sub_node_->registerCallback(std::bind(
                                    &PoseGraphManager::callbackNode, this, std::placeholders::_1, std::placeholders::_2));

    sub_save_flag_ = this->create_subscription<std_msgs::msg::String>(
        "save_dir", 1, std::bind(&PoseGraphManager::saveFlagCallback, this, std::placeholders::_1));

    // hydra_loop_timer_ = this->create_wall_timer(
    //   std::chrono::duration<double>(1.0 / loop_pub_hz),
    //   std::bind(&PoseGraphManager::loopPubTimerFunc, this));

    map_cloud_.reset(new pcl::PointCloud<PointType>());
    map_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / map_update_hz),
                                         std::bind(&PoseGraphManager::buildMap, this));

    loop_search_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    loop_detector_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / loop_detector_hz),
        std::bind(&PoseGraphManager::detectLoopClosureByLoopDetector, this),
        loop_search_callback_group_);

    loop_nnsearch_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(1.0 / loop_nnsearch_hz),
                                std::bind(&PoseGraphManager::detectLoopClosureByNNSearch, this),
                                loop_search_callback_group_);

    graph_vis_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(1.0 / vis_hz),
                                std::bind(&PoseGraphManager::visualizePoseGraph, this));

    registration_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    lc_reg_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 100.0),
                                            std::bind(&PoseGraphManager::performRegistration, this),
                                            registration_callback_group_);

    // 20 Hz is enough as long as it's faster than the full registration process.
    lc_vis_timer_ =
        this->create_wall_timer(std::chrono::duration<double>(1.0 / 20.0),
                                std::bind(&PoseGraphManager::visualizeLoopClosureClouds, this),
                                registration_callback_group_);

    if (!lc_config.is_multilayer_env_)
    {
        RCLCPP_WARN(
            get_logger(),
            "'loop.is_multilayer_env' is set to `false`. "
            "This setting is recommended for outdoor environments to ignore the effect of Z-drift. "
            "However, if you're running SLAM in an indoor multi-layer environment, "
            "consider setting it to true to enable full 3D NN search for loop candidates.");
    }
    RCLCPP_INFO(this->get_logger(), "Main class, starting node...");
}

PoseGraphManager::~PoseGraphManager()
{
    if (save_map_bag_)
    {
        RCLCPP_INFO(this->get_logger(),
                    "NOTE(hlim): skipping final bag save in ROS2 example code.");
    }
    if (save_map_pcd_)
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());

        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            if (keyframes_.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No keyframes available; skipping final PCD save.");
                return;
            }
            corrected_map->reserve(keyframes_[0].scan_.size() * keyframes_.size());
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].scan_, keyframes_[i].pose_corrected_);
            }
        }
        const auto &voxelized_map = voxelize(corrected_map, save_voxel_res_);
        pcl::io::savePCDFileASCII<PointType>(package_path_ + "/result.pcd", *voxelized_map);
        RCLCPP_INFO(this->get_logger(), "Result saved in .pcd format (Destructor).");
    }
}

void PoseGraphManager::appendKeyframePose(const PoseGraphNode &node)
{
    const rclcpp::Time timestamp(node.timestamp_);

    odoms_.points.emplace_back(node.pose_(0, 3), node.pose_(1, 3), node.pose_(2, 3));

    corrected_odoms_.points.emplace_back(
        node.pose_corrected_(0, 3), node.pose_corrected_(1, 3), node.pose_corrected_(2, 3));

    odom_path_.header.stamp      = timestamp;
    corrected_path_.header.stamp = timestamp;
    odom_path_.poses.emplace_back(eigenToPoseStamped(node.pose_, map_frame_, timestamp));
    corrected_path_.poses.emplace_back(
        eigenToPoseStamped(node.pose_corrected_, map_frame_, timestamp));
    return;
}

void PoseGraphManager::callbackNode(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
                                    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &scan_msg)
{
    if (!PoseGraphNode::hasValidMessage(*odom_msg, *scan_msg, max_sync_interval_))
    {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Dropping invalid synchronized odometry and cloud message.");
        return;
    }

    pcl::PointCloud<PointType> scan;
    pcl::fromROSMsg(*scan_msg, scan);
    if (!PoseGraphNode::hasValidInput(*odom_msg, *scan_msg, scan, max_sync_interval_))
    {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Dropping synchronized odometry and cloud with invalid point data.");
        return;
    }

    // NOTE(hlim): For clarification, 'current' refers to the real-time incoming messages,
    // while 'latest' indicates the last keyframe information already appended to keyframes_.
    Eigen::Matrix4d current_odom = current_frame_.pose_;
    current_frame_ = PoseGraphNode(
        *odom_msg, std::move(scan), latest_keyframe_idx_, scan_voxel_res_, store_voxelized_scan_);

    kiss_matcher::TicToc total_timer;
    kiss_matcher::TicToc local_timer;

    visualizeCurrentData(current_odom, odom_msg->header.stamp, scan_msg->header.frame_id);

    if (!is_initialized_.load())
    {
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            keyframes_.push_back(current_frame_);
        }
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            appendKeyframePose(current_frame_);
        }

        auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise =
            gtsam::noiseModel::Diagonal::Variances(variance_vector);

        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            gtsam_graph_.add(
                gtsam::PriorFactor<gtsam::Pose3>(0, eigenToGtsam(current_frame_.pose_), prior_noise));
            init_esti_.insert(latest_keyframe_idx_, eigenToGtsam(current_frame_.pose_));
        }

        ++latest_keyframe_idx_;
        is_initialized_.store(true);

        RCLCPP_INFO(this->get_logger(), "The first node comes. Initialization complete.");
    }
    else
    {
        const auto t_keyframe_processing = local_timer.toc();
        bool keyframe_was_added = false;
        {
            std::lock_guard<std::mutex> graph_lock(graph_mutex_);
            std::lock_guard<std::mutex> keyframes_lock(keyframes_mutex_);
            if (keyframes_.empty())
            {
                return;
            }

            const PoseGraphNode &latest_keyframe = keyframes_.back();
            if (!checkIfKeyframe(current_frame_, latest_keyframe))
            {
                return;
            }

            const size_t new_keyframe_idx  = latest_keyframe_idx_;
            const size_t prev_keyframe_idx  = new_keyframe_idx - 1;
            const gtsam::Pose3 pose_from    = eigenToGtsam(latest_keyframe.pose_corrected_);
            const gtsam::Pose3 pose_to      = eigenToGtsam(current_frame_.pose_corrected_);

            auto variance_vector =
                (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
            gtsam::noiseModel::Diagonal::shared_ptr odom_noise =
                gtsam::noiseModel::Diagonal::Variances(variance_vector);

            keyframes_.push_back(current_frame_);
            gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_keyframe_idx,
                                                                new_keyframe_idx,
                                                                pose_from.between(pose_to),
                                                                odom_noise));
            init_esti_.insert(new_keyframe_idx, pose_to);
            pending_keyframe_update_ = true;

            ++latest_keyframe_idx_;
            keyframe_was_added = true;
        }

        if (keyframe_was_added)
        {
            {
                std::lock_guard<std::mutex> lock(vis_mutex_);
                appendKeyframePose(current_frame_);
            }

            local_timer.tic();
            applyPendingGraphUpdate();
            const auto t_optim = local_timer.toc();

            const auto t_total = total_timer.toc();
            size_t keyframe_count = 0;
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                keyframe_count = keyframes_.size();
            }

            RCLCPP_INFO(
                this->get_logger(),
                "# of Keyframes: %zu. Timing (msec) → Total: %.1f | Keyframe: %.1f | Optim.: %.1f",
                keyframe_count,
                t_total,
                t_keyframe_processing,
                t_optim);
        }
    }
}

void PoseGraphManager::applyPendingGraphUpdate()
{
    bool loop_closure_was_added = false;
    bool keyframe_was_added     = false;
    gtsam::Values corrected_esti_copied;
    {
        std::lock_guard<std::mutex> lock(graph_mutex_);
        loop_closure_was_added = loop_closure_added_.exchange(false);
        keyframe_was_added     = pending_keyframe_update_;
        isam_handler_->update(gtsam_graph_, init_esti_);
        isam_handler_->update();
        if (loop_closure_was_added)
        {
            isam_handler_->update();
            isam_handler_->update();
            isam_handler_->update();
        }
        gtsam_graph_.resize(0);
        init_esti_.clear();
        pending_keyframe_update_ = false;
        corrected_esti_copied    = isam_handler_->calculateEstimate();

        {
            std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
            corrected_esti_ = corrected_esti_copied;
            last_corrected_pose_ =
                gtsamToEigen(corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size() - 1));
            if (keyframe_was_added)
            {
                odom_delta_ = Eigen::Matrix4d::Identity();
            }
        }

        if (loop_closure_was_added)
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            const size_t num_updates = std::min(corrected_esti_copied.size(), keyframes_.size());
            for (size_t i = 0; i < num_updates; ++i)
            {
                keyframes_[i].pose_corrected_ =
                    gtsamToEigen(corrected_esti_copied.at<gtsam::Pose3>(i));
            }
        }
    }
}

void PoseGraphManager::buildMap()
{
    static size_t start_idx = 0;

    if (map_pub_->get_subscription_count() > 0)
    {
        rclcpp::Time latest_map_timestamp(0, 0, RCL_ROS_TIME);
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            if (need_map_update_.exchange(false))
            {
                map_cloud_->clear();
                start_idx = 0;
            }

            if (keyframes_.empty())
            {
                return;
            }

            // NOTE(hlim): Building the full map causes RViz delay when keyframes > 500.
            // Since the map is for visualization only, we apply a heuristic to reduce cost.
            for (size_t i = start_idx; i < keyframes_.size(); ++i)
            {
                const auto &i_th_scan = [&]()
                                        {
                                            // It's already voxelized
                                            if (store_voxelized_scan_)
                                            {
                                                return keyframes_[i].scan_;
                                            }

                                            if (keyframes_[i].voxelized_scan_.empty())
                                            {
                                                keyframes_[i].voxelized_scan_ =
                                                    *voxelize(keyframes_[i].scan_, scan_voxel_res_);
                                            }
                                            return keyframes_[i].voxelized_scan_;
                                        }();

                *map_cloud_ += transformPcd(i_th_scan, keyframes_[i].pose_corrected_);
            }

            start_idx = keyframes_.size();
            latest_map_timestamp = rclcpp::Time(keyframes_.back().timestamp_);
        }

        const auto &voxelized_map = voxelize(map_cloud_, map_voxel_res_);
        map_pub_->publish(toROSMsg(*voxelized_map, map_frame_, latest_map_timestamp));
    }
}

void PoseGraphManager::detectLoopClosureByLoopDetector()
{
    const auto loop_correction_generation = loopSearchGeneration();
    if (!loop_correction_generation)
    {
        return;
    }

    LoopIdxPairs loop_idx_pairs;
    size_t query_idx = 0;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        if (!is_initialized_.load() || keyframes_.empty())
        {
            return;
        }
        if (!pending_loop_detector_query_idx_ && pending_nnsearch_query_idx_)
        {
            return;
        }

        query_idx = pending_loop_detector_query_idx_.value_or(keyframes_.back().idx_);
        if (query_idx >= keyframes_.size())
        {
            pending_loop_detector_query_idx_.reset();
            return;
        }

        auto &query = keyframes_[query_idx];
        if (!pending_loop_detector_query_idx_.has_value() && query.loop_detector_processed_)
        {
            return;
        }
        query.loop_detector_processed_ = true;

        loop_idx_pairs = loop_detector_->fetchLoopCandidates(query, keyframes_);
    }

    const auto enqueue_result =
        enqueueLoopCandidates(loop_idx_pairs,
                              kiss_matcher::LoopCandidateSource::kLoopDetector,
                              *loop_correction_generation);
    if (enqueue_result == LoopCandidateEnqueueResult::kEnqueued)
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        pending_loop_detector_query_idx_.reset();
        return;
    }
    if (enqueue_result == LoopCandidateEnqueueResult::kQueueFull)
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        pending_loop_detector_query_idx_ = query_idx;
        return;
    }
    if (enqueue_result == LoopCandidateEnqueueResult::kBatchTooLarge)
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        pending_loop_detector_query_idx_.reset();
        return;
    }

    if (enqueue_result == LoopCandidateEnqueueResult::kSearchBlocked)
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        if (query_idx < keyframes_.size())
        {
            keyframes_[query_idx].loop_detector_processed_ = false;
        }
        pending_loop_detector_query_idx_.reset();
    }
}

void PoseGraphManager::detectLoopClosureByNNSearch()
{
    const auto loop_correction_generation = loopSearchGeneration();
    if (!loop_correction_generation)
    {
        return;
    }

    const bool has_accepted_loop = hasAcceptedLoop();
    LoopIdxPairs loop_idx_pairs;
    size_t query_idx = 0;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        if (!is_initialized_.load() || keyframes_.empty())
        {
            return;
        }
        if (!pending_nnsearch_query_idx_ && pending_loop_detector_query_idx_)
        {
            return;
        }

        query_idx = pending_nnsearch_query_idx_.value_or(keyframes_.back().idx_);
        if (query_idx >= keyframes_.size())
        {
            pending_nnsearch_query_idx_.reset();
            return;
        }

        auto &query = keyframes_[query_idx];
        if (!pending_nnsearch_query_idx_.has_value() && query.nnsearch_processed_)
        {
            return;
        }
        query.nnsearch_processed_ = true;
    }

    {
        std::lock_guard<std::mutex> lc_lock(lc_mutex_);
        std::lock_guard<std::mutex> keyframes_lock(keyframes_mutex_);
        if (query_idx >= keyframes_.size())
        {
            pending_nnsearch_query_idx_.reset();
            return;
        }
        const auto &query = keyframes_[query_idx];
        loop_idx_pairs = has_accepted_loop
                             ? loop_closure_->fetchClosestLoopCandidate(query, keyframes_)
                             : loop_closure_->fetchLoopCandidates(query, keyframes_);
    }

    const auto enqueue_result =
        enqueueLoopCandidates(loop_idx_pairs,
                              kiss_matcher::LoopCandidateSource::kNNSearch,
                              *loop_correction_generation);
    if (enqueue_result == LoopCandidateEnqueueResult::kEnqueued)
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        pending_nnsearch_query_idx_.reset();
        return;
    }
    if (enqueue_result == LoopCandidateEnqueueResult::kQueueFull)
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        pending_nnsearch_query_idx_ = query_idx;
        return;
    }
    if (enqueue_result == LoopCandidateEnqueueResult::kBatchTooLarge)
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        pending_nnsearch_query_idx_.reset();
        return;
    }

    if (enqueue_result == LoopCandidateEnqueueResult::kSearchBlocked)
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        if (query_idx < keyframes_.size())
        {
            keyframes_[query_idx].nnsearch_processed_ = false;
        }
        pending_nnsearch_query_idx_.reset();
    }
}

std::optional<uint64_t> PoseGraphManager::loopSearchGeneration()
{
    std::lock_guard<std::mutex> lock(loop_queue_mutex_);
    if (loopSearchBlockedLocked())
    {
        return std::nullopt;
    }
    return loop_correction_generation_;
}

bool PoseGraphManager::loopSearchBlockedLocked() const
{
    return loop_correction_in_progress_ ||
           (has_accepted_loop_ &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          last_accepted_loop_time_)
                    .count() < loop_candidate_cooldown_);
}

bool PoseGraphManager::hasAcceptedLoop()
{
    std::lock_guard<std::mutex> lock(loop_queue_mutex_);
    return has_accepted_loop_;
}

PoseGraphManager::LoopCandidateEnqueueResult PoseGraphManager::enqueueLoopCandidates(
    const LoopIdxPairs &loop_idx_pairs,
    const kiss_matcher::LoopCandidateSource source,
    const uint64_t loop_correction_generation)
{
    std::lock_guard<std::mutex> lock(loop_queue_mutex_);
    if (loopSearchBlockedLocked() || loop_correction_generation != loop_correction_generation_)
    {
        return LoopCandidateEnqueueResult::kSearchBlocked;
    }

    if (loop_idx_pairs.size() > loop_candidate_queue_.capacity())
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Dropping loop candidate batch of %zu: queue capacity is %zu.",
            loop_idx_pairs.size(),
            loop_candidate_queue_.capacity());
        return LoopCandidateEnqueueResult::kBatchTooLarge;
    }

    if (!loop_candidate_queue_.enqueue(loop_idx_pairs, source))
    {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             5000,
                             "Loop candidate queue is full (%zu); retrying the current query later.",
                             loop_candidate_queue_.size());
        return LoopCandidateEnqueueResult::kQueueFull;
    }
    return LoopCandidateEnqueueResult::kEnqueued;
}

void PoseGraphManager::beginLoopCorrection()
{
    std::vector<kiss_matcher::QueuedLoopCandidate> discarded_candidates;
    {
        std::lock_guard<std::mutex> lock(loop_queue_mutex_);
        // Pending candidates were generated against the pre-correction trajectory.
        discarded_candidates = loop_candidate_queue_.takeAll();
        loop_correction_in_progress_ = true;
        ++loop_correction_generation_;
    }

    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    const auto reset_pending_query = [this](const std::optional<size_t> &pending_query_idx,
                                            const bool is_loop_detector)
    {
        if (!pending_query_idx || *pending_query_idx >= keyframes_.size())
        {
            return;
        }

        if (is_loop_detector)
        {
            keyframes_[*pending_query_idx].loop_detector_processed_ = false;
        }
        else
        {
            keyframes_[*pending_query_idx].nnsearch_processed_ = false;
        }
    };
    reset_pending_query(pending_loop_detector_query_idx_, true);
    reset_pending_query(pending_nnsearch_query_idx_, false);
    pending_loop_detector_query_idx_.reset();
    pending_nnsearch_query_idx_.reset();

    for (const auto &candidate : discarded_candidates)
    {
        const size_t query_idx = candidate.indices_.first;
        if (query_idx >= keyframes_.size())
        {
            continue;
        }

        if (candidate.source_ == kiss_matcher::LoopCandidateSource::kLoopDetector)
        {
            keyframes_[query_idx].loop_detector_processed_ = false;
        }
        else
        {
            keyframes_[query_idx].nnsearch_processed_ = false;
        }
    }
}

void PoseGraphManager::startLoopSearchCooldown()
{
    std::lock_guard<std::mutex> lock(loop_queue_mutex_);
    last_accepted_loop_time_       = std::chrono::steady_clock::now();
    has_accepted_loop_             = true;
    loop_correction_in_progress_   = false;
}

void PoseGraphManager::performRegistration()
{
    std::lock_guard<std::mutex> registration_lock(registration_mutex_);
    kiss_matcher::TicToc reg_timer;
    kiss_matcher::QueuedLoopCandidate queued_candidate;
    {
        std::lock_guard<std::mutex> lock(loop_queue_mutex_);
        if (!loop_candidate_queue_.tryPop(queued_candidate))
        {
            return;
        }
    }
    const auto [query_idx, match_idx] = queued_candidate.indices_;

    std::vector<PoseGraphNode> keyframes_snapshot;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        if (query_idx >= keyframes_.size() || match_idx >= keyframes_.size())
        {
            RCLCPP_WARN(this->get_logger(),
                        "LC skipped. Invalid keyframe indices: query=%zu, match=%zu, size=%zu",
                        query_idx,
                        match_idx,
                        keyframes_.size());
            return;
        }
        keyframes_snapshot = keyframes_;
    }

    RegOutput reg_output;
    {
        std::lock_guard<std::mutex> lock(lc_mutex_);
        reg_output = loop_closure_->performLoopClosure(keyframes_snapshot, query_idx, match_idx);
        succeeded_query_idx_ = query_idx;
    }
    need_lc_cloud_vis_update_.store(true);

    if (reg_output.is_valid_)
    {
        RCLCPP_INFO(this->get_logger(), "LC accepted. Overlapness: %.3f", reg_output.overlapness_);
        beginLoopCorrection();
        gtsam::Pose3 pose_from =
            eigenToGtsam(reg_output.pose_ * keyframes_snapshot[query_idx].pose_corrected_);
        gtsam::Pose3 pose_to = eigenToGtsam(keyframes_snapshot[match_idx].pose_corrected_);

        // TODO(hlim): Parameterize
        auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
        gtsam::noiseModel::Diagonal::shared_ptr loop_noise =
            gtsam::noiseModel::Diagonal::Variances(variance_vector);

        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
                                 query_idx, match_idx, pose_from.between(pose_to), loop_noise));
            loop_closure_added_.store(true);
        }
        applyPendingGraphUpdate();
        startLoopSearchCooldown();

        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            vis_loop_edges_.emplace_back(query_idx, match_idx);
        }
        need_map_update_.store(true);
        need_graph_vis_update_.store(true);

        // --------------------------------------------------
        // TODO(hlim): resurrect pose_graph_tools_msgs
        // pose_graph_tools_msgs::msg::PoseGraphEdge edge;
        // double lidar_end_time_compensation = 0.1;
        // edge.header.stamp = this->now();
        // edge.robot_from = 0;
        // edge.robot_to = 0;
        // edge.type = 1;

        // edge.key_to = static_cast<uint64_t>(
        //   (keyframes_.back().timestamp_ - lidar_end_time_compensation) * 1e9);
        // edge.key_from = static_cast<uint64_t>(
        //   (keyframes_[closest_keyframe_idx].timestamp_ - lidar_end_time_compensation) * 1e9);

        // Eigen::Matrix4d pose_inv = pose_to.matrix().inverse() * pose_from.matrix();
        // edge.pose = poseEigToPoseGeo(pose_inv);
        // loop_msgs_.edges.emplace_back(edge);
        // last_lc_time_ = this->now().seconds();
        // --------------------------------------------------
    }
    else
    {
        if (reg_output.overlapness_ == 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "LC rejected. KISS-Matcher failed");
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(), "LC rejected. Overlapness: %.3f", reg_output.overlapness_);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Reg: %.1f msec", reg_timer.toc());
}

void PoseGraphManager::visualizeCurrentData(const Eigen::Matrix4d &current_odom,
                                            const rclcpp::Time &timestamp,
                                            const std::string &frame_id)
{
    // NOTE(hlim): Instead of visualizing only when adding keyframes (node-wise), which can feel
    // choppy, we visualize the current frame every cycle to ensure smoother, real-time
    // visualization.
    {
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        odom_delta_ = odom_delta_ * current_odom.inverse() * current_frame_.pose_;
        current_frame_.pose_corrected_ = last_corrected_pose_ * odom_delta_;

        geometry_msgs::msg::PoseStamped ps =
            eigenToPoseStamped(current_frame_.pose_corrected_, map_frame_, timestamp);
        realtime_pose_pub_->publish(ps);

        Eigen::Matrix4d tf_pose = current_frame_.pose_corrected_;
        std::string child_frame = base_frame_.empty() ? frame_id : base_frame_;
        bool should_publish_tf  = true;
        if (publish_map_to_odom_tf_)
        {
            if (odom_frame_.empty() || odom_frame_ == map_frame_)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    5000,
                    "Skipping map->odom TF publish: odom_frame is empty or equals map_frame.");
                should_publish_tf = false;
            }
            else
            {
                // The pose graph estimates map->base. For the standard SLAM TF tree,
                // publish map->odom and let the front end remain the sole owner of odom->base.
                tf_pose     = current_frame_.pose_corrected_ * current_frame_.pose_.inverse();
                child_frame = odom_frame_;
            }
        }

        if (should_publish_tf)
        {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp    = timestamp;
            transform_stamped.header.frame_id = map_frame_;
            transform_stamped.child_frame_id  = child_frame;
            Eigen::Quaterniond q(tf_pose.block<3, 3>(0, 0));
            transform_stamped.transform.translation.x = tf_pose(0, 3);
            transform_stamped.transform.translation.y = tf_pose(1, 3);
            transform_stamped.transform.translation.z = tf_pose(2, 3);
            transform_stamped.transform.rotation.x    = q.x();
            transform_stamped.transform.rotation.y    = q.y();
            transform_stamped.transform.rotation.z    = q.z();
            transform_stamped.transform.rotation.w    = q.w();
            tf_broadcaster_->sendTransform(transform_stamped);
        }
    }

    scan_pub_->publish(toROSMsg(
                           transformPcd(current_frame_.scan_, current_frame_.pose_corrected_), map_frame_, timestamp));

    bool has_latest_position = false;
    geometry_msgs::msg::Point latest_position;
    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        if (!corrected_path_.poses.empty())
        {
            latest_position      = corrected_path_.poses.back().pose.position;
            has_latest_position = true;
        }
    }
    if (has_latest_position)
    {
        loop_detection_radius_pub_->publish(visualizeLoopDetectionRadius(latest_position));
    }
}

void PoseGraphManager::visualizePoseGraph()
{
    if (!is_initialized_.load())
    {
        return;
    }

    if (need_graph_vis_update_.exchange(false))
    {
        gtsam::Values corrected_esti_copied;
        pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
        nav_msgs::msg::Path corrected_path;
        std::vector<builtin_interfaces::msg::Time> keyframe_timestamps;

        {
            std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
            corrected_esti_copied = corrected_esti_;
        }
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            keyframe_timestamps.reserve(keyframes_.size());
            for (const auto &keyframe : keyframes_)
            {
                keyframe_timestamps.push_back(keyframe.timestamp_);
            }
        }

        corrected_path.header.frame_id = map_frame_;
        const size_t corrected_pose_count =
            std::min(corrected_esti_copied.size(), keyframe_timestamps.size());
        for (size_t i = 0; i < corrected_pose_count; ++i)
        {
            gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
            corrected_odoms.points.emplace_back(
                pose_.translation().x(), pose_.translation().y(), pose_.translation().z());

            const rclcpp::Time timestamp(keyframe_timestamps[i]);
            corrected_path.poses.push_back(gtsamToPoseStamped(pose_, map_frame_, timestamp));
            corrected_path.header.stamp = timestamp;
        }
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            if (!vis_loop_edges_.empty())
            {
                loop_detection_pub_->publish(visualizeLoopMarkers(corrected_esti_copied));
            }
            corrected_odoms_      = corrected_odoms;
            corrected_path_.header.stamp = corrected_path.header.stamp;
            corrected_path_.poses = corrected_path.poses;
        }
    }

    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        path_pub_->publish(odom_path_);
        corrected_path_pub_->publish(corrected_path_);
    }
}

void PoseGraphManager::visualizeLoopClosureClouds()
{
    if (!need_lc_cloud_vis_update_.exchange(false))
    {
        return;
    }

    size_t succeeded_query_idx = 0;
    pcl::PointCloud<PointType> source_cloud;
    pcl::PointCloud<PointType> target_cloud;
    pcl::PointCloud<PointType> final_aligned_cloud;
    pcl::PointCloud<PointType> coarse_aligned_cloud;
    pcl::PointCloud<PointType> debug_cloud;
    {
        std::lock_guard<std::mutex> lock(lc_mutex_);
        succeeded_query_idx = succeeded_query_idx_;
        source_cloud         = loop_closure_->getSourceCloud();
        target_cloud         = loop_closure_->getTargetCloud();
        final_aligned_cloud  = loop_closure_->getFinalAlignedCloud();
        coarse_aligned_cloud = loop_closure_->getCoarseAlignedCloud();
        debug_cloud          = loop_closure_->getDebugCloud();
    }

    rclcpp::Time query_timestamp;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        if (succeeded_query_idx >= keyframes_.size())
        {
            RCLCPP_WARN(this->get_logger(),
                        "Skipping LC cloud visualization. Invalid query index: %zu",
                        succeeded_query_idx);
            return;
        }
        query_timestamp = rclcpp::Time(keyframes_[succeeded_query_idx].timestamp_);
    }

    debug_src_pub_->publish(toROSMsg(source_cloud, map_frame_, query_timestamp));
    debug_tgt_pub_->publish(toROSMsg(target_cloud, map_frame_, query_timestamp));
    debug_fine_aligned_pub_->publish(toROSMsg(final_aligned_cloud, map_frame_, query_timestamp));
    debug_coarse_aligned_pub_->publish(toROSMsg(coarse_aligned_cloud, map_frame_, query_timestamp));
    debug_cloud_pub_->publish(toROSMsg(debug_cloud, map_frame_, query_timestamp));
}

visualization_msgs::msg::Marker PoseGraphManager::visualizeLoopMarkers(
    const gtsam::Values &corrected_poses) const
{
    visualization_msgs::msg::Marker edges;
    edges.type               = visualization_msgs::msg::Marker::LINE_LIST;
    edges.scale.x            = 0.12f;
    edges.header.frame_id    = map_frame_;
    edges.pose.orientation.w = 1.0f;
    edges.color.r            = 1.0f;
    edges.color.g            = 1.0f;
    edges.color.b            = 1.0f;
    edges.color.a            = 1.0f;

    for (size_t i = 0; i < vis_loop_edges_.size(); ++i)
    {
        if (vis_loop_edges_[i].first >= corrected_poses.size() ||
            vis_loop_edges_[i].second >= corrected_poses.size())
        {
            continue;
        }
        gtsam::Pose3 pose  = corrected_poses.at<gtsam::Pose3>(vis_loop_edges_[i].first);
        gtsam::Pose3 pose2 = corrected_poses.at<gtsam::Pose3>(vis_loop_edges_[i].second);

        geometry_msgs::msg::Point p, p2;
        p.x  = pose.translation().x();
        p.y  = pose.translation().y();
        p.z  = pose.translation().z();
        p2.x = pose2.translation().x();
        p2.y = pose2.translation().y();
        p2.z = pose2.translation().z();

        edges.points.push_back(p);
        edges.points.push_back(p2);
    }
    return edges;
}

visualization_msgs::msg::Marker PoseGraphManager::visualizeLoopDetectionRadius(
    const geometry_msgs::msg::Point &latest_position) const
{
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = map_frame_;
    sphere.id              = 100000;  // arbitrary number
    sphere.type            = visualization_msgs::msg::Marker::SPHERE;
    sphere.pose.position.x = latest_position.x;
    sphere.pose.position.y = latest_position.y;
    sphere.pose.position.z = latest_position.z;
    sphere.scale.x         = 2 * loop_detection_radius_;
    sphere.scale.y         = 2 * loop_detection_radius_;
    sphere.scale.z         = 2 * loop_detection_radius_;
    // Use transparanet cyan color
    sphere.color.r = 0.0;
    sphere.color.g = 0.824;
    sphere.color.b = 1.0;
    sphere.color.a = 0.5;

    return sphere;
}

bool PoseGraphManager::checkIfKeyframe(const PoseGraphNode &query_node,
                                       const PoseGraphNode &latest_node)
{
    return keyframe_thr_ < (latest_node.pose_corrected_.block<3, 1>(0, 3) -
                            query_node.pose_corrected_.block<3, 1>(0, 3))
           .norm();
}

void PoseGraphManager::saveFlagCallback(const std_msgs::msg::String::ConstSharedPtr &msg)
{
    std::string save_dir        = !msg->data.empty() ? msg->data : package_path_;
    std::string seq_directory   = save_dir + "/" + seq_name_;
    std::string scans_directory = seq_directory + "/scans";

    if (save_in_kitti_format_)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Scans are saved in %s, following the KITTI and TUM format",
                    scans_directory.c_str());

        if (fs::exists(seq_directory))
        {
            fs::remove_all(seq_directory);
        }
        fs::create_directories(scans_directory);

        std::ofstream kitti_pose_file(seq_directory + "/poses_kitti.txt");
        std::ofstream tum_pose_file(seq_directory + "/poses_tum.txt");
        tum_pose_file << "#timestamp x y z qx qy qz qw\n";

        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                std::stringstream ss_;
                ss_ << scans_directory << "/" << std::setw(6) << std::setfill('0') << i << ".pcd";
                RCLCPP_INFO(this->get_logger(), "Saving %s...", ss_.str().c_str());
                pcl::io::savePCDFileASCII<PointType>(ss_.str(), keyframes_[i].scan_);

                const auto &pose_ = keyframes_[i].pose_corrected_;
                kitti_pose_file << pose_(0, 0) << " " << pose_(0, 1) << " " << pose_(0, 2) << " "
                                << pose_(0, 3) << " " << pose_(1, 0) << " " << pose_(1, 1) << " "
                                << pose_(1, 2) << " " << pose_(1, 3) << " " << pose_(2, 0) << " "
                                << pose_(2, 1) << " " << pose_(2, 2) << " " << pose_(2, 3) << "\n";

                const rclcpp::Time timestamp(keyframes_[i].timestamp_);
                const auto &lidar_optim_pose_ =
                    eigenToPoseStamped(keyframes_[i].pose_corrected_,
                                       map_frame_,
                                       timestamp);
                tum_pose_file << keyframes_[i].timestamp_.sec << "." << std::setw(9)
                              << std::setfill('0') << keyframes_[i].timestamp_.nanosec << " "
                              << std::setfill(' ') << std::fixed << std::setprecision(8)
                              << lidar_optim_pose_.pose.position.x << " "
                              << lidar_optim_pose_.pose.position.y << " "
                              << lidar_optim_pose_.pose.position.z << " "
                              << lidar_optim_pose_.pose.orientation.x << " "
                              << lidar_optim_pose_.pose.orientation.y << " "
                              << lidar_optim_pose_.pose.orientation.z << " "
                              << lidar_optim_pose_.pose.orientation.w << "\n";
            }
        }
        kitti_pose_file.close();
        tum_pose_file.close();
        RCLCPP_INFO(this->get_logger(), "Scans and poses saved in .pcd and KITTI format");
    }
    if (save_map_bag_)
    {
        RCLCPP_INFO(this->get_logger(),
                    "NOTE(hlim): rosbag2 saving not directly implemented; skipping.");
    }
    if (save_map_pcd_)
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());

        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            if (keyframes_.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No keyframes available; skipping map PCD save.");
                return;
            }
            corrected_map->reserve(keyframes_[0].scan_.size() * keyframes_.size());
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].scan_, keyframes_[i].pose_corrected_);
            }
        }
        const auto &voxelized_map = voxelize(corrected_map, save_voxel_res_);
        pcl::io::savePCDFileASCII<PointType>(seq_directory + "/" + seq_name_ + "_map.pcd",
                                             *voxelized_map);
        RCLCPP_INFO(this->get_logger(), "Accumulated map cloud saved in .pcd format");
    }
}

// ----------------------------------------------------------------------

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto node = std::make_shared<PoseGraphManager>(options);

    // To allow timer callbacks to run concurrently using multiple threads
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
