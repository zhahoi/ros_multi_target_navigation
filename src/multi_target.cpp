#include <navigation_test/multi_target.h>

namespace robot_ctrl {
    NavigationMultiTarget::NavigationMultiTarget(void) : 
        last_goal_time_(ros::Time::now()) {
        
        ros::NodeHandle param_n("~");
        
        // 读取目标点的数量
        param_n.param("pose_number", pose_number_, 4);
        goal_pose_list_.resize(pose_number_);

        // 读取目标点
        XmlRpc::XmlRpcValue goal_poses_param;
        if (param_n.getParam("goal_poses", goal_poses_param)) {
            if (goal_poses_param.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                for (int i = 0; i < goal_poses_param.size(); ++i) {
                    if (goal_poses_param[i].hasMember("x") && goal_poses_param[i].hasMember("y") && goal_poses_param[i].hasMember("theta")) {
                        goal_pose_list_[i].x = static_cast<double>(goal_poses_param[i]["x"]);
                        goal_pose_list_[i].y = static_cast<double>(goal_poses_param[i]["y"]);
                        goal_pose_list_[i].theta = static_cast<double>(goal_poses_param[i]["theta"]);
                        ROS_INFO("Loaded pose%d: x = %.2f, y = %.2f, theta = %.2f", 
                                 i + 1, goal_pose_list_[i].x, goal_pose_list_[i].y, goal_pose_list_[i].theta);
                    }
                }
            }
        } else {
            ROS_ERROR("Failed to load goal_poses from parameter server.");
            return;
        }

        // 确保目标点的数量与列表的大小匹配
        if (pose_number_ != goal_pose_list_.size()) {
            ROS_ERROR("Number of poses in goal_pose_list does not match pose_number!");
            return;
        }

        // 读取距离和角度阈值
        param_n.param("position_tolerance", position_tolerance_, 0.3);  // 位置阈值，单位：米
        param_n.param("angle_tolerance_deg", angle_tolerance_deg_, 15.0);      // 角度阈值，单位：度
        angle_tolerance_ = angle_tolerance_deg_ * M_PI / 180.0;   // 转换为弧度
        
        // 初始化MoveBaseClient
        ac_ = new MoveBaseClient("move_base", true);
    }

    NavigationMultiTarget::~NavigationMultiTarget(void) {
        delete ac_;
    }

    void NavigationMultiTarget::navigationRun(void) {
        // 如果已完成目标点导航，不再继续
        if (completed_) {
            ROS_INFO("Navigation completed. Stopping further movement.");
            return;
        }

        // 如果目标点被取消、完成或者失败，进行下一步导航
        if (navi_status_ == CANCEL_NAVI || navi_status_ == NAVI_SUCCESS || navi_status_ == NAVI_FAILED) { 

#ifndef LOOP_NAVIGATION
            // 如果所有目标点都已经到达，则停止
            if (index_current_ >= pose_number_) {
                completed_ = true;  // 只有当所有目标点都到达后才设置完成
                ROS_INFO("All targets reached. Stopping navigation.");
                return;
            }
#endif
            // 获取当前目标点
            geometry_msgs::Pose2D goal_pose = goal_pose_list_[index_current_];
            int status = setGoal(goal_pose); // 设置当前目标
            if (status) {
                navi_status_ = START_NAVI; // 设置导航状态为开始
                last_goal_time_ = ros::Time::now();  // 记录当前目标点的时间
            }
        }

        // 检查目标是否超时，如果超时跳过当前目标点
        if (ros::Time::now() - last_goal_time_ > max_goal_time_) {
            ROS_WARN("Goal %d timeout! Skipping to next target.", index_current_);
            // 超时跳过当前目标点
            navi_status_ = NAVI_FAILED;
#ifdef LOOP_NAVIGATION
            index_current_ = (index_current_ + 1) % pose_number_;  // 跳到下一个目标点（循环）
#else
            index_current_++;  // 跳到下一个目标点
#endif
            last_goal_time_ = ros::Time::now();  // 更新最后目标时间
        }
    }

    void NavigationMultiTarget::activeCb(void) {
        ROS_INFO("Goal just went active!");
        navi_status_ = ON_NAVI;
    }

    void NavigationMultiTarget::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
        // 获取当前位姿
        const auto& current_pose = feedback->base_position.pose;
        
        // 当前目标点索引检查
        if(index_current_ >= goal_pose_list_.size()) return;

        // 计算位置和角度误差
        const auto& target = goal_pose_list_[index_current_];
        double position_distance = computePositionDistance(current_pose.position, target);
        double angle_diff = computeAngleDifference(current_pose.orientation, target.theta);

        // 阈值判断
        if(position_distance <= position_tolerance_ && angle_diff <= angle_tolerance_) {
            ROS_INFO("Threshold reached! Position error: %.2f m, Angle error: %.1f degree", 
                    position_distance, angle_diff * 180 / M_PI);
            threshold_reached_ = true;
            ac_->cancelGoal(); // 触发提前终止
        }
    }

    void NavigationMultiTarget::doneCb(const actionlib::SimpleClientGoalState& state,
                                    const move_base_msgs::MoveBaseResultConstPtr& result) {   
        if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED || threshold_reached_) {
            ROS_INFO("Navigation success!");
            navi_status_ = NAVI_SUCCESS;
#ifdef LOOP_NAVIGATION
            index_current_ = (index_current_ + 1) % pose_number_;  // 跳到下一个目标点（循环）
#else
            index_current_++;  // 跳到下一个目标点
#endif
            threshold_reached_ = false; // 重置标志
        } else {
            ROS_WARN("Navigation failed! State: %s", state.toString().c_str());
            navi_status_ = NAVI_FAILED;
#ifdef LOOP_NAVIGATION
            index_current_ = (index_current_ + 1) % pose_number_;  // 失败仍继续（循环）
#else
            index_current_++;  // 失败仍继续（可根据需求修改）
#endif
        }
        last_goal_time_ = ros::Time::now();
    }

    bool NavigationMultiTarget::setGoal(geometry_msgs::Pose2D goal_pose) {    
        geometry_msgs::Quaternion pose_orientation = tf::createQuaternionMsgFromYaw(goal_pose.theta * M_PI / 180);

        if (!ac_->waitForServer(ros::Duration(1.0))) {
            ROS_WARN("Connected to move base server fail when sending goal");
            return false;
        }

        ROS_INFO("Setting goal: x = %.2f, y = %.2f, theta = %.2f", goal_pose.x, goal_pose.y, goal_pose.theta);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = goal_pose.x;
        goal.target_pose.pose.position.y = goal_pose.y;
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation = pose_orientation;

        ac_->sendGoal(goal, 
                      boost::bind(&NavigationMultiTarget::doneCb, this, _1, _2), 
                      boost::bind(&NavigationMultiTarget::activeCb, this), 
                      boost::bind(&NavigationMultiTarget::feedbackCb, this, _1)); 
        return true;
    }

    void NavigationMultiTarget::cancelGoal(void) {
        ac_->cancelGoal();
    }

    double NavigationMultiTarget::computePositionDistance(const geometry_msgs::Point& current_pos, const geometry_msgs::Pose2D& target) {
        double dx = current_pos.x - target.x;
        double dy = current_pos.y - target.y;
        return std::hypot(dx, dy);
    }

    double NavigationMultiTarget::computeAngleDifference(const geometry_msgs::Quaternion& current_orientation, double target_theta) {
        double current_yaw = tf::getYaw(current_orientation);
        double target_yaw = target_theta * M_PI / 180.0;
        double angle_diff = std::abs(current_yaw - target_yaw);
        angle_diff = std::fmod(angle_diff, 2 * M_PI);
        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
        return angle_diff;
    }
}
