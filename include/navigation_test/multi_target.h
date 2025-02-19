#ifndef _MULTI_TARGET_HPP_
#define _MULTI_TARGET_HPP_
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h> 
#include <vector>
#include <XmlRpcValue.h>
#include <memory>

#define LOOP_NAVIGATION   // 用于判定是否循环执行目标点的标志

namespace robot_ctrl {
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    
    enum NaviStatus
    {
        CANCEL_NAVI         = 0,    // 结束导航
        START_NAVI          = 1,
        ON_NAVI             = 2,    // 导航中
        NAVI_SUCCESS        = 3,    // 到达目标点 
        NAVI_FAILED         = 4
    };

    class NavigationMultiTarget {
    public:
        NavigationMultiTarget(void);
        ~NavigationMultiTarget(void);

        void navigationRun(void);
        bool setGoal(geometry_msgs::Pose2D goal_pose);
        void cancelGoal(void);

    private:
        void activeCb(void);
        void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
        void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
        double computeAngleDifference(const geometry_msgs::Quaternion& current_orientation, double target_theta);
        double computePositionDistance(const geometry_msgs::Point& current_pos, const geometry_msgs::Pose2D& target);

    private:
        MoveBaseClient* ac_;                        // for set navi goal
        NaviStatus navi_status_ = CANCEL_NAVI;  // navi status
        std::vector<geometry_msgs::Pose2D> goal_pose_list_;

        // New variables for handling goal timeout, retries, and navigation state
        ros::Duration max_goal_time_ = ros::Duration(100.0);  // 设置目标点超时时长，即超过100s未到达目标点，即跳过该点，执行下一个目标点
        ros::Time last_goal_time_;
        int pose_number_;
        double position_tolerance_;
        double angle_tolerance_deg_; // 角度误差容忍阈值（度）
        double angle_tolerance_;

        int index_current_ = 0;
        bool completed_ = false;
        bool threshold_reached_ = false;

        bool loop_navigation_;  // 新增的变量，用于控制是否循环目标点
    };
}
#endif
