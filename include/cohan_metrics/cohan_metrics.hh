#ifndef COHAN_METRICS_HH_ 
#define COHAN_METRICS_HH_ 

#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cohan_msgs/TrackedAgents.h>
#include <cohan_msgs/TrackedAgent.h>
#include <cohan_msgs/TrackedSegmentType.h>
#include <fstream>
#include <math.h>
#include <Eigen/Core>

#define default_segment_ cohan_msgs::TrackedSegmentType::TORSO

namespace cohan{

    class PoseSE2{
      public:
        PoseSE2(){
          position_.coeffRef(0) = 0.0;
          position_.coeffRef(1) = 0.0;
          theta_ = 0.0;
        }

        PoseSE2(Eigen::Vector2d pose, double yaw){
          position_ = pose;
          theta_ = yaw;
        }

        PoseSE2(double x, double y, double yaw){
          position_.coeffRef(0) = x;
          position_.coeffRef(1) = y;
          theta_ = yaw;
        }

        ~PoseSE2(){}

        double& x() {return position_.coeffRef(0);}

        const double& x() const {return position_.coeffRef(0);}

        double& y() {return position_.coeffRef(1);}

        const double& y() const {return position_.coeffRef(1);}

        double& theta() {return theta_;}

        const double& theta() const {return theta_;}

        Eigen::Vector2d& position() {return position_;}

        const Eigen::Vector2d& position() const {return position_;}

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      private:
        Eigen::Vector2d position_;
        double theta_;

    };

    class Metrics{

      public:
        
        Metrics(std::string odom_topic, std::string agents_topic, std::string log_name);
      
        ~Metrics(){};

        void odomCB(const nav_msgs::Odometry::ConstPtr& odom_msg);

        void agentsCB(const cohan_msgs::TrackedAgents::ConstPtr& agents_msg);

        std::string computeMetrics(int agent_id, double time_stamp);

        bool calculateSupriseMetrics();

        bool calculateDangerMetrics();

        bool inFOV(PoseSE2 &A, PoseSE2 &B, float fov);

        bool checkrobotSeen();

        bool checkObstacleView(PoseSE2 &A_real, PoseSE2 &B_real);

        void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& map);
      
      private:

        nav_msgs::Odometry robot_odom;
        cohan_msgs::TrackedAgents agents_;
        ros::Subscriber r_odom_sub_, agents_sub_, map_sub_;
        tf2_ros::Buffer tfBuffer;
        std::ofstream log_file_;
        ros::Timer timer;
        PoseSE2 robot_pose_, human_pose_;

        //Metric costs
        double c_visibility, c_fear, c_panic, c_shock, c_react;

        // Helper variables for metric calculations
        double seen_ratio;
        ros::Time surprise_last_compute_;
        bool robotIsMoving;
        Eigen::Vector2d robot_vel_;
        double robot_omega_;
        bool processing_;
        bool robot_seen_;

        //Ros param variable
        double human_fov_; 
        ros::Duration seen_full_increase_durr_, seen_full_decrease_durr_;
        double human_radius_, robot_radius_;
        double proxemics_dist;

        //Map info
        std::vector<std::vector<int>> g_map_;
        double  offset_map_x, offset_map_y;
        double resolution_map;


    };

}// namespace cohan
#endif