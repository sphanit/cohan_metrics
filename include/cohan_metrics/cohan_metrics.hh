#ifndef COHAN_METRICS_HH_ 
#define COHAN_METRICS_HH_ 

#include <nav_msgs/Odometry.h>
#include <cohan_msgs/TrackedAgents.h>
#include <cohan_msgs/TrackedAgent.h>
#include <cohan_msgs/TrackedSegment.h>
#include <fstream>
#include <math.h>
#define default_segment_ cohan_msgs::TrackedSegmentType::TORSO

namespace cohan{
    class Metrics{
      
      public:
        
        Metrics(std::string odom_topic, std::string agents_topic, std::string log_name);
      
        ~Metrics(){};

        void odomCB(const nav_msgs::Odometry::ConstPtr& odom_msg);

        void agentsCB(const cohan_msgs::TrackedAgents::ConstPtr& agents_msg);

        std::string computeMetrics(cohan_msgs::TrackedSegment h_segment, nav_msgs::Odometry r_odom);

      
      private:

        nav_msgs::Odometry robot_odom_;
        cohan_msgs::TrackedAgents agents_;
        ros::Subscriber r_odom_sub_, agents_sub_;
        ofstream log_file_;


    };
}// namespace cohan
#endif