#include<cohan_metrics/cohan_metrics.hh>

namespace cohan{

    Metrics::Metrics(std::string odom_topic, std::string agents_topic, std::string log_name){
        ros::NodeHandle nh;
        r_odom_sub_ = nh.subscribe(odom_topic, 1, &Metrics::odomCB, this);
        agents_sub_ = nh.subscribe(agents_topic, 1, &Metrics::agentsCB, this);
        string log_file_path = ros::package::getPath("cohan_metrics") + "/logs/"+log_name+".txt";
        log_file_.open(log_file_path);
        log_file_ << "LOG STARTS : " << ros::Time::now().toSec() << endl;
        ROS_INFO("Started the metrics node");
        ros::spin();
    }

    void Metrics::odomCB(const nav_msgs::Odometry::ConstPtr& odom_msg){
        robot_odom = *odom_msg;
        auto now =  ros::Time::now();

        auto q = robot_odom.pose.pose.orientation;
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        double theta_yaw = atan2(siny_cosp, cosy_cosp);

        //Logging the robot data
        log_file_ << now.toSec() << " : R " << robot_odom.pose.pose.position.x << " " << robot_odom.pose.pose.position.y  << " " << theta_yaw << endl;
        log_file_ << now.toSec() << " : VEL_R " << std::to_string(sqrt(pow(robot_odom.twist.twist.linear.x,2) + pow(robot_odom.twist.twist.linear.y,2))) << endl;

        if(std::fabs(robot_odom.header.stamp.toSec() - agents_.header.stamp.toSec()) < 0.01){
            for(auto agent : agents_.agents){
                for(auto segment : agent.segments){
                    if (segment.type == default_segment_){
                        q = segment.pose.pose.orientation;
                        siny_cosp = 2 * (q.w * q.z + q.x * q.y);
                        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
                        theta_yaw = atan2(siny_cosp, cosy_cosp);

                        //Logging the human data and the calculated metrics
                        log_file_ << now.toSec() << " : H " << segment.pose.pose.position.x << " " << segment.pose.pose.position.y  << " " << theta_yaw << endl;
                        log_file_ << now.toSec() << " : VEL_H " << std::to_string(sqrt(pow(segment.twist.twist.linear.x,2) + pow(segment.twist.twist.linear.y,2))) << endl;
                        std::string costs = computeMetrics(segment, robot_odom);
                        log_file_ << costs << std::endl;
                    }
                }
            }
        }

    }

    void Metrics::agentsCB(const cohan_msgs::TrackedAgents::ConstPtr& agents_msg){
        agents_ = *agents_msg;
    }

    std::string Metrics::computeMetrics(cohan_msgs::TrackedSegment &h_segment, nav_msgs::Odometry &r_odom){

    }



} //namespace cohan