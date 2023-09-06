#include<cohan_metrics/cohan_metrics.hh>

namespace cohan{

    Metrics::Metrics(std::string odom_topic, std::string agents_topic, std::string log_name){
        //Start ROS node
        ros::NodeHandle nh;
        int fov_int; double f_nb;
        
        //Subscribers
        r_odom_sub_ = nh.subscribe(odom_topic, 1, &Metrics::odomCB, this);
        agents_sub_ = nh.subscribe(agents_topic, 1, &Metrics::agentsCB, this);
        
        //ROS params
        nh.param(std::string("human_fov"), fov_int, int(120)); 
        human_fov_ = fov_int*M_PI/180;
        nh.param(std::string("seen_full_increase_durr"), f_nb, double(1.0)); 
        seen_full_increase_durr_ = ros::Duration(f_nb);
	    nh.param(std::string("seen_full_decrease_durr"), f_nb, double(2.0)); 
        seen_full_decrease_durr_ = ros::Duration(f_nb);
        nh.param(std::string("human_radius"), human_radius_, double(0.3));
	    nh.param(std::string("robot_radius"), robot_radius_, double(0.3));
        
        //Initialize properties
        robotIsMoving = false;
        c_react = 0;
        c_visibility = 0;
        c_shock = 0;
        c_fear = 0;
        c_panic = 0;
        robot_vel_.coeffRef(0) = 0;
        robot_vel_.coeffRef(1) = 0;
        
        //Start log file
        std::string log_file_path = ros::package::getPath("cohan_metrics") + "/logs/"+log_name+".txt";
        log_file_.open(log_file_path);
        log_file_ << "LOG STARTS : " << ros::Time::now().toSec() << std::endl;
        
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
        PoseSE2  robot_pose(robot_odom.pose.pose.position.x, robot_odom.pose.pose.position.y, theta_yaw);

        if(fabs(robot_odom.twist.twist.linear.x) > 0.001 || fabs(robot_odom.twist.twist.linear.x) > 0.001){
            robotIsMoving = true;
            robot_vel_.coeffRef(0) = robot_odom.twist.twist.linear.x;
            robot_vel_.coeffRef(1) = robot_odom.twist.twist.linear.y;
        }
        else{
            robotIsMoving = false;
            robot_vel_.coeffRef(0) = 0;
            robot_vel_.coeffRef(1) = 0;

        }

        //Logging the robot data
        log_file_ << now.toSec() << " : R " << robot_odom.pose.pose.position.x << " " << robot_odom.pose.pose.position.y  << " " << theta_yaw << std::endl;
        log_file_ << now.toSec() << " : VEL_R " << std::to_string(sqrt(pow(robot_odom.twist.twist.linear.x,2) + pow(robot_odom.twist.twist.linear.y,2))) << std::endl;

        if(std::fabs(robot_odom.header.stamp.toSec() - agents_.header.stamp.toSec()) < 0.01){
            for(auto agent : agents_.agents){
                for(auto segment : agent.segments){
                    if (segment.type == default_segment_){
                        q = segment.pose.pose.orientation;
                        siny_cosp = 2 * (q.w * q.z + q.x * q.y);
                        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
                        theta_yaw = atan2(siny_cosp, cosy_cosp);
                        PoseSE2  human_pose(segment.pose.pose.position.x, segment.pose.pose.position.y, theta_yaw);

                        //Logging the human data and the calculated metrics
                        log_file_ << now.toSec() << " : H " << segment.pose.pose.position.x << " " << segment.pose.pose.position.y  << " " << theta_yaw << std::endl;
                        log_file_ << now.toSec() << " : VEL_H " << std::to_string(sqrt(pow(segment.twist.twist.linear.x,2) + pow(segment.twist.twist.linear.y,2))) << std::endl;
                        std::string costs = computeMetrics(human_pose, robot_pose, now.toSec());
                        log_file_ << costs;
                    }
                }
            }
        }

    }

    void Metrics::agentsCB(const cohan_msgs::TrackedAgents::ConstPtr& agents_msg){
        agents_ = *agents_msg;
    }

    std::string Metrics::computeMetrics(PoseSE2 &h_pose, PoseSE2 &r_pose, double time_stamp){
        std::string cost_logs = "";

        if(calculateSupriseMetrics(h_pose, r_pose)){
            cost_logs += "COST_REACT "+std::to_string(c_react)+" "+std::to_string(time_stamp)+"\n";
            cost_logs += "COST_SHOCK "+std::to_string(c_shock)+" "+std::to_string(time_stamp)+"\n";
            cost_logs += "COST_VISIBILITY "+std::to_string(c_visibility)+" "+std::to_string(time_stamp)+"\n";
        }

        if(calculateDangerMetrics(h_pose, r_pose)){
            cost_logs += "COST_FEAR "+std::to_string(c_fear)+" "+std::to_string(time_stamp)+"\n";
            cost_logs += "COST_PANIC "+std::to_string(c_panic)+" "+std::to_string(time_stamp)+"\n";
        }

        return cost_logs;
    }

    bool Metrics::calculateSupriseMetrics(PoseSE2 &h_pose, PoseSE2 &r_pose){
        ros::Duration dt = ros::Time::now() - surprise_last_compute_;
        double dist = (h_pose.position() - r_pose.position()).norm();
        std::vector<double> costs(3,0);
        
        double dist_eff = dist - (human_radius_+robot_radius_);
        double alpha = (proxemics_dist/dist_eff);

        c_react = 0.0;
        c_visibility = 0.0;
        c_shock = 0.0;

        if(this->robotSeen(r_pose, h_pose)){
            seen_ratio += dt.toSec()/seen_full_increase_durr_.toSec();
            seen_ratio = std::min(seen_ratio, 1.0);

            if(robotIsMoving){
                c_react = alpha*(1.0 - seen_ratio);
            }

            if(seen_ratio < 1.0){
                c_visibility = alpha*c_visibility;
            }

            if(seen_ratio <= 0.25){
                c_shock = std::max(alpha*(1.0 - (4*seen_ratio)), 0.0);
            }

            return true;
        }

        return false;
    }

    bool Metrics::calculateDangerMetrics(PoseSE2 &h_pose, PoseSE2 &r_pose){
        double ttc =  -1;
        c_fear = 0;
        c_panic = 0;

        auto C = h_pose.position() - r_pose.position();
        double C_sq = C.dot(C);

        double radius_sum = human_radius_ + robot_radius_;
        double radius_sum_sq = radius_sum*radius_sum;

        if(C_sq <= radius_sum_sq){ // In collision --> May be add an extra metric
            ttc  = 0.0;
        }
        else{
            auto V = robot_vel_;
            double C_dot_V = C.dot(V);
            if(C_dot_V > 0){
                double V_sq = V.dot(V);
                double f = (C_dot_V*C_dot_V) - (V_sq - (C_sq - radius_sum_sq));
                if(f > 0){
                    ttc = (C_dot_V - sqrt(f)) / V_sq;
                }
                else{
                    double g = sqrt(V_sq*C_sq - C_dot_V*C_dot_V);
                    
                    if((g - (sqrt(V_sq)*radius_sum)) > 0.01){
                        c_panic = sqrt(V_sq/C_sq)*(g/(g - (sqrt(V_sq)*radius_sum)));
                    }
                }
            }
        }

        if(ttc > 0 && robotIsMoving){
            c_fear = 1/ttc;
        }

    } 

    bool Metrics::inFOV(PoseSE2 &A, PoseSE2 &B, float fov){
        // check if A is in the specified field of view of B (w/o obstacle)
        c_visibility = -1.0;
        float alpha;
        float qy = A.y() - B.y();
        float qx = A.x() - B.x();
        if(qx==0)
        {
            if(qy>0)
                alpha = M_PI/2;
            else
                alpha = -M_PI/2;
        }
        else
        {
            float q = abs(qy/qx);
            if(qx>0)
            {
                if(qy>0)
                    alpha = atan(q);
                else
                    alpha = -atan(q);
            }
            else
            {
                if(qy>0)
                    alpha = M_PI - atan(q);
                else
                    alpha = atan(q) - M_PI;
            }
        }

        float diff;
        if(alpha * B.theta() > 0) // same sign
            diff = abs(alpha - B.theta());
        else
        {
            if(alpha < 0)
                diff = std::min(abs(alpha - B.theta()), abs((alpha+2*M_PI) - B.theta()));
            else
                diff = std::min(abs(alpha - B.theta()), abs(alpha - (B.theta()+2*M_PI)));
        }

        c_visibility = diff/(fov/2);
        return diff < fov/2;
    }


    bool Metrics::robotSeen(PoseSE2 &robot_pose, PoseSE2 &human_pose){
            PoseSE2 human_pose_offset(human_pose.x()-offset_map_x, human_pose.y()-offset_map_y, human_pose.theta()); 
            PoseSE2 robot_pose_offset(robot_pose.x()-offset_map_x, robot_pose.y()-offset_map_y, robot_pose.theta());

            // check if the robot is in the field of view of the human
            // (without obstacles)
            if(this->inFOV(robot_pose_offset, human_pose_offset, human_fov_))
            {
                // check if there are obstacles blocking the human view of the robot
                if(this->checkObstacleView(human_pose_offset, robot_pose_offset))
                {
                    // the human sees the robot
                    return  true;
                }
                else
                {
                    // human can't see the robot
                    return  false;
                }
            }
            else
            {
                return false;
            }
    }

    bool Metrics::checkObstacleView(PoseSE2 &A_real, PoseSE2 &B_real){
        // check if there are obstacles preventing A from seeing B

        int A_map_x; int A_map_y;
        A_map_x = (int)(A_real.x() / resolution_map); A_map_y = (int)(A_real.y() / resolution_map);
        int B_map_x; int B_map_y;
        B_map_x = (int)(B_real.x() / resolution_map); B_map_y = (int)(B_real.y() / resolution_map);

        // if outside the map
        if(A_map_x < 0 || A_map_x >= (int)g_map_[0].size() || A_map_y < 0 || A_map_y >= (int)g_map_.size()
        || B_map_x < 0 || B_map_x >= (int)g_map_[0].size() || B_map_y < 0 || B_map_y >= (int)g_map_.size())
            return false;

        // particular cases
        // if one of the poses is an obstacle
        if(g_map_[A_map_y][A_map_x] == 1 || g_map_[B_map_y][B_map_x] == 1)
            return false;
        else if(A_map_x == B_map_x || A_map_y == B_map_y)
        {
            // same place
            if(A_map_x == B_map_x && A_map_y == B_map_y)
                return true;

            // vertical
            else if(A_map_x == B_map_x)
            {
                for(int i=0; A_map_y + i != B_map_y;)
                {
                    int xi = A_map_x;
                    int yi = A_map_y + i;

                    if(g_map_[yi][xi]==1) // if obstacle
                        return false;

                    // up
                    if(B_map_y > A_map_y)
                        i++;
                    // down
                    else
                        i--;
                }
            }

            // horizontal
            else if(A_map_y == B_map_y)
            {
                for(int i=0; A_map_x + i != B_map_x;)
                {
                    int xi = A_map_x + i;
                    int yi = A_map_y;

                    if(g_map_[yi][xi]==1)
                        return false;

                    // right
                    if(B_map_x > A_map_x)
                        i++;
                    // left
                    else
                        i--;
                }
            }
        }
        // general cases
        else
        {
            float m = (float)(B_map_y - A_map_y)/(float)(B_map_x - A_map_x);
            float b = A_map_y - m * A_map_x;

            float marge = 0.9;
            float delta_x = std::min(marge/abs(m), marge);

            // sign
            if(B_map_x < A_map_x)
                delta_x = -delta_x;

            int i=1;
            bool cond = true;
            while(cond)
            {
                float xi_f = A_map_x + i * delta_x;
                float yi_f = m * xi_f + b;

                int xi = (int)(xi_f);
                int yi = (int)(yi_f);

                if(g_map_[yi][xi]==1) // if obstacle
                    return false;

                i++;
                if(delta_x > 0)
                    cond = i*delta_x + A_map_x < B_map_x;
                else
                    cond = i*delta_x + A_map_x > B_map_x;
            }
        }

        return true;
    }

    void Metrics::mapCB(const nav_msgs::OccupancyGrid::ConstPtr& map){
        offset_map_x = map->info.origin.position.x;
        offset_map_y = map->info.origin.position.y;

        int width = map->info.width;
        int height = map->info.height;
        int cell;

        resolution_map = map->info.resolution;

        for(int i=0; i<height; i++)
        {
            std::vector<int> line;
            for(int j=0; j<width; j++)
            {
                cell = map->data[width*i+j];
                if(cell == 0)
                    line.push_back(0);
                else
                    line.push_back(1);
            }
            g_map_.push_back(line);
        }
    }



} //namespace cohan