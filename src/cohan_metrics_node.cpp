#include <cohan_metrics/cohan_metrics.hh>
#define ROBOT_ODOM "/mobile_base_controller/odom"
// #define ROBOT_ODOM "/base_odometry/odom" // Change it based on the robot
#define HUMANS_TOPIC "/tracked_agents"

int main(int argc, char **argv) {
  ros::init(argc, argv, "cohan_metrics");

  if (argc < 2) {
    cohan::Metrics metrics(ROBOT_ODOM, HUMANS_TOPIC, "log");
  } else {
    std::string log_name_ = argv[1];
    cohan::Metrics metrics(ROBOT_ODOM, HUMANS_TOPIC, log_name_);
  }

  return 0;
}