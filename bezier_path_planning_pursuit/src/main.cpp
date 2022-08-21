#include "path_planner.h"

using namespace bezier_path_planning_pursuit;
std::string node_name = "bezier_path_planning_pursuit";

std::size_t file_count_boost(const boost::filesystem::path &root)
{
    namespace fs = boost::filesystem;
    if (!fs::exists(root) || !fs::is_directory(root))
        return 0;

    std::size_t result = 0;
    fs::directory_iterator last;
    for (fs::directory_iterator pos(root); pos != last; ++pos)
    {
        ++result;
        if (fs::is_directory(*pos))
            result += file_count_boost(pos->path());
    }

    return result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, node_name);

    ros::NodeHandle nh;
    ros::NodeHandle arg_n("~");

    int looprate = 30; // Hz
    std::string data_path = "";
    bool use_tf = false;
    float max_accel = 2.5;
    float max_vel = 1.5;
    float corner_speed_rate = 0.8;
    float xy_goal_tolerance = 0.05;
    float yaw_goal_tolerance = 0.01;
    float fix_angle_gain = 10.0;
    float path_granularity = 0.01;
    std::string global_frame_id = "odom";
    std::string base_frame_id = "base_link";
    std::string angle_source = "pose";

    float acc_lim_theta = 3.2;
    float max_vel_theta = 1.57;
    float initial_vel = 0.1;

    arg_n.getParam("control_frequency", looprate);
    arg_n.getParam("use_tf", use_tf);
    arg_n.getParam("data_path", data_path);
    arg_n.getParam("acc_lim_xy", max_accel);
    arg_n.getParam("acc_lim_theta", acc_lim_theta);
    arg_n.getParam("max_vel_xy", max_vel);
    arg_n.getParam("max_vel_theta", max_vel_theta);
    arg_n.getParam("initial_vel", initial_vel);
    arg_n.getParam("corner_speed_rate", corner_speed_rate);
    arg_n.getParam("global_frame_id", global_frame_id);
    arg_n.getParam("base_frame_id", base_frame_id);
    arg_n.getParam("xy_goal_tolerance", xy_goal_tolerance);
    arg_n.getParam("yaw_goal_tolerance", yaw_goal_tolerance);
    arg_n.getParam("fix_angle_gain", fix_angle_gain);
    arg_n.getParam("path_granularity", path_granularity);
    arg_n.getParam("angle_source", angle_source);

    std::size_t number = file_count_boost(data_path);
    int LINE_NUM = (int)number;

    Path_Planner planner(nh, looprate, use_tf, data_path, max_accel, max_vel, corner_speed_rate, global_frame_id, base_frame_id, initial_vel, xy_goal_tolerance, yaw_goal_tolerance, angle_source, max_vel_theta, acc_lim_theta, fix_angle_gain, LINE_NUM, path_granularity);
    ros::spin(); // Wait to receive action goal
    return 0;
}
