#include "rclcpp/rclcpp.hpp"
#include "catch_them_all_project_interfaces/msg/turtle_list.hpp"
#include "catch_them_all_project_interfaces/srv/catch_turtle.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
 
class MainTurtleControllerNode : public rclcpp::Node 
{
public:
    MainTurtleControllerNode() : Node("main_turtle_controller")
    {
        turtle_list_subscriber = this->create_subscription<catch_them_all_project_interfaces::msg::TurtleList>(
            "turtle_list", 10, std::bind(&MainTurtleControllerNode::callbackTurtleList, this, std::placeholders::_1));
        pose_subscriber = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&MainTurtleControllerNode::callbackTurtlePose, this, std::placeholders::_1));
        catch_turtle_client = this->create_client<catch_them_all_project_interfaces::srv::CatchTurtle>("catch_turtle");
        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "MainTurtleController node has been started ...");
        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(time_step_), std::bind(&MainTurtleControllerNode::controlLoop, this));
    }
// Function to call the catch_turtle service with the name of the turtle to catch
    void catchTurtle(const std::string & turtle_name)
    {
        while (!catch_turtle_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the catch_turtle service to be available...");
        }
        auto request = std::make_shared<catch_them_all_project_interfaces::srv::CatchTurtle::Request>();
        request->name = turtle_name;
        catch_turtle_client->async_send_request(request, std::bind(&MainTurtleControllerNode::callbackCatchTurtleResponse, this, std::placeholders::_1));
    }
// Callback function for the turtle_list topic subscriber
// it will update the list of alive turtles and print the number of alive turtles
    void callbackTurtleList(const catch_them_all_project_interfaces::msg::TurtleList::SharedPtr msg)
    {
        turtles_alive = msg->alive_turtles;
        RCLCPP_INFO(this->get_logger(), " %zu turtles alive.", turtles_alive.size());
    }

    void callbackTurtlePose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle_pose_ = *msg;
        pose_received_ = true;
    }
// Function to find the closest turtle to the current turtle pose
    catch_them_all_project_interfaces::msg::TurtlePose findClosestTurtle()
    {
        catch_them_all_project_interfaces::msg::TurtlePose closest_turtle;
        if (turtles_alive.empty()) {
            RCLCPP_WARN(this->get_logger(), "No turtles alive to find the closest one.");
            return closest_turtle; 
        }
        float min_diatnce = 0;
        for (size_t i = 0; i < turtles_alive.size(); i++)
        {
            float distance = std::sqrt(std::pow(turtles_alive[i].x - turtle_pose_.x, 2) + std::pow(turtles_alive[i].y - turtle_pose_.y, 2));
            if (i == 0 || distance < min_diatnce)
            {
                min_diatnce = distance;
                closest_turtle = turtles_alive[i];
            }
        }
        target_state_ = true; 
        return closest_turtle;
    }
// Function to publish the cmd_vel topic with the calculated linear and angular velocities
    void publishCmdVel(float linear, float angular)
    {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = linear;
        cmd_vel_msg.angular.z = angular;
        cmd_vel_publisher->publish(cmd_vel_msg);
    }
//main control loop function (P controller)
//it calculates the linear and angular velocities to move towards the target turtle and publish the cmd_vel topic
    void controlLoop()
    {
        if (!pose_received_) {
            RCLCPP_WARN(this->get_logger(), "No turtle pose received yet, waiting...");
            return;
        }
        if (!target_state_) {
            target_ = findClosestTurtle();
            if (!target_state_) {
                RCLCPP_WARN(this->get_logger(), "No target turtle found, waiting...");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Target turtle: %s", target_.name.c_str());
            target_state_ = true;
        }

        float angle_to_target = std::atan2(target_.y - turtle_pose_.y, target_.x - turtle_pose_.x);
        float distance_to_target = std::sqrt(std::pow(target_.x - turtle_pose_.x, 2) + std::pow(target_.y - turtle_pose_.y, 2));
        float angle_error = angle_to_target - turtle_pose_.theta;

        if (angle_error > M_PI) // Normalize the angle error to the range [-pi, pi]
        { 
            angle_error -= 2 * M_PI;
        } else if (angle_error < -M_PI) 
        {
            angle_error += 2 * M_PI;
        }
        float angular_velocity = p_angle_ * angle_error;
        float linear_velocity = p_distance_ * distance_to_target;
        publishCmdVel(linear_velocity, angular_velocity);

        if (distance_to_target < 0.5) {
            RCLCPP_INFO(this->get_logger(), "Attempting to catch turtle: %s", target_.name.c_str());
            catchTurtle(target_.name);
            angle_to_target = 0;
            distance_to_target = 0;
            angle_error = 0;
            angular_velocity = 0;
            linear_velocity = 0;
            publishCmdVel(0, 0);
            target_state_ = false; 
        }
    }


 
private:

    void callbackCatchTurtleResponse(rclcpp::Client<catch_them_all_project_interfaces::srv::CatchTurtle>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Successfully caught the turtle.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to catch the turtle.");
        }
    }

    std::vector<catch_them_all_project_interfaces::msg::TurtlePose> turtles_alive;
    turtlesim::msg::Pose turtle_pose_;
    bool target_state_ = false; // to store if we have selected the target turtle or not
    bool pose_received_ = false; // to store if we have received the pose message at least once at the beginning befote the control loop starts
    catch_them_all_project_interfaces::msg::TurtlePose target_;
    // P controller gains and control loop time step
    float p_angle_ = 6.0; 
    float p_distance_ = 2.0;
    int time_step_ = 10; //in milliseconds

    rclcpp::Subscription<catch_them_all_project_interfaces::msg::TurtleList>::SharedPtr turtle_list_subscriber;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber;
    rclcpp::Client<catch_them_all_project_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_client;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::TimerBase::SharedPtr control_timer_;

    
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainTurtleControllerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
