#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "catch_them_all_project_interfaces/msg/turtle_list.hpp"
#include "catch_them_all_project_interfaces/srv/catch_turtle.hpp"
#include <random>
#include <vector>

using namespace std::chrono_literals;
using namespace std::placeholders;
 
class TurtlespawnerNode : public rclcpp::Node 
{
public:
    TurtlespawnerNode() : Node("turtle_spawner")
    {
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        kill_client_ = this->create_client<turtlesim::srv::Kill>("kill");
        catch_service_ = this->create_service<catch_them_all_project_interfaces::srv::CatchTurtle>("catch_turtle", std::bind(&TurtlespawnerNode::callbackCatchTurtle, this, _1, _2));
        timer_ = this->create_wall_timer(1s, std::bind(&TurtlespawnerNode::callback_timer, this));
        std::random_device rd;
        generator_ = std::mt19937(rd());
        distribution_ = std::uniform_real_distribution<float>(1.0f, 10.0f);
        publisher_ = this->create_publisher<catch_them_all_project_interfaces::msg::TurtleList>("turtle_list", 10);
        RCLCPP_INFO(this->get_logger(),"TurtlesimSpawn node has been started ...");
    }
// Function to publish the list of alive turtles
    void publishTurtleList()
    {
        auto msg = catch_them_all_project_interfaces::msg::TurtleList();
        msg.alive_turtles = turtles_alive;
        publisher_->publish(msg);
    }

    // Callback function for the catch_turtle service
    // it will kill the turtle when a client calls the service with the name of the turtle to catch
    // then it will remove the turtle from the list of alive turtles and publish the updated list
    void callbackCatchTurtle(const std::shared_ptr<catch_them_all_project_interfaces::srv::CatchTurtle::Request> request,
                            const std::shared_ptr<catch_them_all_project_interfaces::srv::CatchTurtle::Response> response)
    {
        std::string turtle_to_catch = request->name;

        for(size_t i = 0; i < turtles_alive.size(); i++)
        {
            if(turtles_alive[i].name == turtle_to_catch)
            {
                auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
                kill_request->name = turtle_to_catch;
                kill_client_->async_send_request(kill_request);
                turtles_alive.erase(turtles_alive.begin() + i);
                publishTurtleList();
                response->success = true;
                RCLCPP_INFO(this->get_logger(), "%s has been caught and removed from the simulation.", turtle_to_catch.c_str());
                return;
            }
        }
        response->success = false;
        RCLCPP_WARN(this->get_logger(), "Turtle %s not found!", turtle_to_catch.c_str());
    }
// Callback function for the timer
// it will spawn a new turtle every second at a random position with a given name and add it to the list of alive turtles
    void callback_timer()
    {
        while (!spawn_client_->wait_for_service(1s))
        {
        RCLCPP_WARN(this->get_logger(), "Waiting for the server ... ");

        }
        catch_them_all_project_interfaces::msg::TurtlePose current_turtle;
        current_turtle.x = distribution_(generator_);
        current_turtle.y = distribution_(generator_);
        current_turtle.theta = 0.0;
        current_turtle.name = "turtle" + std::to_string(number_of_turtles + 1);

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = current_turtle.x;
        request->y = current_turtle.y;
        request->theta = current_turtle.theta;
        request->name = current_turtle.name;

        spawn_client_->async_send_request(request,std::bind(&TurtlespawnerNode::callbackSpawnClient, this, _1));
        turtles_alive.push_back(current_turtle);
    }
// Callback function for the spawn service client
    void callbackSpawnClient(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
    {
        auto response = future.get();

        RCLCPP_INFO(this->get_logger()," %s spawned ", response->name.c_str());
        publishTurtleList();
        number_of_turtles++;
    }
        

private:

    std::mt19937 generator_; 
    std::uniform_real_distribution<float> distribution_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Service<catch_them_all_project_interfaces::srv::CatchTurtle>::SharedPtr catch_service_;
    rclcpp::Publisher<catch_them_all_project_interfaces::msg::TurtleList>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<catch_them_all_project_interfaces::msg::TurtlePose> turtles_alive;
    int number_of_turtles = 1;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtlespawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
