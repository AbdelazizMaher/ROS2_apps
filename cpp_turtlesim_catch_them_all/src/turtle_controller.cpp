/* Include the main ROS 2 C++ API header */
#include "rclcpp/rclcpp.hpp"

#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

#include "my_custom_interfaces/msg/turtle.hpp"
#include "my_custom_interfaces/msg/turtle_array.hpp"
#include "my_custom_interfaces/srv/kill_turtle.hpp"

/* Define a class that inherits from rclcpp::Node */
class TurtleControllerNode : public rclcpp::Node
{
public:
    /* Constructor, initializing the Node with the name "turtle_controller" */
    TurtleControllerNode() : Node("turtle_controller")
    {
        this->declare_parameter("catch_closest_turtle_first", true);
        catch_closest_turtle_first_ = this->get_parameter("catch_closest_turtle_first").as_bool();

        new_pos_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        turtle_pos_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10, std::bind(&TurtleControllerNode::callbackNewPos, this, std::placeholders::_1));

        alive_turtles_subscriber_ = this->create_subscription<my_custom_interfaces::msg::TurtleArray>(
            "alive_turtles", 10, std::bind(&TurtleControllerNode::callbackALiveTurtles, this, std::placeholders::_1));

        control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&TurtleControllerNode::callbackControlPos, this));
    }

private:
    void callbackALiveTurtles(const my_custom_interfaces::msg::TurtleArray::SharedPtr msg)
    {
        if (!msg->turtles.empty())
        {
            if (catch_closest_turtle_first_)
            {
                auto closest_turtle = msg->turtles.at(0);
                double closest_turtle_distance = getDistanceFromCurrentPose(closest_turtle);
                for (auto turtle : msg->turtles)
                {
                    double distance = getDistanceFromCurrentPose(turtle);

                    if (distance < closest_turtle_distance)
                    {
                        closest_turtle = turtle;
                        closest_turtle_distance = distance;
                    }
                }
                turtle_to_catch_ = closest_turtle;
            }
            else
            {
                turtle_to_catch_ = msg->turtles.at(0);
            }
        }
    }

    double getDistanceFromCurrentPose(my_custom_interfaces::msg::Turtle turtle)
    {
        double dist_x = turtle.x - pose_.x;
        double dist_y = turtle.y - pose_.y;
        return std::sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    void callbackNewPos(const turtlesim::msg::Pose::SharedPtr msg)
    {
        pose_ = *msg.get();
    }

    void callbackControlPos()
    {
        if (turtle_to_catch_.name == "")
        {
            return;
        }

        double dist_x = turtle_to_catch_.x - pose_.x;
        double dist_y = turtle_to_catch_.y - pose_.y;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        auto msg = geometry_msgs::msg::Twist();

        if (distance > 0.5)
        {
            msg.linear.x = 2 * distance;

            double steering_angle = std::atan2(dist_y, dist_x);
            double angle_diff = steering_angle - pose_.theta;
            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }

            msg.angular.z = 6 * angle_diff;
        }
        else
        {
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;

            catch_turtle_threads_.push_back(
                std::thread(std::bind(&TurtleControllerNode::callKillTurtle_server, this, turtle_to_catch_.name)));

            turtle_to_catch_.name = "";
        }

        new_pos_publisher_->publish(msg);
    }

    void callKillTurtle_server(std::string turtle_name)
    {
        /* Create a client for the SetBool service named "kill_turtle" */
        auto client = this->create_client<my_custom_interfaces::srv::KillTurtle>("kill_turtle");
        /* Wait for the service to be available, checking every second */
        while (!client->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");

        /* Create a shared pointer to a new KillTurtle request */
        auto request = std::make_shared<my_custom_interfaces::srv::KillTurtle::Request>();
        /* Set the request data to the value of the turtle_name argument */
        request->name = turtle_name;

        /* Send the request asynchronously and get a future to track the response */
        auto future = client->async_send_request(request);

        /* Catch any exceptions that occur during the service call */
        try
        {
            /* Wait for the response (blocking) and get the result */
            auto response = future.get();

            if (!response->success)
            {
                RCLCPP_ERROR(this->get_logger(), "turtle %s could not be killed", turtle_name.c_str());
            }
        }
        catch (const std::exception &e)
        {
            /* Log an error message if the service call fails */
            RCLCPP_ERROR(this->get_logger(), "Service call failed!");
        }
    }

    bool catch_closest_turtle_first_;

    my_custom_interfaces::msg::Turtle turtle_to_catch_;

    turtlesim::msg::Pose pose_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr new_pos_publisher_;

    rclcpp::Subscription<my_custom_interfaces::msg::TurtleArray>::SharedPtr alive_turtles_subscriber_;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pos_subscriber_;

    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    std::vector<std::thread> catch_turtle_threads_;
};

int main(int argc, char **argv)
{
    /* Initialize the ROS 2 system(Initializes the ROS 2 client library) */
    rclcpp::init(argc, argv);
    /* Create a shared pointer to an instance of TurtleControllerNode */
    auto node = std::make_shared<TurtleControllerNode>();
    /* Spin the node to start the event loop to process callbacks */
    rclcpp::spin(node);
    /* Shutdown the ROS 2 system */
    rclcpp::shutdown();
}