#include <thread>
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/ca_nraw.hpp"
#include "rad_control/rad.hpp"

using namespace std::chrono_literals;
using namespace custom_interfaces::msg;
using std::placeholders::_1;

std::shared_ptr<rclcpp::Node> can_config;
std::shared_ptr<rclcpp::Publisher<CANraw>> can_pub;
std::shared_ptr<rclcpp::Subscription<CANraw>> can_sub;
uint8_t rad_id, command_id;
bool ack,ready;

#define INPUT_CHECK(c) try {c} catch(...){RCLCPP_ERROR(can_config->get_logger(), "INVALID INPUT"); continue;}

int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);

    can_config = std::make_shared<rclcpp::Node>("rad_science_controller_node");
    can_pub = can_config->create_publisher<CANraw>("/can/can_out", 10);

    CANraw can_out_msg;
    RAD rad{&can_out_msg};
    rclcpp::WallRate loop_rate(500ms);

    std::thread spin_thread([](){rclcpp::spin(can_config);});

    std::string in;

    std::cout << "Enter Science Arm RAD ID (prefix h for hex #) => ";
    std::getline (std::cin,in);
    // Removing whitespace
    in.erase(std::remove_if(in.begin(), in.end(), isspace), in.end());
   
    int base = 10;
    INPUT_CHECK(
        if (in[0] == 'h')
        {
        base = 16;
        in = in.substr(1);
        }
        rad_id = std::stoi(in, 0, base);
        rad.set_can_id(rad_id);
    )

    while(true)
    {
        ack = false;
        std::cout << "Enter number of steps, prefaced by u (up) or d (down) (e.g. u600). q to quit=> ";
        std::getline (std::cin,in);

        in.erase(std::remove_if(in.begin(), in.end(), isspace), in.end());
        if (in == "q" || std::cin.fail())
            break;    

        bool up = false;
        int steps = 0;

        INPUT_CHECK(
        if (in[0] == 'u')
        {
            up = true;
            in = in.substr(1);
        }
        else if (in[0] == 'd')
        {
            up = false;
            in = in.substr(1);
        }
        else
        {
            std::cout << "Invalid input. Exiting";
            break;
        }

        steps = std::stoi(in, 0, 10);
        
        )

        if (!up)  //ADJUST THIS - CORRELATE EITHER UP OR DOWN TO NEGATIVE STEPS
        {
            steps = -1 * steps;
        }

        rad.pulse_stepper((float)steps);

        can_pub->publish(can_out_msg);
        RCLCPP_INFO(can_config->get_logger(), "SENT CAN FRAME 0x%x", can_out_msg.address);


    }

    spin_thread.~thread();
    rclcpp::shutdown();

}