#include "rad_control/rad_status_configure.hpp"
#include "rad_control/rad.hpp"


RAD_Status_Configure::RAD_Status_Configure() : Node("rad_status_configure_node")
{
  this->declare_parameter("can_topic", "/can/can_out");
  topic = this->get_parameter("can_topic").as_string();

  can_pub_ = this->create_publisher<CANraw>(topic, 10);
 
}

void extract_can_message(std::string msgInput, std::string &can_message, std::string &device_id, std::string &can_data, std::string delimiter = " ")
{
    can_message.clear();
    device_id.clear();
    can_data.clear();

    int start, end = -1*delimiter.size();

    start = end + del.size(); // start becomes 0
    end = s.find(delimiter, start); // find first space


    can_message = s.substr(start, end-start);

    if (end == std::string::npos)
    {
        //reached end of string
        return;
    }

    start = end + del.size(); // set start to after first space
    end = s.find(delimiter, start); // find second space

    device_id = s.substr(start, end-start);

    if (end == std::string::npos)
    {
        //reached end of string
        return;
    }

    start = end + del.size(); // set start to after first space
    end = s.find(delimiter, start); // find second space

    can_data = s.substr(start, end-start); //return float or int value

}

uint16_t decode_can_msg(std::string msg)
{
    uint16_t id;

    if (msg == "SET_PID") {id = SET_P_VALUE;}



    return id;
}

bool decode_is_global(std::string msg)
{
    if (id == "ESTOP" ||
        id == "DISABLE" ||
        id == "ENABLE" ||
        id == "PING_HEALTH" ||
        id == "ASSING_ID")
    {
        return true;
    }
    
    return false;
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  std::string can_message;
  std::string device_id;
  std::string can_data;


    while (true)
    {

        std::string msgInput;
        std::cout << "CAN Message: ";
        std::getline(std::cin, msgInput);

        extract_can_message(msgInput, can_message, device_id, can_data, " ");

        if (can_message.empty())
        {
            continue;
        }
        if (device_id.empty())
        {
            device_id = "00";
        }
        if (can_data.empty())
        {
            can_data = "00";
        }

        uint16_t CAN_ID = decode_can_id(can_message);


        if (decode_is_global(can_message))
        {  
            pub.address = (CAN_ID << 8) | device_id;
        }
        else
        {
            pub.address = (2 << 25) | (CAN_ID << 8) | device_id;
        }

        pub.data




    }



  rclcpp::shutdown();
  
  return 0;
}
