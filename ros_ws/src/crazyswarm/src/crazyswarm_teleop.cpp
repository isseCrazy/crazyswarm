#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>

#include <crazyswarm/Takeoff.h>
#include <crazyswarm/Land.h>

namespace Xbox360Buttons {

    enum {
        Green  = 0,
        Red    = 1,
        Blue   = 2,
        Yellow = 3,
        LB     = 4,
        RB     = 5,
        Back   = 6,
        Start  = 7,
        COUNT  = 8,
    };

}

class Manager
{
public:

    Manager()
        : m_subscribeJoy()
        , m_serviceEmergency()
        , m_advertiseTakeoff()
        , m_advertiseLand()        
    {
        ros::NodeHandle nh;
        m_subscribeJoy = nh.subscribe("/joy", 1, &Manager::joyChanged, this);
        
        ROS_INFO("Wait for emergency service...");

        ros::service::waitForService("/emergency");
        m_serviceEmergency = nh.serviceClient<std_srvs::Empty>("/emergency");
        
        m_advertiseTakeoff = nh.advertise<crazyswarm::Takeoff>("/takeoff", 1);
        m_advertiseLand = nh.advertise<crazyswarm::Land>("/land", 1);
       
        ROS_INFO("Manager ready.");
    }

    ~Manager()
    {
    }
private:
    void joyChanged(
        const sensor_msgs::Joy::ConstPtr& msg)
    {
        static std::vector<int> lastButtonState(Xbox360Buttons::COUNT);

        if (msg->buttons.size() >= Xbox360Buttons::COUNT
            && lastButtonState.size() >= Xbox360Buttons::COUNT)
        {
            if (msg->buttons[Xbox360Buttons::Red] == 1 && lastButtonState[Xbox360Buttons::Red] == 0) {
                emergency();
            }
            if (msg->buttons[Xbox360Buttons::Start] == 1 && lastButtonState[Xbox360Buttons::Start] == 0) {
                takeoff();
            }
            if (msg->buttons[Xbox360Buttons::Back] == 1 && lastButtonState[Xbox360Buttons::Back] == 0) {
                land();
            }
        }

        lastButtonState = msg->buttons;
    }

    void emergency()
    {
        ROS_INFO("emergency requested...");
        std_srvs::Empty srv;
        m_serviceEmergency.call(srv);
        ROS_INFO("Done.");
    }

    void takeoff()
    {
        crazyswarm::Takeoff msg;
        msg.groupMask = 0;
        msg.height = 0.5;
        msg.duration = ros::Duration(2.0);
        m_advertiseTakeoff.publish(msg);
    }

    void land()
    {
        crazyswarm::Land msg;
        msg.groupMask = 0;
        msg.height = 0.05;
        msg.duration = ros::Duration(3.5);
        m_advertiseLand.publish(msg);
    }
private:
    ros::Subscriber m_subscribeJoy;

    ros::ServiceClient m_serviceEmergency;
    ros::Publisher m_advertiseTakeoff;
    ros::Publisher m_advertiseLand;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");

  Manager manager;
  ros::spin();

  return 0;
}
