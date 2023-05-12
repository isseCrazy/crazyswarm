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
    {
      
    }

  
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");

  Manager manager;
  ros::spin();

  return 0;
}
