#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/callback_queue.h>

#include "crazyswarm/LogBlock.h"
#include "crazyswarm/GenericLogData.h"
#include "crazyswarm/UpdateParams.h"
#include "crazyswarm/UploadTrajectory.h"
#include "crazyswarm/NotifySetpointsStop.h"
#undef major
#undef minor
#include "crazyswarm/Hover.h"
#include "crazyswarm/Takeoff.h"
#include "crazyswarm/Land.h"
#include "crazyswarm/GoTo.h"
#include "crazyswarm/StartTrajectory.h"
#include "crazyswarm/SetGroupMask.h"
#include "crazyswarm/FullState.h"
#include "crazyswarm/Position.h"
#include "crazyswarm/VelocityWorld.h"
#include "crazyswarm/Crazyflie.h"
#include "std_srvs/Empty.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Float32.h"

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud.h>

//#include <regex>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <crazyflie_cpp/Crazyflie.h>

// debug test
#include <signal.h>
#include <csignal> // or C++ style alternative

// Motion Capture
#include <libmotioncapture/motioncapture.h>

// Object tracker
#include <libobjecttracker/object_tracker.h>
#include <libobjecttracker/cloudlog.hpp>

#include <fstream>
#include <future>
#include <mutex>
#include <wordexp.h> // tilde expansion


class ROSLogger : public Logger
{
public:
  ROSLogger()
    : Logger()
  {
  }

  virtual ~ROSLogger() {}

  virtual void info(const std::string& msg)
  {
    ROS_INFO("%s", msg.c_str());
  }

  virtual void warning(const std::string& msg)
  {
    ROS_WARN("%s", msg.c_str());
  }

  virtual void error(const std::string& msg)
  {
    ROS_ERROR("%s", msg.c_str());
  }
};

static ROSLogger rosLogger;

void logWarn(const std::string& msg)
{
  ROS_WARN("%s", msg.c_str());
}

class CrazyflieROS
{
public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& tf_prefix,
    const std::string& worldFrame,
    int id,
    const std::string& type,
    const std::vector<crazyswarm::LogBlock>& log_blocks)
    : m_tf_prefix(tf_prefix)
    , m_cf(
      link_uri,
      rosLogger,
      std::bind(&CrazyflieROS::onConsole, this, std::placeholders::_1))
    , m_worldFrame(worldFrame)
    , m_id(id)
    , m_type(type)
    , m_logBlocks(log_blocks)
    , m_initializedPosition(false)
    , m_queue()
    , m_endOfLife(false)
  {
    ros::NodeHandle nl("~");
    nl.param("enable_logging", m_enableLogging, false);
    nl.param("enable_logging_pose", m_enableLoggingPose, false);
    nl.param("enable_parameters", m_enableParameters, true);
    nl.param("force_no_cache", m_forceNoCache, false);
    
    ros::NodeHandle n;
    n.setCallbackQueue(&m_queue);
    m_subscribeUploadTrajectory = n.subscribe(tf_prefix + "/upload_trajectory", 5, &CrazyflieROS::uploadTrajectory, this);
    m_subscribeStartTrajectory = n.subscribe(tf_prefix + "/start_trajectory", 5, &CrazyflieROS::startTrajectory, this);
    m_subscribeTakeoff = n.subscribe(tf_prefix + "/takeoff", 5, &CrazyflieROS::takeoff, this);
    m_subscribeLand = n.subscribe(tf_prefix + "/land", 5, &CrazyflieROS::land, this);
    m_subscribeGoTo = n.subscribe(tf_prefix + "/go_to", 5, &CrazyflieROS::goTo, this);
    m_subscribeSetGroupMask = n.subscribe(tf_prefix + "/set_group_mask", 5, &CrazyflieROS::setGroupMask, this);
    m_subscribeNotifySetpointsStop = n.subscribe(tf_prefix + "/notify_setpoints_stop", 5, &CrazyflieROS::notifySetpointsStop, this);

    m_subscribeCmdVel = n.subscribe(tf_prefix + "/cmd_vel", 1, &CrazyflieROS::cmdVelChanged, this);
    m_subscribeCmdPosition = n.subscribe(tf_prefix + "/cmd_position", 1, &CrazyflieROS::cmdPositionSetpoint, this);
    m_subscribeCmdFullState = n.subscribe(tf_prefix + "/cmd_full_state", 1, &CrazyflieROS::cmdFullStateSetpoint, this);
    m_subscribeCmdVelocityWorld = n.subscribe(tf_prefix + "/cmd_velocity_world", 1, &CrazyflieROS::cmdVelocityWorldSetpoint, this);
    m_subscribeCmdStop = n.subscribe(m_tf_prefix + "/cmd_stop", 1, &CrazyflieROS::cmdStop, this);
    m_subscribeCmdHover = n.subscribe(m_tf_prefix+"/cmd_hover",1,&CrazyflieROS::cmdHoverSetpoint, this);

    m_advertiseSelfDestruct = n.advertise<std_msgs::Int16>("crazyflie_selfdestruct", 1);

    if (m_enableLogging) {
      startLoggingFile(&n);

      if (m_enableLoggingPose) {
        m_pubPose = n.advertise<geometry_msgs::PoseStamped>(m_tf_prefix + "/pose", 10);
      }
    }
    m_runner = std::thread(&CrazyflieROS::runner, this);
  }

  ~CrazyflieROS()
  {
    m_endOfLife = true;
    m_runner.join();
    m_logBlocks.clear();
    m_logBlocksGeneric.clear();
    m_logFile.close();
    ROS_INFO("[%s] Last Breath", m_tf_prefix.c_str());
  }

  void runner() 
  {
    ROS_INFO("[%s] Starting runner thread", m_tf_prefix.c_str());    

    ros::NodeHandle nl("~");
    bool enableLogging;
    nl.getParam("enable_logging", enableLogging);
  
    int logTime = 1000;
    if (m_logBlocks.size()) {
      logTime /= (int)m_logBlocks.size();
      int freqCnt = 0; 
      for (auto logBlock : m_logBlocks) {
        freqCnt += (int)logBlock.frequency;
      }
      if (!freqCnt) enableLogging = false;
      else logTime /= freqCnt;
    } else enableLogging = false;

    std::chrono::steady_clock::time_point logTimer = std::chrono::steady_clock::now();
    while (!m_endOfLife) {
      if (enableLogging) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - logTimer).count();
        if (elapsed > logTime) {
          std::thread pingExecuter = std::thread(&CrazyflieROS::sendPing, this);
          pingExecuter.detach();
          logTimer = std::chrono::steady_clock::now();
        }
      }
      
      try {
        // See Ros Documentation, this is not how we should use these Exceptions.
        m_queue.callOne();
      } catch (const std::exception & ex) {
        ROS_WARN("[%s] Exception called, its end of Life.", m_tf_prefix.c_str());
        m_endOfLife = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    /* We could*/
    for (auto& logBlock : m_logBlocksGeneric) {
      delete &logBlock;
    }
  }

  void startLoggingFile(ros::NodeHandle * n) {
      m_logFile.open("logcf" + std::to_string(m_id) + ".csv");
      m_logFile << "time,";
      for (auto& logBlock : m_logBlocks) {
        m_pubLogDataGeneric.push_back(n->advertise<crazyswarm::GenericLogData>(m_tf_prefix + "/" + logBlock.topic_name, 10));
        for (const auto& variableName : logBlock.variables) {
          m_logFile << variableName << ",";
        }
      }
      m_logFile << std::endl;
  } 

  const std::string& tf_prefix() const {
    return m_tf_prefix;
  }

  const int id() const {
    return m_id;
  }

  const std::string& type() const {
    return m_type;
  }

  void setEndOfLife() {
    m_endOfLife = true;
  }


public:
  void sendPing() {
    m_cf.sendPing();
  }

  template<class T, class U>
  void updateParam(uint16_t id, const std::string& ros_param) {
      U value;
      ros::param::get(ros_param, value);
      m_cf.addSetParam<T>(id, (T)value);
  }

  void updateParams(
    const crazyswarm::UpdateParams::ConstPtr& req)
  {
    ROS_INFO("[%s] Update parameters", m_tf_prefix.c_str());
    m_cf.startSetParamRequest();
    for (auto&& p : req->params) {
      std::string ros_param = "/" + m_tf_prefix + "/" + p;
      size_t pos = p.find("/");
      std::string group(p.begin(), p.begin() + pos);
      std::string name(p.begin() + pos + 1, p.end());

      auto entry = m_cf.getParamTocEntry(group, name);
      if (entry)
      {
        switch (entry->type) {
          case Crazyflie::ParamTypeUint8:
            updateParam<uint8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt8:
            updateParam<int8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint16:
            updateParam<uint16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt16:
            updateParam<int16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint32:
            updateParam<uint32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt32:
            updateParam<int32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeFloat:
            updateParam<float, float>(entry->id, ros_param);
            break;
        }
      }
      else {
        ROS_ERROR("Could not find param %s/%s", group.c_str(), name.c_str());
      }
    }
    m_cf.setRequestedParams();
  }


  void uploadTrajectory(
    const crazyswarm::UploadTrajectory::ConstPtr& req)
  {
    ROS_INFO("[%s] Upload trajectory", m_tf_prefix.c_str());

    std::vector<Crazyflie::poly4d> pieces(req->pieces.size());
    for (size_t i = 0; i < pieces.size(); ++i) {
      if (   req->pieces[i].poly_x.size() != 8
          || req->pieces[i].poly_y.size() != 8
          || req->pieces[i].poly_z.size() != 8
          || req->pieces[i].poly_yaw.size() != 8) {
        ROS_FATAL("Wrong number of pieces!");
        return;
      }
      pieces[i].duration = req->pieces[i].duration.toSec();
      for (size_t j = 0; j < 8; ++j) {
        pieces[i].p[0][j] = req->pieces[i].poly_x[j];
        pieces[i].p[1][j] = req->pieces[i].poly_y[j];
        pieces[i].p[2][j] = req->pieces[i].poly_z[j];
        pieces[i].p[3][j] = req->pieces[i].poly_yaw[j];
      }
    }
    m_cf.uploadTrajectory(req->trajectoryId, req->pieceOffset, pieces);

    ROS_INFO("[%s] Uploaded trajectory", m_tf_prefix.c_str());
  }

  void startTrajectory(
    const crazyswarm::StartTrajectory::ConstPtr& req)
  {
    ROS_INFO("[%s] Start trajectory", m_tf_prefix.c_str());
    m_cf.startTrajectory(req->trajectoryId, req->timescale, req->reversed, 
          req->relative, req->groupMask);
  }

  void notifySetpointsStop(
    const crazyswarm::NotifySetpointsStop::ConstPtr& req)
  {
    ROS_INFO("[%s] NotifySetpointsStop requested", m_tf_prefix.c_str());
    m_cf.notifySetpointsStop(req->remainValidMillisecs);
  }

  void takeoff(
    const crazyswarm::Takeoff::ConstPtr& req)
  {
    ROS_INFO("[%s] Takeoff", m_tf_prefix.c_str());
    m_cf.takeoff(req->height, req->duration.toSec(), req->groupMask);
  }

  void land(
    const crazyswarm::Land::ConstPtr& req)
  {
    ROS_INFO("[%s] Land", m_tf_prefix.c_str());
    m_cf.land(req->height, req->duration.toSec(), req->groupMask);
  }

  void goTo(
    const crazyswarm::GoTo::ConstPtr& req)
  {
    ROS_INFO("[%s] GoTo", m_tf_prefix.c_str());
    m_cf.goTo(req->goal.x, req->goal.y, req->goal.z, req->yaw,
             req->duration.toSec(), req->relative, req->groupMask);
  }

  void setGroupMask(
    const crazyswarm::SetGroupMask::ConstPtr& req)
  {
    ROS_INFO("[%s] Set Group Mask", m_tf_prefix.c_str());
    m_cf.setGroupMask(req->groupMask);
  }

  void cmdVelChanged(
    const geometry_msgs::Twist::ConstPtr& msg)
  {
    //ROS_INFO("[%s] Cmd Vel Changed", m_tf_prefix.c_str());
    float roll = msg->linear.y;
    float pitch = -msg->linear.x;
    float yawrate = msg->angular.z;
    uint16_t thrust = (uint16_t)msg->linear.z;

    m_cf.sendSetpoint(roll, pitch, yawrate, thrust);
  }

  void cmdPositionSetpoint(
    const crazyswarm::Position::ConstPtr& msg)
  {
    //ROS_INFO("[%s] Cmd Position Setpoint", m_tf_prefix.c_str());
    float x = msg->x;
    float y = msg->y;
    float z = msg->z;
    float yaw = msg->yaw;
    m_cf.sendPositionSetpoint(x, y, z, yaw);
  }

  void cmdFullStateSetpoint(
    const crazyswarm::FullState::ConstPtr& msg)
  {
    //ROS_INFO("[%s] Cmd Full State Setpoint", m_tf_prefix.c_str());

    float x = msg->pose.position.x;
    float y = msg->pose.position.y;
    float z = msg->pose.position.z;
    float vx = msg->twist.linear.x;
    float vy = msg->twist.linear.y;
    float vz = msg->twist.linear.z;
    float ax = msg->acc.x;
    float ay = msg->acc.y;
    float az = msg->acc.z;

    float qx = msg->pose.orientation.x;
    float qy = msg->pose.orientation.y;
    float qz = msg->pose.orientation.z;
    float qw = msg->pose.orientation.w;
    float rollRate = msg->twist.angular.x;
    float pitchRate = msg->twist.angular.y;
    float yawRate = msg->twist.angular.z;

    m_cf.sendFullStateSetpoint(
      x, y, z,
      vx, vy, vz,
      ax, ay, az,
      qx, qy, qz, qw,
      rollRate, pitchRate, yawRate);
  }

  void cmdHoverSetpoint(const crazyswarm::Hover::ConstPtr& msg){
    //ROS_INFO("[%s] Cmd Hover Setpoint", m_tf_prefix.c_str());
    float vx = msg->vx;
    float vy = msg->vy;
    float yawRate = msg->yawrate;
    float zDistance = msg->zDistance;

    m_cf.sendHoverSetpoint(vx, vy, yawRate, zDistance);
  }
  void cmdVelocityWorldSetpoint(
    const crazyswarm::VelocityWorld::ConstPtr& msg)
  {
    //ROS_INFO("[%s] Cmd Vel Worl Setpoint", m_tf_prefix.c_str());

    float x = msg->vel.x;
    float y = msg->vel.y;
    float z = msg->vel.z;
    float yawRate = msg->yawRate;

    m_cf.sendVelocityWorldSetpoint(
      x, y, z, yawRate);
  }

  void cmdStop(
    const std_msgs::Empty::ConstPtr& msg)
  {
    //ROS_INFO("[%s] Cmd Stop", m_tf_prefix.c_str());
    m_cf.sendStop();
  }

  void run()
  {
    auto start = std::chrono::system_clock::now();

    std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
    m_cf.setLinkQualityCallback(cb_lq);

    m_cf.logReset();

    int numParams = 0;
    if (m_enableParameters)
    {
      ROS_INFO("[%s] Requesting parameters...", m_tf_prefix.c_str());
      m_cf.requestParamToc(m_forceNoCache);
      for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
        switch (entry.type) {
          case Crazyflie::ParamTypeUint8:
            ros::param::set(paramName, m_cf.getParam<uint8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt8:
            ros::param::set(paramName, m_cf.getParam<int8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint16:
            ros::param::set(paramName, m_cf.getParam<uint16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt16:
            ros::param::set(paramName, m_cf.getParam<int16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint32:
            ros::param::set(paramName, (int)m_cf.getParam<uint32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt32:
            ros::param::set(paramName, m_cf.getParam<int32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeFloat:
            ros::param::set(paramName, m_cf.getParam<float>(entry.id));
            break;
        }
        ++numParams;
      }
      ros::NodeHandle n;
      n.setCallbackQueue(&m_queue);
      m_subscribeUpdateParams = n.subscribe(m_tf_prefix + "/update_params", 5, &CrazyflieROS::updateParams, this);
    }
    auto end1 = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds1 = end1-start;
    ROS_INFO("[%s] reqParamTOC: %f s (%d params)", m_tf_prefix.c_str(), elapsedSeconds1.count(), numParams);

    // Logging
    if (m_enableLogging) {

      ROS_INFO("[%s] Requesting logging variables...", m_tf_prefix.c_str());
      m_cf.requestLogToc(m_forceNoCache);
      auto end2 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsedSeconds2 = end2-end1;
      ROS_INFO("[%s] reqLogTOC: %f s", m_tf_prefix.c_str(), elapsedSeconds2.count());

      m_logBlocksGeneric.resize(m_logBlocks.size());
      // custom log blocks
      size_t i = 0;
      for (auto& logBlock : m_logBlocks)
      {
        std::function<void(uint32_t, std::vector<double>*, void* userData)> cb =
          std::bind(
            &CrazyflieROS::onLogCustom,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

        m_logBlocksGeneric[i].reset(new LogBlockGeneric(
          &m_cf,
          logBlock.variables,
          (void*)&m_pubLogDataGeneric[i],
          cb));
        m_logBlocksGeneric[i]->start(100 / logBlock.frequency ); 
        ++i;
      }
      
      auto end3 = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsedSeconds3 = end3-end2;
      ROS_INFO("[%s] logBlocks: %f s", m_tf_prefix.c_str(), elapsedSeconds1.count());
    

      if (m_enableLoggingPose) {
        std::function<void(uint32_t, logPose*)> cb = std::bind(&CrazyflieROS::onPoseData, this, std::placeholders::_1, std::placeholders::_2);

        m_logBlockPose.reset(new LogBlock<logPose>(
          &m_cf,{
            {"stateEstimate", "x"},
            {"stateEstimate", "y"},
            {"stateEstimate", "z"},
            {"stateEstimateZ", "quat"}
          }, cb));
        m_logBlockPose->start(10); // 100ms
      }
    }
 
    ROS_INFO("[%s] Requesting memories...", m_tf_prefix.c_str());
    m_cf.requestMemoryToc();

    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    ROS_INFO("[%s] Ready. Elapsed: %f s", m_tf_prefix.c_str(), elapsedSeconds.count());

    
  }

  const Crazyflie::ParamTocEntry* getParamTocEntry(
    const std::string& group,
    const std::string& name) const
  {
    return m_cf.getParamTocEntry(group, name);
  }

  void initializePositionIfNeeded(float x, float y, float z)
  {
    if (m_initializedPosition) {
      return;
    }

    m_cf.startSetParamRequest();
    auto entry = m_cf.getParamTocEntry("kalman", "initialX");
    m_cf.addSetParam(entry->id, x);
    entry = m_cf.getParamTocEntry("kalman", "initialY");
    m_cf.addSetParam(entry->id, y);
    entry = m_cf.getParamTocEntry("kalman", "initialZ");
    m_cf.addSetParam(entry->id, z);
    m_cf.setRequestedParams();

    entry = m_cf.getParamTocEntry("kalman", "resetEstimation");
    m_cf.setParam<uint8_t>(entry->id, 1);

    // kalmanUSC might not be part of the firmware
    entry = m_cf.getParamTocEntry("kalmanUSC", "resetEstimation");
    if (entry) {
      m_cf.startSetParamRequest();
      entry = m_cf.getParamTocEntry("kalmanUSC", "initialX");
      m_cf.addSetParam(entry->id, x);
      entry = m_cf.getParamTocEntry("kalmanUSC", "initialY");
      m_cf.addSetParam(entry->id, y);
      entry = m_cf.getParamTocEntry("kalmanUSC", "initialZ");
      m_cf.addSetParam(entry->id, z);
      m_cf.setRequestedParams();

      entry = m_cf.getParamTocEntry("kalmanUSC", "resetEstimation");
      m_cf.setParam<uint8_t>(entry->id, 1);
    }

    m_initializedPosition = true;
  }

private:
  struct logPose {
    float x;
    float y;
    float z;
    int32_t quatCompressed;
  } __attribute__((packed));

private:

  void onLinkQuality(float linkQuality) { 
      static int linkQualityTimeout = 0; 
      if (linkQuality < 0.7) {
        ROS_WARN_NAMED(m_tf_prefix, "Link Quality low (%f)", linkQuality);
      }
  }

  void onConsole(const char* msg) {
    m_messageBuffer += msg;
    size_t pos = m_messageBuffer.find('\n');
    if (pos != std::string::npos) {
      m_messageBuffer[pos] = 0;
      ROS_INFO("[%s] %s", m_tf_prefix.c_str(), m_messageBuffer.c_str());
      m_messageBuffer.erase(0, pos+1);
    }
  }

  void onPoseData(uint32_t time_in_ms, logPose* data) {
    if (m_enableLoggingPose) {
      geometry_msgs::PoseStamped msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "world";

      msg.pose.position.x = data->x;
      msg.pose.position.y = data->y;
      msg.pose.position.z = data->z;

      float q[4];
      quatdecompress(data->quatCompressed, q);
      msg.pose.orientation.x = q[0];
      msg.pose.orientation.y = q[1];
      msg.pose.orientation.z = q[2];
      msg.pose.orientation.w = q[3];

      m_pubPose.publish(msg);


      tf::Transform tftransform;
      tftransform.setOrigin(tf::Vector3(data->x, data->y, data->z));
      tftransform.setRotation(tf::Quaternion(q[0], q[1], q[2], q[3]));
      m_br.sendTransform(tf::StampedTransform(tftransform, ros::Time::now(), "world", m_tf_prefix));
    }
  }

  void onLogCustom(uint32_t time_in_ms, std::vector<double>* values, void* userData) {

    ros::Publisher* pub = reinterpret_cast<ros::Publisher*>(userData);

    crazyswarm::GenericLogData msg;
    msg.header.stamp = ros::Time(time_in_ms/1000.0);
    msg.values = *values;

    m_logFile << time_in_ms / 1000.0 << ",";
    for (const auto& value : *values) {
      m_logFile << value << ",";
    }
    m_logFile << std::endl;

    pub->publish(msg);
  }

private:
  std::string m_tf_prefix;
  Crazyflie m_cf;
  std::string m_worldFrame;
  bool m_enableParameters;
  bool m_enableLogging;
  bool m_enableLoggingPose;
  int m_id;
  std::string m_type;

  ros::Subscriber m_subscribeUpdateParams;
  ros::Subscriber m_subscribeUploadTrajectory;
  ros::Subscriber m_subscribeStartTrajectory;
  ros::Subscriber m_subscribeTakeoff;
  ros::Subscriber m_subscribeLand;
  ros::Subscriber m_subscribeGoTo;
  ros::Subscriber m_subscribeSetGroupMask;
  ros::Subscriber m_subscribeNotifySetpointsStop;
  ros::Publisher m_advertiseSelfDestruct;

  ros::Subscriber m_subscribeCmdVel;
  ros::Subscriber m_subscribeCmdPosition;
  ros::Subscriber m_subscribeCmdFullState;
  ros::Subscriber m_subscribeCmdVelocityWorld;
  ros::Subscriber m_subscribeCmdStop;
  ros::Subscriber m_subscribeCmdHover; // Hover vel subscriber

  tf::TransformBroadcaster m_br;

  std::vector<crazyswarm::LogBlock> m_logBlocks;
  std::vector<ros::Publisher> m_pubLogDataGeneric;
  std::vector<std::unique_ptr<LogBlockGeneric> > m_logBlocksGeneric;

  ros::Subscriber m_subscribeJoy;

  ros::Publisher m_pubPose;
  std::unique_ptr<LogBlock<logPose>> m_logBlockPose;

  std::ofstream m_logFile;
  bool m_forceNoCache;
  bool m_initializedPosition;
  std::string m_messageBuffer;

  ros::CallbackQueue m_queue;
  bool m_endOfLife;
  std::thread m_runner;
};

// handles all Crazyflies
class CrazyflieServer
{
private:
  struct latency
  {
    double objectTracking;
    double broadcasting;
  };
  struct rosConfiguration  
  { 
    std::string objectTrackingType;
    bool useMotionCaptureObjectTracking;
    std::string logFilePath;
    std::string interactiveObject;
    bool printLatency;
    bool writeCSVs;
    bool sendPositionOnly;
    bool disableWarnings;
    std::string motionCaptureType;

    std::vector<std::string> genericLogTopics;
    std::vector<int> genericLogTopicFrequencies;
  } rosConfig;

  struct CFConfig 
  {
    std::string uri;
    std::string tf_prefix;
    std::string frame;
    int idNumber;
    std::string type;
  };

  struct latencyEntry {
        std::string name;
        double secs;
  };

public:
  CrazyflieServer()
    : m_isEmergency(false)
    , m_broadcastingNumRepeats(15)
    , m_broadcastingDelayBetweenRepeatsMs(1)
    , m_br()
    , m_phaseStart()

  {
    ros::NodeHandle nh;
    nh.setCallbackQueue(&m_queue);

    m_serviceEmergency = nh.advertiseService("emergency", &CrazyflieServer::emergency, this);
    m_subscribeStartTrajectory = nh.subscribe("start_trajectory", 5, &CrazyflieServer::startTrajectory, this);
    m_subscribeTakeoff = nh.subscribe("takeoff", 5, &CrazyflieServer::takeoff, this);
    m_subscribeLand = nh.subscribe("land", 5, &CrazyflieServer::land, this);
    m_subscribeGoTo = nh.subscribe("go_to", 5, &CrazyflieServer::goTo, this);
    m_subscribeUpdateParams = nh.subscribe("update_params", 5, &CrazyflieServer::updateParams, this);

    m_pubPointCloud = nh.advertise<sensor_msgs::PointCloud>("pointCloud", 1);

  
    //Vinzenz 
    m_serviceAddCrazyflie = nh.advertiseService("add_crazyflie", &CrazyflieServer::addCrazyflieCallback, this);
    m_serviceRemoveCrazyflie = nh.advertiseService("remove_crazyflie", &CrazyflieServer::removeCrazyflieCallback, this);

    m_subscribeCrazyflieSelfdestruct = nh.subscribe("crazyflie_selfdestruct", 1, &CrazyflieServer::crazyflieSelfdestructCallback, this);
  }

  ~CrazyflieServer()
  {
    for (CrazyflieROS* cf : m_cfs) {
      delete cf;
    }
    delete m_tracker;
  }


public:
  void run()
  {
    std::thread tSlow(&CrazyflieServer::runSlow, this);
    initialize();
    runFast();
    tSlow.join();
  }

  void runSlow()
  {
    while(ros::ok() && !m_isEmergency) {
      m_queue.callAvailable(ros::WallDuration(0));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void initialize() 
  {
    std::vector<libobjecttracker::DynamicsConfiguration> dynamicsConfigurations;
    std::vector<libobjecttracker::MarkerConfiguration> markerConfigurations;
    std::set<int> channels;

    readMarkerConfigurations(markerConfigurations);
    readDynamicsConfigurations(dynamicsConfigurations);
    readChannels(channels);    

    readRosConfiguration(&rosConfig);
    readLogBlocks(&rosConfig);
    initializePointCloudLogger(&rosConfig);
    initializeMocap(&rosConfig);        

    // Read all Crazyflies and create Broadcasters
    std::vector<libobjecttracker::Object> objects;
    std::vector<CFConfig> cfConfigs;

    int r = 0;
    ROS_INFO("Using %d radios", (int)channels.size());
    for (int channel : channels) {
      CrazyflieBroadcaster* cfbc = new CrazyflieBroadcaster("radio://" + std::to_string(r) + "/" + std::to_string(channel) + "/2M/FFE7E7E7E7");
      m_cfbcs.push_back(cfbc);
      readObjects(objects, cfConfigs, channel, m_logBlocks, r); 
      ++r;
    }

    // Initialize Tracker
    m_tracker = new libobjecttracker::ObjectTracker(
      dynamicsConfigurations,
      markerConfigurations,
      objects);
    m_tracker->setLogWarningCallback(logWarn);

    //Add Crazyflies
    for (const auto& config : cfConfigs) {
      addCrazyflie(config.uri, config.tf_prefix, "/world", config.idNumber, config.type, m_logBlocks);  
    }

    if (rosConfig.writeCSVs) {
      m_outputCSVs.resize(m_cfs.size());
      for (auto& output : m_outputCSVs) {
        output.reset(new std::ofstream);
      }
    }

    ROS_INFO("Started %lu Crazyflie Threads", m_cfs.size());
  }

  void runFast()
  {
    ros::NodeHandle nl("~");
    // If we use motion capture positions need to get broadcasted
    if (m_mocap) {
      auto startTime = std::chrono::high_resolution_clock::now();
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);
      std::vector<CrazyflieBroadcaster::externalPose> states;

      std::vector<latencyEntry> latencies;
      std::vector<double> latencyTotal(6 + 3 * 2, 0.0);

      while (ros::ok() && !m_isEmergency) {
        m_mocap->waitForNextFrame();

        latencies.clear();
        auto startIteration = std::chrono::high_resolution_clock::now();
        double totalLatency = 0;
        trackMocapLatency(&latencies, &latencyTotal, totalLatency);
   
        if (!rosConfig.useMotionCaptureObjectTracking) {
          // Get pointcloud
          const auto& pointcloud = m_mocap->pointCloud();
          markers->clear();
          for (size_t i = 0; i < pointcloud.rows(); ++i) {
            const auto &point = pointcloud.row(i);
            markers->push_back(pcl::PointXYZ(point(0), point(1), point(2)));
          }
          publishPointCloud(markers);          
        } else if (rosConfig.useMotionCaptureObjectTracking) {
          // Get mocap rigid bodies
          m_mocapRigidBodies = m_mocap->rigidBodies();
        }

        states.clear();
        runTracker(&rosConfig, markers, states);
        broadcastPositions(states);

        auto endIteration = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = endIteration - startIteration;
        double elapsedSeconds = elapsed.count();
        if (elapsedSeconds > 0.09 && !rosConfig.disableWarnings) { // originally 100 Hz (0.009) -> 10Hz enough
          ROS_WARN("Latency too high! Is %f s.", elapsedSeconds);
        }

        if (rosConfig.printLatency) printLatencies(&latencies, &latencyTotal, totalLatency);
      }

      if (m_logClouds) m_pointCloudLogger->flush();
    }

  
  }



  void publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr markers) 
  {
      static sensor_msgs::PointCloud msgPointCloud;
      msgPointCloud.header.frame_id = "world";
      msgPointCloud.header.seq += 1;
      msgPointCloud.header.stamp = ros::Time::now();
      msgPointCloud.points.resize(markers->size());
      for (size_t i = 0; i < markers->size(); ++i) {
        const pcl::PointXYZ& point = markers->at(i);
        msgPointCloud.points[i].x = point.x;
        msgPointCloud.points[i].y = point.y;
        msgPointCloud.points[i].z = point.z;
      }
      m_pubPointCloud.publish(msgPointCloud);

      if (m_logClouds) {
        m_pointCloudLogger->log(markers);
      }
  }

  void trackMocapLatency(
    std::vector<latencyEntry>* latencies,
    std::vector<double>* latencyTotal, 
    double totalLatency) 
  {
    const auto& mocapLatency = m_mocap->latency();
    float totalMocapLatency = 0;
    for (const auto& item : mocapLatency) {
      totalMocapLatency += item.value();
    }
    if (totalMocapLatency > 0.035 && !rosConfig.disableWarnings) {
      std::stringstream sstr;
      sstr << "MoCap Latency high: " << totalMocapLatency << " s." << std::endl;
      for (const auto& item : mocapLatency) {
        sstr << "  Latency: " << item.name() << ": " << item.value() << " s." << std::endl;
      }
      ROS_WARN("%s", sstr.str().c_str());
    }
    if (rosConfig.printLatency) {
          size_t i = 0;
          for (const auto& item : mocapLatency) {
            (*latencies).push_back({item.name(), item.value()});
            (*latencyTotal)[i] += item.value();
            totalLatency += item.value();
            (*latencyTotal).back() += item.value();
          }
          ++i;
    }
  }

  void printLatencies(
    std::vector<latencyEntry>* latencies,
    std::vector<double>* latencyTotal, 
    double totalLatency)
  {
    static uint32_t latencyCount;
    latencyCount++;
    std::cout << "Latencies" << std::endl;
    for (auto& latency : *latencies) {
      std::cout << latency.name << ": " << latency.secs * 1000 << " ms" << std::endl;
    }
    std::cout << "Total " << totalLatency * 1000 << " ms" << std::endl;
    // // if (latencyCount % 100 == 0) {
      std::cout << "Avg " << latencyCount << std::endl;
      for (size_t i = 0; i < (*latencyTotal).size(); ++i) {
        std::cout << (*latencyTotal)[i] / latencyCount * 1000.0 << ",";
      }
      std::cout << std::endl;
    // // }
  }

  bool addCrazyflieCallback(
    crazyswarm::Crazyflie::Request& req,
    crazyswarm::Crazyflie::Response& res)
  {
  
    ros::NodeHandle nGlobal;

    int channel = req.channel;
    int id = req.id;
    std::string type = req.type.data;
     if (type.empty()) {
      logWarn("No type given, but needed");
      return false;
    } 
    geometry_msgs::Point pos =  req.initialPosition;
    
    int markerConfigurationIdx;
    nGlobal.getParam("crazyflieTypes/" + type + "/markerConfiguration", markerConfigurationIdx);
    int dynamicsConfigurationIdx;
    nGlobal.getParam("crazyflieTypes/" + type + "/dynamicsConfiguration", dynamicsConfigurationIdx);
  
    //find radio id
    int radio_id = -1;
    for (auto cfbc : m_cfbcs) {
       if (channel == cfbc->m_channel) {
          radio_id = cfbc->m_devId;
          break;
       }
    } 
    if (radio_id == -1) {
      radio_id = (int)m_cfbcs.size();
      CrazyflieBroadcaster* cfbc = new CrazyflieBroadcaster("radio://" + std::to_string(radio_id) + "/" + std::to_string(channel) + "/2M/FFE7E7E7E7");
      m_cfbcs.push_back(cfbc);
    }    

    std::stringstream sstr;
    sstr << std::setfill ('0') << std::setw(2) << std::hex << id;
    std::string idHex = sstr.str();
    std::string uri = "radio://" + std::to_string(radio_id) + "/" + std::to_string(channel) + "/2M/E7E7E7E7" + idHex;
    std::string tf_prefix = "cf" + std::to_string(id);

    
    Eigen::Affine3f m;
    m = Eigen::Translation3f(pos.x, pos.y, pos.z);
    try {
      // Blocks for a timeout second (defined in Crazyflie.h (Crazyflie_cpp), line 391) (fixed in crazyflie_cpp)
      // + the time used for updating the params, which can take a while with balanced sending
      addCrazyflie(uri, tf_prefix, "/world", id, type, m_logBlocks);    
    } catch ( const std::exception & ex){//}...) {
      // Crazyflie wasnt found
      logWarn("Crazyflie couldn't be found!");
      logWarn(ex.what());
      return false;
    }
    m_tracker->addObject(libobjecttracker::Object(markerConfigurationIdx, dynamicsConfigurationIdx, m, tf_prefix));
    addCfToParam(id, channel, type, pos);
    return true;
  }


  bool removeCrazyflieCallback(
    crazyswarm::Crazyflie::Request& req,
    crazyswarm::Crazyflie::Response& res)
  {
    
    ROS_WARN("Got called to remove CF %d", req.id);
    int id = req.id;
    int channel = req.channel;
    return removeCrazyflie(id);
    
  }
  bool removeCrazyflie(
    int id)
  {
    ros::NodeHandle nGlobal;
    removeCfFromParam(id);
    std::vector<CrazyflieROS*> cfs;
    for (auto cf : m_cfs) {
      if (cf->id() == id) {
        cf->setEndOfLife();
        //delete cf; // We would like to do this, but m_cf in CrazyflieROS cannt be deleted without error
        continue;
      }
      cfs.push_back(cf);
    }
    m_cfs.clear();
    for (auto cf : cfs) {
      m_cfs.push_back(cf);
    }
   
    std::string name = "cf" + std::to_string(id);
    m_tracker->removeObject(name);
    return true;
  }

  void initializePointCloudLogger(
    rosConfiguration * config) 
  {
    // tilde-expansion
    wordexp_t wordexp_result;
    if (wordexp(config->logFilePath.c_str(), &wordexp_result, 0) == 0) {
      // success - only read first result, could be more if globs were used
      config->logFilePath = wordexp_result.we_wordv[0];
    }
    wordfree(&wordexp_result);

    m_pointCloudLogger = new libobjecttracker::PointCloudLogger(config->logFilePath);
    m_logClouds = !config->logFilePath.empty();
  }

  void crazyflieSelfdestructCallback(const std_msgs::Int16::ConstPtr& msg)
  {
    int id = msg->data;
    crazyswarm::Crazyflie cf;
    cf.request.id  = id;
    cf.request.channel = 0;
    ROS_WARN("CF %d called for selfdestruct, calling Service!", id);
    //std::this_thread::sleep_for(std::chrono::milliseconds(200)); // sleep for 200ms / double the time for timeout 
    //ros::Duration(0.5).sleep(); // This got called during a Service call, which has to get finished
    // The number is strongly dependent on the timeout defined in Crazyflie.h (Crazyflie.cpp) line 391
    CrazyflieServer::removeCrazyflieCallback(cf.request, cf.response);
  }

private:

  bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL("Emergency requested!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto cfbc : m_cfbcs) {
        cfbc->emergencyStop();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }
    m_isEmergency = true;

    return true;
  }

  void takeoff(
    const crazyswarm::Takeoff::ConstPtr& req)
  {
    ROS_INFO("Takeoff!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto cfbc : m_cfbcs) {
        cfbc->takeoff(req->height, req->duration.toSec(), req->groupMask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }
  }

  void land(
    const crazyswarm::Land::ConstPtr& req)
  {
    ROS_INFO("Land!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto cfbc : m_cfbcs) {
          cfbc->land(req->height, req->duration.toSec(), req->groupMask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }
  }

  void goTo(
    const crazyswarm::GoTo::ConstPtr& req)
  {
    ROS_INFO("GoTo!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto cfbc : m_cfbcs) {
          cfbc->goTo(req->goal.x, req->goal.y, req->goal.z, req->yaw,
           req->duration.toSec(), req->groupMask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }
  }

  void startTrajectory(
    const crazyswarm::StartTrajectory::ConstPtr& req)
  {
    ROS_INFO("Start trajectory!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto cfbc : m_cfbcs) {
        cfbc->startTrajectory(req->trajectoryId, req->timescale, req->reversed, req->groupMask);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }
  }

  void updateParams(
    const crazyswarm::UpdateParams::ConstPtr& req)
  {
    ROS_INFO("UpdateParams!");

    for (size_t i = 0; i < m_broadcastingNumRepeats; ++i) {
      for (auto cfbc : m_cfbcs) {
        // ToDO
        //cfbc->updateParams(req->params);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(m_broadcastingDelayBetweenRepeatsMs));
    }
  }

  void initializeMocap(
    rosConfiguration * config)
  {
    ros::NodeHandle nl("~");
      // Make a new client
    std::map<std::string, std::string> cfg;
    std::string hostname;
    nl.getParam("motion_capture_host_name", hostname);
    cfg["hostname"] = hostname;
    if (nl.hasParam("motion_capture_interface_ip")) {
      std::string interface_ip;
      nl.param<std::string>("motion_capture_interface_ip", interface_ip);
      cfg["interface_ip"] = interface_ip;
    }

    if (config->motionCaptureType != "none") {
      ROS_INFO(
        "libmotioncapture connecting to %s at hostname '%s' - "
        "might block indefinitely if unreachable!",
        config->motionCaptureType.c_str(),
        hostname.c_str()
      );
      m_mocap.reset(libmotioncapture::MotionCapture::connect(config->motionCaptureType, cfg));
      if (!m_mocap) {
        throw std::runtime_error("Unknown motion capture type!");
      }
    }
  }


  void runTracker(
    rosConfiguration * config,
    pcl::PointCloud<pcl::PointXYZ>::Ptr markers,
    std::vector<CrazyflieBroadcaster::externalPose> &states) 
  {
    auto stamp = std::chrono::high_resolution_clock::now();
    
    
    if (config->useMotionCaptureObjectTracking) {
        for (auto cf : m_cfs) {
          bool found = publishRigidBody(cf->tf_prefix(), cf->id(), states);
          if (found) {
            cf->initializePositionIfNeeded(states.back().x, states.back().y, states.back().z);
          }
        }
    } else {
      // run object tracker
      {
        auto start = std::chrono::high_resolution_clock::now();
        m_tracker->update(markers); // Here we crash if adding to empty group
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedSeconds = end-start;
        m_latency.objectTracking = elapsedSeconds.count();
      }
      for (size_t i = 0; i < m_cfs.size(); ++i) { 
        if (m_tracker->objects()[i].lastTransformationValid()) {

          const Eigen::Affine3f& transform = m_tracker->objects()[i].transformation();
          Eigen::Quaternionf q(transform.rotation());
          const auto& translation = transform.translation();

          states.resize(states.size() + 1);
          states.back().id = m_cfs[i]->id();
          states.back().x = translation.x();
          states.back().y = translation.y();
          states.back().z = translation.z();
          states.back().qx = q.x();
          states.back().qy = q.y();
          states.back().qz = q.z();
          states.back().qw = q.w();

          m_cfs[i]->initializePositionIfNeeded(states.back().x, states.back().y, states.back().z);

          tf::Transform tftransform;
          tftransform.setOrigin(tf::Vector3(translation.x(), translation.y(), translation.z()));
          tftransform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
          m_br.sendTransform(tf::StampedTransform(tftransform, ros::Time::now(), "world", m_cfs[i]->tf_prefix()));

          if (m_outputCSVs.size() > 0) {
            std::chrono::duration<double> tDuration = stamp - m_phaseStart;
            double t = tDuration.count();
            auto rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
            *m_outputCSVs[i] << t << "," << states.back().x << "," << states.back().y << "," << states.back().z
                                  << "," << rpy(0) << "," << rpy(1) << "," << rpy(2) << "\n";
          }
        } else if (!config->disableWarnings) {
          std::chrono::duration<double> elapsedSeconds = stamp - m_tracker->objects()[i].lastValidTime();
          ROS_WARN("No updated pose for CF %s for %f s.",
            m_cfs[i]->tf_prefix().c_str(),
            elapsedSeconds.count());
        }
      }
    }
    

 
  }

  void broadcastPositions(
    std::vector<CrazyflieBroadcaster::externalPose> &states
  )
  {
   /** There is a tradeoff to make:
      *  if we broadcast positions collision avoidance will work, if we send them out seperately it wont.
      * But the radio communication is more effective if we dont broadcast. 
      */
    bool broadcastPositions = true;
    if (broadcastPositions) {
      auto start = std::chrono::high_resolution_clock::now();
      if (!rosConfig.sendPositionOnly) {
        for (auto cfbc : m_cfbcs) {
          cfbc->sendExternalPoses(states);
        }
      } else {
        std::vector<CrazyflieBroadcaster::externalPosition> positions(states.size());
        for (size_t i = 0; i < positions.size(); ++i) {
          positions[i].id = states[i].id;
          positions[i].x  = states[i].x;
          positions[i].y  = states[i].y;
          positions[i].z  = states[i].z;
        }
        for (auto cfbc : m_cfbcs) {
          cfbc->sendExternalPositions(positions);
        }
      }
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsedSeconds = end-start;
      m_latency.broadcasting = elapsedSeconds.count();
      // totalLatency += elapsedSeconds.count();
      // ROS_INFO("Broadcasting: %f s", elapsedSeconds.count());
    } else {
      // send through crazyflie
    }

  }

  bool publishRigidBody(const std::string& name, uint8_t id, std::vector<CrazyflieBroadcaster::externalPose> &states)
  {
    assert(m_pMocapRigidBodies);
    const auto& iter = m_pMocapRigidBodies->find(name);
    if (iter != m_pMocapRigidBodies->end()) {
      const auto& rigidBody = iter->second;

      states.resize(states.size() + 1);
      states.back().id = id;
      states.back().x = rigidBody.position().x();
      states.back().y = rigidBody.position().y();
      states.back().z = rigidBody.position().z();
      states.back().qx = rigidBody.rotation().x();
      states.back().qy = rigidBody.rotation().y();
      states.back().qz = rigidBody.rotation().z();
      states.back().qw = rigidBody.rotation().w();

      tf::Transform transform;
      transform.setOrigin(tf::Vector3(
        states.back().x,
        states.back().y,
        states.back().z));
      tf::Quaternion q(
        states.back().qx,
        states.back().qy,
        states.back().qz,
        states.back().qw);
      transform.setRotation(q);
      m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
      return true;
    } else {
      ROS_WARN("No updated pose for motion capture object %s", name.c_str());
    }
    return false;
  }


  // Reads all available marker configurations, adds them to parameter server
  void readMarkerConfigurations(
    std::vector<libobjecttracker::MarkerConfiguration>& markerConfigurations)
  {
    markerConfigurations.clear();
    ros::NodeHandle nGlobal;
    int numConfigurations;
    nGlobal.getParam("numMarkerConfigurations", numConfigurations);
    for (int i = 0; i < numConfigurations; ++i) {
      markerConfigurations.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
      std::stringstream sstr;
      sstr << "markerConfigurations/" << i << "/numPoints";
      int numPoints;
      nGlobal.getParam(sstr.str(), numPoints);

      std::vector<double> offset;
      std::stringstream sstr2;
      sstr2 << "markerConfigurations/" << i << "/offset";
      nGlobal.getParam(sstr2.str(), offset);
      for (int j = 0; j < numPoints; ++j) {
        std::stringstream sstr3;
        sstr3 << "markerConfigurations/" << i << "/points/" << j;
        std::vector<double> points;
        nGlobal.getParam(sstr3.str(), points);
        markerConfigurations.back()->push_back(pcl::PointXYZ(points[0] + offset[0], points[1] + offset[1], points[2] + offset[2]));
      }
    }
  }

  // Reads all available dynamic configurations, adds them to parameter server
  void readDynamicsConfigurations(
    std::vector<libobjecttracker::DynamicsConfiguration>& dynamicsConfigurations)
  {
    ros::NodeHandle nGlobal;
    int numConfigurations;
    nGlobal.getParam("numDynamicsConfigurations", numConfigurations);
    dynamicsConfigurations.resize(numConfigurations);
    for (int i = 0; i < numConfigurations; ++i) {
      std::stringstream sstr;
      sstr << "dynamicsConfigurations/" << i;
      nGlobal.getParam(sstr.str() + "/maxXVelocity", dynamicsConfigurations[i].maxXVelocity);
      nGlobal.getParam(sstr.str() + "/maxYVelocity", dynamicsConfigurations[i].maxYVelocity);
      nGlobal.getParam(sstr.str() + "/maxZVelocity", dynamicsConfigurations[i].maxZVelocity);
      nGlobal.getParam(sstr.str() + "/maxPitchRate", dynamicsConfigurations[i].maxPitchRate);
      nGlobal.getParam(sstr.str() + "/maxRollRate", dynamicsConfigurations[i].maxRollRate);
      nGlobal.getParam(sstr.str() + "/maxYawRate", dynamicsConfigurations[i].maxYawRate);
      nGlobal.getParam(sstr.str() + "/maxRoll", dynamicsConfigurations[i].maxRoll);
      nGlobal.getParam(sstr.str() + "/maxPitch", dynamicsConfigurations[i].maxPitch);
      nGlobal.getParam(sstr.str() + "/maxFitnessScore", dynamicsConfigurations[i].maxFitnessScore);
    }
  }

  void readChannels(
    std::set<int>& channels)
  {
    // read CF config
    ros::NodeHandle nGlobal;

    if (!nGlobal.hasParam("crazyflies")) return;
    XmlRpc::XmlRpcValue crazyflies;
    nGlobal.getParam("crazyflies", crazyflies);
    ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);

    channels.clear();
    for (int32_t i = 0; i < crazyflies.size(); ++i) {
      ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
      int channel = crazyflie["channel"];
      channels.insert(channel);
    }
  }

  void readRosConfiguration(
    rosConfiguration * config)
  {
    ros::NodeHandle nl("~");
    nl.getParam("object_tracking_type", config->objectTrackingType);
    config->useMotionCaptureObjectTracking = (config->objectTrackingType == "motionCapture");
    nl.param<std::string>("save_point_clouds", config->logFilePath, "");
    nl.param<std::string>("interactive_object", config->interactiveObject, "");
    nl.getParam("print_latency", config->printLatency);
    nl.getParam("write_csvs", config->writeCSVs);
    nl.param<std::string>("motion_capture_type", config->motionCaptureType, "vicon");

    nl.param<int>("broadcasting_num_repeats", m_broadcastingNumRepeats, 15);
    nl.param<int>("broadcasting_delay_between_repeats_ms", m_broadcastingDelayBetweenRepeatsMs, 1);
    nl.param<bool>("send_position_only", config->sendPositionOnly, false);
    nl.param<bool>("disable_warnings", config->disableWarnings, false);

    nl.param("genericLogTopics", config->genericLogTopics, std::vector<std::string>());
    nl.param("genericLogTopicFrequencies", config->genericLogTopicFrequencies, std::vector<int>());
  }

  void readLogBlocks(
    rosConfiguration * config) 
  {
    ros::NodeHandle nl("~");
    std::vector<crazyswarm::LogBlock> logBlocks;
    if (config->genericLogTopics.size() == config->genericLogTopicFrequencies.size())
    {
      size_t i = 0;
      for (auto& topic : config->genericLogTopics)
      {
        crazyswarm::LogBlock logBlock;
        logBlock.topic_name = topic;
        logBlock.frequency = config->genericLogTopicFrequencies[i];
        nl.getParam("genericLogTopic_" + topic + "_Variables", logBlock.variables);
        logBlocks.push_back(logBlock);
        ++i;
      }
    }
    else
    {
      ROS_ERROR("Cardinality of genericLogTopics and genericLogTopicFrequencies does not match!");
    }
    m_logBlocks = logBlocks;
  }


  void readObjects(
    std::vector<libobjecttracker::Object>& objects,
    std::vector<CFConfig>& cfConfigs,
    int channel,
    const std::vector<crazyswarm::LogBlock>& logBlocks,
    int radio_id)
  {
    ros::NodeHandle nGlobal;
    if (!nGlobal.hasParam("crazyflies")) return;

    XmlRpc::XmlRpcValue crazyflies;
    nGlobal.getParam("crazyflies", crazyflies);
    ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);    

    for (int32_t i = 0; i < crazyflies.size(); ++i) {
      ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
      int id = crazyflie["id"];
      int ch = crazyflie["channel"];
      std::string type = crazyflie["type"];
      if (ch == channel) {
        XmlRpc::XmlRpcValue pos = crazyflie["initialPosition"];
        ROS_ASSERT(pos.getType() == XmlRpc::XmlRpcValue::TypeArray);

        std::vector<double> posVec(3);
        for (int32_t j = 0; j < pos.size(); ++j) {
          switch (pos[j].getType()) {
          case XmlRpc::XmlRpcValue::TypeDouble:
            posVec[j] = static_cast<double>(pos[j]);
            break;
          case XmlRpc::XmlRpcValue::TypeInt:
            posVec[j] = static_cast<int>(pos[j]);
            break;
          default:
            std::stringstream message;
            message << "crazyflies.yaml error:"
              " entry " << j << " of initialPosition for cf" << id <<
              " should be type int or double.";
            throw std::runtime_error(message.str().c_str());
          }
        }
        Eigen::Affine3f m;
        m = Eigen::Translation3f(posVec[0], posVec[1], posVec[2]);
        int markerConfigurationIdx;
        nGlobal.getParam("crazyflieTypes/" + type + "/markerConfiguration", markerConfigurationIdx);
        int dynamicsConfigurationIdx;
        nGlobal.getParam("crazyflieTypes/" + type + "/dynamicsConfiguration", dynamicsConfigurationIdx);
        std::string name = "cf" + std::to_string(id);
        
        std::stringstream sstr;
        sstr << std::setfill ('0') << std::setw(2) << std::hex << id;
        std::string idHex = sstr.str();

        std::string uri = "radio://" + std::to_string(radio_id) + "/" + std::to_string(channel) + "/2M/E7E7E7E7" + idHex;
        std::string tf_prefix = "cf" + std::to_string(id);
        std::string frame = "cf" + std::to_string(id);

        objects.push_back(libobjecttracker::Object(markerConfigurationIdx, dynamicsConfigurationIdx, m, name));
        cfConfigs.push_back({uri, tf_prefix, frame, id, type});
      }
    }
    ROS_INFO("Parsed crazyflies.yaml successfully!"); 
  }

  void addCrazyflie(
    const std::string& uri,
    const std::string& tf_prefix,
    const std::string& worldFrame,
    int id,
    const std::string& type,
    const std::vector<crazyswarm::LogBlock>& logBlocks)
  {
    ROS_INFO("Adding CF: %s (%s, %s)...", tf_prefix.c_str(), uri.c_str(), tf_prefix.c_str());
    auto start = std::chrono::high_resolution_clock::now();
    CrazyflieROS* cf = new CrazyflieROS(
      uri,
      tf_prefix,
      worldFrame,
      id,
      type,
      logBlocks);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    ROS_INFO("[%s] CF ctor: %f s", tf_prefix.c_str(), elapsed.count());
    cf->run();
    auto end2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed2 = end2 - end;
    ROS_INFO("[%s] CF run: %f s", tf_prefix.c_str(), elapsed2.count());
    start = std::chrono::high_resolution_clock::now();
    updateParams(cf);
    auto end3 = std::chrono::high_resolution_clock::now();
    elapsed = end3 - start;
    ROS_INFO("[%s] Update params: %f s",tf_prefix.c_str(), elapsed.count());

    m_cfs.push_back(cf);
  }

  // Add crazyflie to parameter server
  // See XmlRpcpp Documentation for more detail 
  void addCfToParam (int id, int channel, std::string type, geometry_msgs::Point pos) {
    ros::NodeHandle nGlobal;

    int index = 0;
    XmlRpc::XmlRpcValue crazyflies;
    if (nGlobal.hasParam("crazyflies")) {
      nGlobal.getParam("crazyflies", crazyflies);
      index = crazyflies.size();
    }
    XmlRpc::XmlRpcValue new_cf; 
    new_cf["id"] = id;
    new_cf["channel"] = channel;
    new_cf["type"] = type;
    new_cf["initialPosition"][0] = pos.x;
    new_cf["initialPosition"][1] = pos.y;
    new_cf["initialPosition"][2] = pos.z;

    crazyflies[index] = new_cf;
    nGlobal.setParam("crazyflies", crazyflies);
  }

  // Remove crazyflie from parameter Server
  bool removeCfFromParam (int id, int channel = 0) {
    ros::NodeHandle nGlobal;
    if (!nGlobal.hasParam("crazyflies")) return false;

    XmlRpc::XmlRpcValue crazyflies;
    nGlobal.getParam("crazyflies", crazyflies);
    XmlRpc::XmlRpcValue new_crazyflies;
    int cnt = 0;
    for (int i = 0; i < crazyflies.size(); i++) {
      if (static_cast<int>(crazyflies[i]["id"]) == id && (static_cast<int>(crazyflies[i]["channel"]) == channel || !channel) ) continue;
      cnt++;
      new_crazyflies[cnt] = crazyflies[i];
    }
    if (!cnt) nGlobal.deleteParam("crazyflies");
    else nGlobal.setParam("crazyflies", new_crazyflies);

    return true;
  }

 
  void updateParams(
    CrazyflieROS* cf)
  {
    ros::NodeHandle n("~");
    ros::NodeHandle nGlobal;
    // update parameters
    // std::cout << "attempt: " << "firmwareParams/" + cf->type() << std::endl;
    // char dummy;
    // std::cin >> dummy;

    // update global, type-specific, and CF-specific parameters
    std::vector<XmlRpc::XmlRpcValue> firmwareParamsVec(2);
    n.getParam("firmwareParams", firmwareParamsVec[0]);
    nGlobal.getParam("crazyflieTypes/" + cf->type() + "/firmwareParams", firmwareParamsVec[1]);

    // There might be Firmware Params in the crazyflies.yaml
    if (nGlobal.hasParam("crazyflies")) {
      XmlRpc::XmlRpcValue crazyflies;
      nGlobal.getParam("crazyflies", crazyflies);
      ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);
      for (int32_t i = 0; i < crazyflies.size(); ++i) {
        ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
        int id = crazyflie["id"];
        if (id == cf->id()) {
          if (crazyflie.hasMember("firmwareParams")) {
            firmwareParamsVec.push_back(crazyflie["firmwareParams"]);
          }
          break;
        }
      }
    }


    crazyswarm::UpdateParams request;

    for (auto& firmwareParams : firmwareParamsVec) {
      // ROS_ASSERT(firmwareParams.getType() == XmlRpc::XmlRpcValue::TypeArray);
      auto iter = firmwareParams.begin();
      for (; iter != firmwareParams.end(); ++iter) {
        std::string group = iter->first;
        XmlRpc::XmlRpcValue v = iter->second;
        auto iter2 = v.begin();
        for (; iter2 != v.end(); ++iter2) {
          std::string param = iter2->first;
          XmlRpc::XmlRpcValue value = iter2->second;
          if (value.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
            bool b = value;
            nGlobal.setParam(cf->tf_prefix() + "/" + group + "/" + param, b);
            std::cout << "update " << group + "/" + param << " to " << b << std::endl;
          } else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
            int b = value;
            nGlobal.setParam(cf->tf_prefix() + "/" + group + "/" + param, b);
            std::cout << "update " << group + "/" + param << " to " << b << std::endl;
          } else if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            double b = value;
            nGlobal.setParam(cf->tf_prefix() + "/" + group + "/" + param, b);
            std::cout << "update " << group + "/" + param << " to " << b << std::endl;
          } else if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
            // "1e-5" is not recognize as double; convert manually here
            std::string value_str = value;
            double value = std::stod(value_str);
            nGlobal.setParam(cf->tf_prefix() + "/" + group + "/" + param, value);
            std::cout << "update " << group + "/" + param << " to " << value << std::endl;
          } else {
            ROS_ERROR("No known type for %s.%s! (type: %d)", group.c_str(), param.c_str(), value.getType());
          }
          request.params.push_back(group + "/" + param);

        }
      }
    }

    crazyswarm::UpdateParams::ConstPtr ptr(new crazyswarm::UpdateParams(request));
    cf->updateParams( ptr);
  }



private:
  std::string m_worldFrame;
  bool m_isEmergency;
  ros::ServiceServer m_serviceEmergency;
  ros::Subscriber m_subscribeStartTrajectory;
  ros::Subscriber m_subscribeTakeoff;
  ros::Subscriber m_subscribeLand;
  ros::Subscriber m_subscribeGoTo;
  ros::Subscriber m_subscribeUpdateParams;
  ros::Publisher m_pubPointCloud;

  libobjecttracker::PointCloudLogger* m_pointCloudLogger;
  bool m_logClouds;

  // tf::TransformBroadcaster m_br;
  std::unique_ptr<libmotioncapture::MotionCapture> m_mocap;
  libobjecttracker::ObjectTracker* m_tracker; // non-owning pointer
  
  std::map<std::string, libmotioncapture::RigidBody>* m_pMocapRigidBodies; // non-owning pointer
  latency m_latency;

  tf::TransformBroadcaster m_br;

  std::vector<CrazyflieBroadcaster*> m_cfbcs;


  std::vector<CrazyflieROS*> m_cfs;

  std::vector<std::unique_ptr<std::ofstream>> m_outputCSVs;


  ros::ServiceServer m_serviceAddCrazyflie;
  ros::ServiceServer m_serviceRemoveCrazyflie;

  ros::Subscriber m_subscribeCrazyflieSelfdestruct;

  int m_broadcastingNumRepeats;
  int m_broadcastingDelayBetweenRepeatsMs;

  
  std::map<std::string, libmotioncapture::RigidBody> m_mocapRigidBodies;
  std::vector<crazyswarm::LogBlock>  m_logBlocks;
  std::chrono::high_resolution_clock::time_point m_phaseStart;


private:
  // We have two callback queues
  // 1. Fast queue handles pose and emergency callbacks. Those are high-priority and can be served quickly
  // 2. Slow queue handles all other requests.
  // Each queue is handled in its own thread. We don't want a thread per CF to make sure that the fast queue
  //  gets called frequently enough.

  ros::CallbackQueue m_queue;
  // ros::CallbackQueue m_slowQueue;


};





int main(int argc, char **argv)
{
  // raise(SIGSTOP);

  ros::init(argc, argv, "crazyflie_server");

  // ros::NodeHandle n("~");
  // std::string worldFrame;
  // n.param<std::string>("world_frame", worldFrame, "/world");
  // std::string broadcastUri;
  // n.getParam("broadcast_uri", broadcastUri);

  CrazyflieServer server;//(broadcastUri, worldFrame);

  // read CF config
  ros::NodeHandle nGlobal;

  if (nGlobal.hasParam("crazyflies")) {

    XmlRpc::XmlRpcValue crazyflies;
    nGlobal.getParam("crazyflies", crazyflies);
    ROS_ASSERT(crazyflies.getType() == XmlRpc::XmlRpcValue::TypeArray);

    std::set<int> cfIds;
    for (int32_t i = 0; i < crazyflies.size(); ++i)
    {
      ROS_ASSERT(crazyflies[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      XmlRpc::XmlRpcValue crazyflie = crazyflies[i];
      int id = crazyflie["id"];
      int channel = crazyflie["channel"];
      if (cfIds.find(id) != cfIds.end()) {
        ROS_FATAL("CF with the same id twice in configuration!");
        return 1;
      }
      cfIds.insert(id);
    } 

    // ROS_INFO("All CFs are ready!");
  }
  
  server.run();

  return 0;
}
