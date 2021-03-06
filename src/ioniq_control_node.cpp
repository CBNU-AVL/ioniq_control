#include <vector>
#include <fstream>
#include <sstream>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <ublox_msgs/NavPVT.h>
#include <ublox_msgs/NavRELPOSNED.h>
#include <can_msgs/Frame.h>

class IoniqControlNode
{
public:
  IoniqControlNode(ros::NodeHandle &_nh, ros::NodeHandle &_ph)
    : nh(_nh), ph(_ph),
      pathIndex(0),
      bEPSEnable(true),
      bEPSIgnore(true),
      steerAngle(0),
      steerSpeed(150),
      vehAccel(0),
      bACCEnable(true),
      bAEBEnable(false),
      clusterDispSpeed(0),
      bActive(false)
  {
    subUbloxPVT = nh.subscribe<ublox_msgs::NavPVT>("/m8p_rover/navpvt", 10, &IoniqControlNode::navPVTCallback, this);
    subUbloxRELPOS = nh.subscribe<ublox_msgs::NavRELPOSNED>("/m8p_rover/navrelposened", 10, &IoniqControlNode::navRELPOSCallback, this);
    subCANACC = nh.subscribe<can_msgs::Frame>("/gcan/recv_msgs", 10, &IoniqControlNode::gcanCallback, this);
    pubCANCmd = nh.advertise<can_msgs::Frame>("/gcan/sent_msgs", 10);
    pubVizPath = nh.advertise<geometry_msgs::PoseArray>("/path", 10);
    timerCmdCalc = nh.createTimer(ros::Duration(0.02), &IoniqControlNode::timerCallback, this);
    
    ROS_INFO("Initializing IoniqControlNode.");
    ph.param("path_file_name", filePath, std::string("PATH.txt"));
    
    //load gps path
    std::ifstream fin;
    double pathLon, pathLat, pathAlt;
    std::string input;
    bool error;
    double pathCount = 0;
    ROS_INFO("path_file_name: %s", filePath.c_str());

    fin.open(filePath);
    error = false;

    while (true)
      {
        getline(fin,input);
        if (!fin) 
        {
          break; //check for eof
        }

        std::istringstream buffer(input);

        if (buffer.eof())
          {
            error = true;
            pathCount = 0;
            break;
          }

        buffer >> pathCount >> pathLon >> pathLat >> pathAlt;
        geometry_msgs::Pose tmpPose;
        //FIXME: How do I convert long, lat, alt to cartesian coordinate?
        tmpPose.position.x = pathLat * 88000;
        tmpPose.position.y = pathLon * 110000;
        tmpPose.position.z = pathAlt;

        path.poses.push_back(tmpPose);

        //do what you want with temp1 and temp2
        ROS_INFO_STREAM("[" << pathCount << "] longitude: " << pathLon << '\t' << "latitude : " << pathLat  << '\t' << "altitude : " << pathAlt );
      }

    if (error)
      ROS_ERROR("path file is corrupted...");
    else
    {
      ROS_INFO("parse succeed.");
    }
  }

  void navPVTCallback(const ublox_msgs::NavPVT::ConstPtr &_msg)
  {
    if(_msg->flags & (ublox_msgs::NavPVT::CARRIER_PHASE_FLOAT | ublox_msgs::NavPVT::CARRIER_PHASE_FIXED))
      {
        currentVelocity.linear.x = _msg->velN * 0.001;
        currentVelocity.linear.y = _msg->velE * 0.001;
        currentVelocity.linear.z = _msg->velD * 0.001;
        if(_msg->flags & ublox_msgs::NavPVT::FLAGS_HEAD_VEH_VALID)
          {
            currentPose.pose.orientation.w = _msg->headVeh;
          }

      }
    ROS_WARN("ublox receiver is not rtk.");
  }

  void navRELPOSCallback(const ublox_msgs::NavRELPOSNED::ConstPtr &_msg)
  {
    if(_msg->flags & (ublox_msgs::NavRELPOSNED::FLAGS_CARR_SOLN_FLOAT | ublox_msgs::NavRELPOSNED::FLAGS_CARR_SOLN_FIXED))
      {
        //TODO: introduce tf for convenient transformation
        currentPose.header.frame_id = "gps";
        currentPose.header.stamp = ros::Time::now();
        //FIXME: the frame 'gps' follows NED coordinate?
        currentPose.pose.position.x = (_msg->relPosN + _msg->relPosHPN * 0.01) * 0.01;
        currentPose.pose.position.y = (_msg->relPosE + _msg->relPosHPE * 0.01) * 0.01;
        currentPose.pose.position.z = (_msg->relPosD + _msg->relPosHPD * 0.01) * 0.01;
      }
  }

  void gcanCallback(const can_msgs::Frame::ConstPtr &_msg)
  {
    //TODO: define custom message for GCAN
    if(_msg->id == 0x710)
      {
        if((_msg->data[0] & 0x01) == 0)
          {
            ROS_WARN("Steer control is not enabled.");
          }
      }
    if(_msg->id == 0x711)
      {
        ROS_INFO("  - R ID:%4x LEN:%1x DATA:%02x %02x %02x %02x %02x %02x %02x %02x\n",
                 _msg->id, _msg->dlc,
                 _msg->data[0], _msg->data[1],
            _msg->data[2], _msg->data[3],
            _msg->data[4], _msg->data[5],
            _msg->data[6], _msg->data[7]);
      }
  }

  void timerCallback(const ros::TimerEvent &_event)
  {
    if(!bActive)
      {
        return;
      }
    tf::Quaternion q(
          currentPose.pose.orientation.x,
          currentPose.pose.orientation.y,
          currentPose.pose.orientation.z,
          currentPose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    const double k = 1;
    //TODO: Select appropriate target point to track: Nearest point(crossroad may cause problems)
    if (pathIndex >= 1 && pathIndex < path.poses.size())
      {
        steerAngle = yaw
            - atan2(path.poses[pathIndex].position.y - path.poses[pathIndex - 1].position.y, path.poses[pathIndex].position.x - path.poses[pathIndex - 1].position.x + DBL_MIN)
            + atan2(k*distPose(path.poses[pathIndex], currentPose.pose), currentVelocity.linear.x + DBL_MIN);

        can_msgs::Frame fMoConf;
        fMoConf.id = 0x156;
        fMoConf.dlc = 8;
        fMoConf.data[0] = static_cast<unsigned char>(bEPSEnable | (bEPSIgnore << 2));
        fMoConf.data[1] = static_cast<unsigned char>(steerSpeed * 2);
        fMoConf.data[2] = static_cast<unsigned char>(bACCEnable | (bAEBEnable << 6));
        fMoConf.data[3] = clusterDispSpeed;
        fMoConf.data[7] = ++aliveCount;

        can_msgs::Frame fMoVal;
        fMoVal.id = 0x157;
        fMoVal.dlc = 8;
        uint16_t tmpSteerCmd = static_cast<uint16_t>(steerAngle * 10);
        uint16_t tmpAccelCmd = static_cast<uint16_t>(vehAccel * 100 + 10.23);
        uint8_t tmpAEBCmd = 1 * 100;
        fMoVal.data[0] = static_cast<unsigned char>(tmpSteerCmd & 0x00FF);
        fMoVal.data[1] = static_cast<unsigned char>((tmpSteerCmd & 0xFFFF) >> 8);
        fMoVal.data[3] = static_cast<unsigned char>(tmpAccelCmd & 0x00FF);
        fMoVal.data[4] = static_cast<unsigned char>((tmpAccelCmd & 0xFFFF) >> 8);
        fMoVal.data[5] = tmpAEBCmd;

        pubCANCmd.publish(fMoConf);
        pubCANCmd.publish(fMoVal);
      }
  }

  //utility functions
  static double distPose(const geometry_msgs::Pose &_a, const geometry_msgs::Pose &_b)
  {
    return sqrt(pow(_a.position.x - _b.position.x, 2)
                + pow(_a.position.y - _b.position.y, 2)
                + pow(_a.position.z - _b.position.z, 2));
  }

  //search the nearest point with a msg from rviz

private:
  ros::NodeHandle nh;
  ros::NodeHandle ph;
  ros::Subscriber subUbloxPVT;
  ros::Subscriber subUbloxRELPOS;
  ros::Subscriber subCANACC;
  ros::Publisher pubCANCmd;
  ros::Publisher pubVizPath;

  geometry_msgs::PoseArray path;
  unsigned int pathIndex;
  geometry_msgs::PoseStamped currentPose;
  geometry_msgs::Twist currentVelocity;
  bool bEPSEnable;
  bool bEPSIgnore;
  double steerAngle;
  double steerSpeed;
  double vehAccel;
  bool bACCEnable;
  bool bAEBEnable;
  //TODO: display target speed of longitudinal controller
  uint8_t clusterDispSpeed;
  uint8_t aliveCount;

  std::string filePath;

  ros::Timer timerCmdCalc;
  bool bActive;
};

int main (int argc, char ** argv)
{
  ros::init(argc, argv, "ioniq_control_node");
  ros::NodeHandle nh("");
  ros::NodeHandle ph("~");
  IoniqControlNode ioniqControlNode(nh, ph);
  ros::spin();
  return 0;
}

