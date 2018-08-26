#include <boost/bind.hpp>
#include <ros/ros.h>

#include <ublox_msgs/NavPVT.h>
#include <ublox_msgs/NavRELPOSNED.h>
#include <can_msgs/Frame.h>
#include <geometry_msgs/PoseArray.h>

class IoniqControlNode
{
public:
    IoniqControlNode(ros::NodeHandle &_nh)
        : nh(_nh),
          bActive(false)
    {
        subUbloxPVT = nh.subscribe<ublox_msgs::NavPVT>("/m8p_rover/navpvt", 10, &IoniqControlNode::navPVTCallback, this);
        subUbloxRELPOS = nh.subscribe<ublox_msgs::NavRELPOSNED>("/m8p_rover/navrelposened", 10, &IoniqControlNode::navRELPOSCallback, this);
        subCANACC = nh.subscribe<can_msgs::Frame>("/gcan/recv_msgs", 10, &IoniqControlNode::gcanCallback, this);
        pubCANCmd = nh.advertise<can_msgs::Frame>("/gcan/send_msgs", 10);
        pubVizPath = nh.advertise<geometry_msgs::PoseArray>("/path", 10);
        timerCmdCalc = nh.createTimer(ros::Duration(0.05), &IoniqControlNode::timerCallback, this);
    }
    void navPVTCallback(const ublox_msgs::NavPVT::ConstPtr &_msg)
    {
        if(_msg->flags & (ublox_msgs::NavPVT::CARRIER_PHASE_FLOAT | ublox_msgs::NavPVT::CARRIER_PHASE_FIXED))
        {
            ;
        }
        ROS_WARN("ublox receiver is not rtk.");
    }
    void navRELPOSCallback(const ublox_msgs::NavRELPOSNED::ConstPtr &_msg)
    {
        if(_msg->flags & (ublox_msgs::NavRELPOSNED::FLAGS_CARR_SOLN_FLOAT | ublox_msgs::NavRELPOSNED::FLAGS_CARR_SOLN_FIXED))
        {
            _msg->relPosN; _msg->relPosE; _msg->relPosD;
        }
    }
    void gcanCallback(const can_msgs::Frame::ConstPtr &_msg)
    {
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
        //stanley
        geometry_msgs::PoseArray path;
        for (unsigned int i=0; i < path.poses.size(); i++)
        {

        }
        geometry_msgs::Pose targetPose;
        geometry_msgs::Pose currentPose;



    }

private:
    ros::NodeHandle nh;
    ros::NodeHandle ph;
    ros::Subscriber subUbloxPVT;
    ros::Subscriber subUbloxRELPOS;
    ros::Subscriber subCANACC;
    ros::Publisher pubCANCmd;
    ros::Publisher pubVizPath;

    //TODO: add follow
    // vehicle pose w.r.t.
    //
    ros::Timer timerCmdCalc;
    bool bActive;
};

int main (int argc, char ** argv)
{
    ros::init(argc, argv, "ioniq_control_node");
    ros::NodeHandle nh;
    IoniqControlNode ioniqControlNode(nh);
    ros::spin();
    return 0;
}

