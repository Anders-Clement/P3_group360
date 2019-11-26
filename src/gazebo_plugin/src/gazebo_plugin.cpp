#ifndef CRUSTCRAWLER_CONTROL
#define CRUSTCRAWLER_CONTROL

#include <functional>
#include "boost/bind.hpp"
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "string.h"

namespace gazebo
{
class CrustCrawlerPlugin : public ModelPlugin
{
    ros::NodeHandle *n;
    ros::Publisher anglePublisher;
    ros::Subscriber angleSubscriber;
    physics::JointPtr joints[5];
    event::ConnectionPtr updateConnection;
    ros::Time prevRun;
    physics::ModelPtr crustyModel;
    double jointsLastAngle[5] = {0, 0, 0, 0, 0};
    double forceTorques[5] = {0,0,0,0,0};

public:
    CrustCrawlerPlugin() : ModelPlugin()
    {
        //ros::init(0, 0, "gazebo_plugin_node");
    }
    ~CrustCrawlerPlugin()
    {
        delete n;
        for (int i = 0; i < 5; i++)
        {
            delete &joints[i];
        }
        //delete joints;
    }

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        crustyModel = _model;

        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                             << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        n = new ros::NodeHandle();
        anglePublisher = n->advertise<std_msgs::Int16MultiArray>("/crustcrawler/getAngleVel", 1);
        angleSubscriber = n->subscribe("/crustcrawler/setTorques", 1, &CrustCrawlerPlugin::torqueUpdater, this);

        ROS_INFO("CrustCrawler Plugin has started, and is ready for combat!");
        ROS_INFO_STREAM("The plugin is attach to model: " << _model->GetName());

        joints[0] = _model->GetJoint("joint1");
        joints[1] = _model->GetJoint("joint2");
        joints[2] = _model->GetJoint("joint3");
        joints[3] = _model->GetJoint("joint4");
        joints[4] = _model->GetJoint("joint5");

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&CrustCrawlerPlugin::OnUpdate, this));
    }

public:
    void OnUpdate()
    {
        if (!ros::ok())
        {
            ROS_INFO("Failed Ros");
            return;
        }
        ros::Time currentTime = ros::Time::now();
        if (currentTime.nsec - prevRun.nsec > (1.0 / 30.0) * 1000000000.0)
        //if (currentTime.toSec() - prevRun.toSec() > (3.0))
        {
            prevRun = ros::Time::now();
            std_msgs::Int16MultiArray msg;
            msg.data.resize(10);
            for (int i = 0; i < 5; i++)
            {
                int dataIndex = (i * 2);
                double currentPose = joints[i]->Position(2);
                msg.data[dataIndex] = currentPose*1000.0;
                msg.data[dataIndex + 1] = ((jointsLastAngle[i] - currentPose)*1000.0)/(1.0/30.0);
                jointsLastAngle[i] = currentPose;
            }
            anglePublisher.publish(msg);
        }
        ros::spinOnce();

        for (int i = 0; i < 5; i++)
        {
            joints[i]->SetForce(0, forceTorques[i]);
        }
    }

    void torqueUpdater(const std_msgs::Float64MultiArray &incoming)
    {
        for (int i = 0; i < 5; i++)
        {
            forceTorques[i] = incoming.data[i];
        }
    }
};
GZ_REGISTER_MODEL_PLUGIN(CrustCrawlerPlugin)
} // namespace gazebo

#endif