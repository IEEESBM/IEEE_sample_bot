#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <geometry_msgs/Twist.h>
#include<time.h>
#include<cstdlib>
#include "control/FourWheeler.h"

namespace gazebo
{
  class FourWheelDifferential: public ModelPlugin
  {
    public:void Load(physics::ModelPtr parent, sdf::ElementPtr sdf_model)
    {
      this->model=parent;

      this->front_left=sdf_model->GetElement("front_left_joint")->Get<std::string>();
      this->front_right=sdf_model->GetElement("front_right_joint")->Get<std::string>();
      this->back_left=sdf_model->GetElement("back_left_joint")->Get<std::string>();
      this->back_right=sdf_model->GetElement("back_right_joint")->Get<std::string>();

      this->updateConnection=event::Events::ConnectWorldUpdateBegin(
        std::bind(&FourWheelDifferential::OnUpdate, this));

      std::string command_topic="/FourWheeler";

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "four_wheel_differential_rosnode",
            ros::init_options::NoSigintHandler);
      }

      this->rosNode.reset(new ros::NodeHandle("four_wheel_differential_rosnode"));

      ros::SubscribeOptions options=
        ros::SubscribeOptions::create<control::FourWheeler>(
          command_topic,
          1,
          boost::bind(&FourWheelDifferential::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
      this->rosSub=this->rosNode->subscribe(options);

      this->rosQueueThread =
        std::thread(std::bind(&FourWheelDifferential::QueueThread, this));

      ROS_WARN("Loaded this differential control mf");
    }

    void OnUpdate()
    {
      // ROS_WARN("updating");
      // ROS_WARN("%f %f", this->left_command, this->right_command);
      // ROS_WARN("%s", this->front_left.c_str());
      this->model->GetJoint(this->front_left.c_str())->SetVelocity(0, this->left_command);
      this->model->GetJoint(this->back_left.c_str())->SetVelocity(0, this->left_command);
      this->model->GetJoint(this->front_right.c_str())->SetVelocity(0, this->right_command);
      this->model->GetJoint(this->back_right.c_str())->SetVelocity(0, this->right_command);
    }

    void OnRosMsg(const control::FourWheeler::ConstPtr &msg)
    {
      ROS_WARN("sup");
      this->right_command=msg->right*10;
      this->left_command=msg->left*10;
    }

  private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    private: physics::ModelPtr model;
    private: std::string front_left;
    private: std::string front_right;
    private: std::string back_left;
    private: std::string back_right;
    private: event::ConnectionPtr updateConnection;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;
    private: float right_command;
    private: float left_command;
  };
  GZ_REGISTER_MODEL_PLUGIN(FourWheelDifferential)
}
