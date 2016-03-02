# ifndef PR2JOINTVELOCITYCONTROLLERINTERFACE_H
# define PR2JOINTVELOCITYCONTROLLERINTERFACE_H

# include <ros/ros.h>

# include "pr2_controller_interface/controller.h"
# include "control_toolbox/pid.h"
# include "control_toolbox/pid_gains_setter.h"
# include <robot_mechanism_controllers/joint_velocity_controller.h>
# include <brics_actuator/JointVelocities.h>


# include <realtime_tools/realtime_publisher.h> 
# include <exception>
# include <std_msgs/Float64.h>
# include <std_msgs/Float32MultiArray.h>
class pr2JointVelocityControllerInterface: public pr2_controller_interface::Controller
{
 private:
  ros::NodeHandle n_;
  pr2_mechanism_model::RobotState* robotState_;    
  ros::Time lastCycle_;
  double timeThreshold_;
  enum{
    DOF = 7
  };

  // The loop control variables:     
  struct loopControl{
    long long loopCount;
    double samplingPeroid;
    ros::Time leftLastCycle_;
    ros::Time rightLastCycle_;
    void init(const ros::Time & input);
    void setLastCycleRightArm(const ros::Time & input){
      rightLastCycle_ = input;
    }
    void setLastCycleLeftArm(const ros::Time & input){
      leftLastCycle_ = input;
    }
  } loopController_;
  
  struct armJoints{

    struct jointVelocities{
      std::vector<double> left;
      std::vector<double> right;
    }velocities;
      
    struct jointLimits{
      struct limits{
      std::vector<double> lower;
      std::vector<double> upper;
      std::vector<double> velocity;
      } left, right;
    } limits;

    void init(ros::NodeHandle n);
    void writeZeroVelocitiesLeftArm();
    void writeZeroVelocitiesRightArm();
    bool checkLeftArmLimits(const brics_actuator::JointVelocities &msg); 
    bool checkRightArmLimits(const brics_actuator::JointVelocities &msg); 

  }armJoints_;
  
  void leftArmCallback(const brics_actuator::JointVelocities &msg);
  void rightArmCallback(const brics_actuator::JointVelocities &msg);

  ros::Subscriber leftArmInputSubscriber_;
  ros::Subscriber rightArmInputSubscriber_;
  


  void jointVelocityControllerIni();
  
  controller::JointVelocityController rController0_;
  controller::JointVelocityController rController1_;
  controller::JointVelocityController rController2_;
  controller::JointVelocityController rController3_;
  controller::JointVelocityController rController4_;
  controller::JointVelocityController rController5_;
  controller::JointVelocityController rController6_;

  control_toolbox::Pid rPid0_; // the pid object
  control_toolbox::Pid rPid1_; // the pid object
  control_toolbox::Pid rPid2_; // the pid object
  control_toolbox::Pid rPid3_; // the pid object
  control_toolbox::Pid rPid4_; // the pid object
  control_toolbox::Pid rPid5_; // the pid object
  control_toolbox::Pid rPid6_; // the pid object

  controller::JointVelocityController lController0_;
  controller::JointVelocityController lController1_;
  controller::JointVelocityController lController2_;
  controller::JointVelocityController lController3_;
  controller::JointVelocityController lController4_;
  controller::JointVelocityController lController5_;
  controller::JointVelocityController lController6_;

  control_toolbox::Pid lPid0_; // the pid object
  control_toolbox::Pid lPid1_; // the pid object
  control_toolbox::Pid lPid2_; // the pid object
  control_toolbox::Pid lPid3_; // the pid object
  control_toolbox::Pid lPid4_; // the pid object
  control_toolbox::Pid lPid5_; // the pid object
  control_toolbox::Pid lPid6_; // the pid object

  void leftArmJointVelocitySet();
  void rightArmJointVelocitySet();
 public:
  bool init(pr2_mechanism_model::RobotState *robot, 
	    ros::NodeHandle &n);
  void starting();
  void update();
  void stopping();


};

# endif
