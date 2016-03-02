# include <pr2JointVelocityControllerInterface/pr2JointVelocityControllerInterface.h>
# include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( pr2JointVelocityControllerInterface,
			pr2_controller_interface::Controller)

bool pr2JointVelocityControllerInterface::init(pr2_mechanism_model::RobotState *robot, 
				     ros::NodeHandle &n){
  try{
    n_ = n;
    if(!robot) 
      return false;
    else
      robotState_ = robot;

  if (
      n.hasParam("timeThreshold")
      ) {
    n.getParam("timeThreshold", timeThreshold_);
  }else
    throw "time limit for the joint velocity is not set...";


    leftArmInputSubscriber_ = n_.subscribe("/pr2LeftArmJointVelocityControllerInterface", 10, &pr2JointVelocityControllerInterface::leftArmCallback, this);
    rightArmInputSubscriber_ = n_.subscribe("/pr2RightArmJointVelocityControllerInterface", 10, &pr2JointVelocityControllerInterface::rightArmCallback, this);    



    

    loopController_.init(robotState_->getTime());
    ROS_INFO("loopController_ has been initialized.\n");

    armJoints_.init(n_);
    ROS_INFO("armJoints_ has been initialized.\n");

    
    jointVelocityControllerIni();
    ROS_INFO("Initialized joint velocity controllers.\n");


  }catch(const char* msg){
    ROS_ERROR("%s, ", msg);
    return false;
  }catch (...) {
    ROS_INFO("Error! Unknown exception caught when updating.");
    return false;
  }
  return true;
}

void  pr2JointVelocityControllerInterface::rightArmCallback(const brics_actuator::JointVelocities &msg){
  std::cout<<"right arm callback "<<std::endl;
  if(!((ros::Time::now() - msg.velocities[0].timeStamp).toSec() > timeThreshold_ )){
    if(!armJoints_.checkRightArmLimits(msg)){

      ROS_ERROR("Received PROBLEMATIC right arm joint velocities");

    }else{
  
      // search for joints in joint state msg
      for(unsigned int i=0; i<DOF; i++){
	armJoints_.velocities.right[i] = msg.velocities[i].value; 
      }
    
      rightArmJointVelocitySet();
      loopController_.setLastCycleRightArm( msg.velocities[0].timeStamp );
	    
    }
  }else{
    ROS_ERROR("The received joint velocity is too old.");
  }

}
void  pr2JointVelocityControllerInterface::leftArmCallback(const brics_actuator::JointVelocities &msg){
  std::cout<<"left arm callback "<<std::endl;
  
  if(!((ros::Time::now() - msg.velocities[0].timeStamp).toSec() > timeThreshold_ )){
    if(!armJoints_.checkLeftArmLimits(msg)){

      ROS_ERROR("Received PROBLEMATIC left arm joint velocities");

    }else{
    
      // search for joints in joint state msg
      for(unsigned int i=0; i<DOF; i++){
	armJoints_.velocities.left[i] = msg.velocities[i].value; 
      }
      leftArmJointVelocitySet();
      loopController_.setLastCycleLeftArm( msg.velocities[0].timeStamp );
    }
  }else{
    ROS_ERROR("The received joint velocity is too old.");
  }
}

void pr2JointVelocityControllerInterface::starting(){

  ROS_INFO("pr2JointVelocityControllerInterface::starting() started.");
    rController0_.starting();
    ROS_INFO("Started right arm 0 joint controller  .\n");
    rController1_.starting();
    ROS_INFO("Started right arm 1 joint controller  .\n");
    rController2_.starting();
    ROS_INFO("Started right arm 2 joint controller  .\n");
    rController3_.starting();
    ROS_INFO("Started right arm 3 joint controller  .\n");
    rController4_.starting();
    ROS_INFO("Started right arm 4 joint controller  .\n");
    rController5_.starting();
    ROS_INFO("Started right arm 5 joint controller  .\n");
    rController6_.starting();
    ROS_INFO("Started right arm 6 joint controller  .\n");

    lController0_.starting();
    ROS_INFO("Started left arm 0 joint controller  .\n");
    lController1_.starting();
    ROS_INFO("Started left arm 1 joint controller  .\n");
    lController2_.starting();
    ROS_INFO("Started left arm 2 joint controller  .\n");
    lController3_.starting();
    ROS_INFO("Started left arm 3 joint controller  .\n");
    lController4_.starting();
    ROS_INFO("Started left arm 4 joint controller  .\n");
    lController5_.starting();
    ROS_INFO("Started left arm 5 joint controller  .\n");
    lController6_.starting();
    ROS_INFO("Started left arm 6 joint controller  .\n");

    loopController_.setLastCycleLeftArm( robotState_->getTime() );
    loopController_.setLastCycleRightArm( robotState_->getTime() );

  ROS_INFO("pr2JointVelocityControllerInterface::starting() finished.");

  // Write zero velocities in the beginning  
  armJoints_.writeZeroVelocitiesLeftArm();
  armJoints_.writeZeroVelocitiesRightArm();
  leftArmJointVelocitySet();
  rightArmJointVelocitySet();

}
void  pr2JointVelocityControllerInterface::jointVelocityControllerIni(){
    if(!lPid0_.init(ros::NodeHandle(n_,"lPid0")))
      throw "lPid0 cannot be initialized";
    if(!lPid1_.init(ros::NodeHandle(n_,"lPid1")))
      throw "lPid1 cannot be initialized";
    if(!lPid2_.init(ros::NodeHandle(n_,"lPid2")))
      throw "lPid2 cannot be initialized";
    if(!lPid3_.init(ros::NodeHandle(n_,"lPid3")))
      throw "lPid3 cannot be initialized";
    if(!lPid4_.init(ros::NodeHandle(n_,"lPid4")))
      throw "lPid4 cannot be initialized";
    if(!lPid5_.init(ros::NodeHandle(n_,"lPid5")))
      throw "lPid5 cannot be initialized";
    if(!lPid6_.init(ros::NodeHandle(n_,"lPid6")))
      throw "lPid6 cannot be initialized";

    
    if(!lController0_.init(robotState_, "l_shoulder_pan_joint", lPid0_))
      throw "lController0_ cannot be initialized";
    if(!lController1_.init(robotState_, "l_shoulder_lift_joint", lPid1_))
      throw "lController1_ cannot be initialized";
    if(!lController2_.init(robotState_, "l_upper_arm_roll_joint", lPid2_))
      throw "lController2_ cannot be initialized";
    if(!lController3_.init(robotState_, "l_elbow_flex_joint", lPid3_))
      throw "lController3_ cannot be initialized";
    if(!lController4_.init(robotState_, "l_forearm_roll_joint", lPid4_))
      throw "lController4_ cannot be initialized";
    if(!lController5_.init(robotState_, "l_wrist_flex_joint", lPid5_))
      throw "lController5_ cannot be initialized";
    if(!lController6_.init(robotState_, "l_wrist_roll_joint", lPid6_))
      throw "lController6_ cannot be initialized";

    if(!rPid0_.init(ros::NodeHandle(n_,"rPid0")))
      throw "rPid0 cannot be initialized";
    if(!rPid1_.init(ros::NodeHandle(n_,"rPid1")))
      throw "rPid1 cannot be initialized";
    if(!rPid2_.init(ros::NodeHandle(n_,"rPid2")))
      throw "rPid2 cannot be initialized";
    if(!rPid3_.init(ros::NodeHandle(n_,"rPid3")))
      throw "rPid3 cannot be initialized";
    if(!rPid4_.init(ros::NodeHandle(n_,"rPid4")))
      throw "rPid4 cannot be initialized";
    if(!rPid5_.init(ros::NodeHandle(n_,"rPid5")))
      throw "rPid5 cannot be initialized";
    if(!rPid6_.init(ros::NodeHandle(n_,"rPid6")))
      throw "rPid6 cannot be initialized";


    if(!rController0_.init(robotState_, "r_shoulder_pan_joint", rPid0_))
      throw "rController0_ cannot be initialized";
    if(!rController1_.init(robotState_, "r_shoulder_lift_joint", rPid1_))
      throw "rController1_ cannot be initialized";
    if(!rController2_.init(robotState_, "r_upper_arm_roll_joint", rPid2_))
      throw "rController2_ cannot be initialized";
    if(!rController3_.init(robotState_, "r_elbow_flex_joint", rPid3_))
      throw "rController3_ cannot be initialized";
    if(!rController4_.init(robotState_, "r_forearm_roll_joint", rPid4_))
      throw "rController4_ cannot be initialized";
    if(!rController5_.init(robotState_, "r_wrist_flex_joint", rPid5_))
      throw "rController5_ cannot be initialized";
    if(!rController6_.init(robotState_, "r_wrist_roll_joint", rPid6_))
      throw "rController6_ cannot be initialized";
}
void pr2JointVelocityControllerInterface::leftArmJointVelocitySet(){
    lController0_.setCommand(armJoints_.velocities.left[0]);
    lController1_.setCommand(armJoints_.velocities.left[1]);
    lController2_.setCommand(armJoints_.velocities.left[2]);
    lController3_.setCommand(armJoints_.velocities.left[3]);
    lController4_.setCommand(armJoints_.velocities.left[4]);
    lController5_.setCommand(armJoints_.velocities.left[5]);
    lController6_.setCommand(armJoints_.velocities.left[6]);

    lController0_.update();
    lController1_.update();
    lController2_.update();
    lController3_.update();
    lController4_.update();
    lController5_.update();
    lController6_.update();
}
void pr2JointVelocityControllerInterface::rightArmJointVelocitySet(){

  rController0_.setCommand(armJoints_.velocities.right[0]);
  rController1_.setCommand(armJoints_.velocities.right[1]);
  rController2_.setCommand(armJoints_.velocities.right[2]);
  rController3_.setCommand(armJoints_.velocities.right[3]);
  rController4_.setCommand(armJoints_.velocities.right[4]);
  rController5_.setCommand(armJoints_.velocities.right[5]);
  rController6_.setCommand(armJoints_.velocities.right[6]);

  rController0_.update();
  rController1_.update();
  rController2_.update();
  rController3_.update();
  rController4_.update();
  rController5_.update();
  rController6_.update();
}
void pr2JointVelocityControllerInterface::update(){
  try{
    if(!robotState_)
      throw "robotState_ is NULL. ";


    // update the input command 

    // safety check
    
    // set the joint velocity 
    if((ros::Time::now() -  loopController_.leftLastCycle_).toSec() > timeThreshold_){
        armJoints_.writeZeroVelocitiesLeftArm();
    }
    if(
       (ros::Time::now() - loopController_.rightLastCycle_ ).toSec() > timeThreshold_
       ){
      armJoints_.writeZeroVelocitiesRightArm();
    }
    leftArmJointVelocitySet();
    rightArmJointVelocitySet();

    loopController_.loopCount++;

    
  }catch(const char* msg){
    ROS_INFO("%s, ", msg);
  }catch( std::exception& e ){  
    ROS_INFO("Standard exception: %s . ", e.what());
  }catch (...) {
    ROS_INFO("Error! Unknown exception caught when updating.");
  }

}
void pr2JointVelocityControllerInterface::stopping(){
  ROS_INFO("pr2JointVelocityControllerInterface::stopping() started.");

  ROS_INFO("pr2JointVelocityControllerInterface is finished .");
}

