# include <pr2JointVelocityControllerInterface/pr2JointVelocityControllerInterface.h>
void pr2JointVelocityControllerInterface::loopControl::init(const ros::Time & input){

  loopCount = 0;
  samplingPeroid = 1/(1000.0);

  setLastCycleRightArm( input );
  setLastCycleLeftArm( input );
}

bool pr2JointVelocityControllerInterface::armJoints::checkLeftArmLimits(
							     const brics_actuator::JointVelocities &msg 
							     ){

  if(msg.velocities.size()==DOF){
    for(int i = 0; i<DOF; i++){
      if(msg.velocities[i].value>limits.left.velocity[i]
				 &&msg.velocities[i].value<-limits.left.velocity[i]){
	return false;
      }
    }
    
  }else{
    return false;
  }

  return true;

}
bool pr2JointVelocityControllerInterface::armJoints::checkRightArmLimits(
							     const brics_actuator::JointVelocities &msg 
							     ){

  if(msg.velocities.size()==DOF){
    for(int i = 0; i<DOF; i++){
      if(msg.velocities[i].value>limits.right.velocity[i]
				 &&msg.velocities[i].value<-limits.right.velocity[i]){
	return false;
      }
    }
    
  }else{
    return false;
  }

  return true;

}


void  pr2JointVelocityControllerInterface::armJoints::writeZeroVelocitiesLeftArm(){

  for(unsigned int i = 0; i < DOF; i++){
    velocities.left[i] = 0.0;
  }

}
void  pr2JointVelocityControllerInterface::armJoints::writeZeroVelocitiesRightArm(){

  for(unsigned int i = 0; i < DOF; i++){
    velocities.right[i] = 0.0;
  }

}
void pr2JointVelocityControllerInterface::armJoints::init(ros::NodeHandle n){
  velocities.left.resize(DOF);
  velocities.left.assign(DOF, 0);
  velocities.right.resize(DOF);
  velocities.right.assign(DOF, 0);
	
  limits.left.upper.resize(DOF);
  limits.left.upper.assign(DOF, 0);
  limits.left.lower.resize(DOF);
  limits.left.lower.assign(DOF, 0);

  limits.left.velocity.resize(DOF);
  limits.left.velocity.assign(DOF, 0);

  limits.right.upper.resize(DOF);
  limits.right.upper.assign(DOF, 0);
  limits.right.lower.resize(DOF);
  limits.right.lower.assign(DOF, 0);

  limits.right.velocity.resize(DOF);
  limits.right.velocity.assign(DOF, 0);

  XmlRpc::XmlRpcValue jointLowerLeftXml, jointUpperLeftXml, 
    jointLowerRightXml, jointUpperRightXml, 
    jointVLimitsXml;

  if (n.hasParam("jointLowerLeft")
      && n.hasParam("jointUpperLeft") 
      && n.hasParam("jointLowerRight") 
      && n.hasParam("jointUpperRight") 
      && n.hasParam("jointVLimits") ) {
    n.getParam("jointLowerLeft", jointLowerLeftXml);
    n.getParam("jointUpperLeft", jointUpperLeftXml);
    n.getParam("jointLowerRight", jointLowerRightXml);
    n.getParam("jointUpperRight", jointUpperRightXml);
    n.getParam("jointVLimits", jointVLimitsXml);
  }else
    throw "Joint limits parameters are not set, shutting down node...";

  for (int i = 0; i < DOF; i++){
    // left arm 
    limits.left.lower[i] = static_cast<double>(jointLowerLeftXml[i]);
    limits.left.upper[i] = static_cast<double>(jointUpperLeftXml[i]);
    limits.left.velocity[i] = static_cast<double>(jointVLimitsXml[i]);
    // right arm 
    limits.right.lower[i] = static_cast<double>(jointLowerRightXml[i]);
    limits.right.upper[i] = static_cast<double>(jointUpperRightXml[i]);
    limits.right.velocity[i] = static_cast<double>(jointVLimitsXml[i]);
  }

  writeZeroVelocitiesLeftArm();
  writeZeroVelocitiesRightArm();

}// end of ini 


