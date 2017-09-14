#include <locomotion/Locomotor.hpp>

namespace Locomotion
{


Locomotor::Locomotor(){
    this->m_feedbackControllers = std::shared_ptr<FeedbackControllersManager> (new Locomotion::FeedbackControllersManager());

}
Locomotor::Locomotor(const std::string configurationFileName){
    std::string fdbkConfFiles = configurationFileName;
    this->m_feedbackControllers = std::shared_ptr<FeedbackControllersManager> (new Locomotion::FeedbackControllersManager(fdbkConfFiles));

    
}


Locomotor::~Locomotor(){

}



void Locomotor::command(const std::string locomotionControllerName, std::string commandName){
    std::cout<<"not available yet"<<std::endl;
}

void Locomotor::update(const RobotState& robotState, ControlState *controlState){
    m_finalControlState=*controlState;
    m_locomotionControlState=m_finalControlState;
    m_feedbackControllers->update(robotState,*controlState , &m_finalControlState);
    *controlState = m_finalControlState;

}

Eigen::VectorXd Locomotor::getRefCom(){
    return m_finalControlState.getCOM();
}

Eigen::VectorXd Locomotor::getRefZmp(){
    return m_finalControlState.getZMP();
}

Eigen::VectorXd Locomotor::getRefLinearMomentum(){
    return m_finalControlState.getLinearMomentum();
}

Eigen::VectorXd Locomotor::getRefAngularMomentum(){
   return m_finalControlState.getAngularMomentum();
}

Eigen::VectorXd Locomotor::getRefTask(std::string nameOfReference){
   return m_finalControlState.getReferencePose(nameOfReference);
}

Eigen::VectorXd Locomotor::getRefJoint(std::string nameOfReference){
    return m_finalControlState.getReferenceJoint(nameOfReference);
}


}


