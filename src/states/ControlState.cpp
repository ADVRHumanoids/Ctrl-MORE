#include <iostream>

#include <locomotion/states/ControlState.hpp>


namespace Locomotion {

ControlState::ControlState() {    
    Eigen::VectorXd temp(1);
    m_taskSpaceReferencePose.emplace("default",temp);
    m_taskSpaceReferenceVel.emplace("default",temp);
    m_taskSpaceReferenceAcc.emplace("default",temp);
    m_jointSpaceReference.emplace("default",temp);
    m_taskSpaceReferenceZMP.emplace("default",temp);
    m_Otherreference.emplace("default",temp);
    m_com<<0,0,0;
    m_linearMomentum<<0,0,0;

    /// \brief angular Momentum.
    m_angularMomentum<<0,0,0;

    /// \brief ZMP Position.
    m_zmp<<0,0,0;

    /// \brief control step time.
    m_dt=0.005;

    m_ctrlMode=task;
    ReturnPose.resize(7);
    ReturnVel.resize(6);
    ReturnAcc.resize(6);

}


ControlState::~ControlState() {
}
ControlState::ControlState(ControlState& other){

}
bool ControlState::referenceExists(const std::string& name) const
{
    if (m_taskSpaceReferencePose.find(name) == m_taskSpaceReferencePose.end()) {
        return false;
    }
    
    return true;
}

Eigen::VectorXd ControlState::getReferencePose(const std::string& name)  {
    this->ReturnPose= Eigen::VectorXd::Zero(7);
    /// \note In the previous version it call getReferenceVel. was this and error?
    getReferencePose(name, &this->ReturnPose);
    return ReturnPose;
}

bool ControlState::getReferencePose(const std::string& name, Eigen::VectorXd *poseVector) const {
    if (poseVector->rows() != 7) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[CtrlState.getReferencePose] Wrong size of argument vector used when querying for " <<name<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
        return false;
    }
    std::unordered_map<std::string, Eigen::VectorXd>::const_iterator pose = m_taskSpaceReferencePose.find(name);
    if (pose == m_taskSpaceReferencePose.end()) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[CtrlState.getReferencePose] Element " <<name<<" doesn't exist" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
    }
    else{
        *poseVector = m_taskSpaceReferencePose.at(name);
    }    
    return true;
}

Eigen::VectorXd ControlState::getReferenceVel(const std::string& name) {
    this->ReturnVel = Eigen::VectorXd::Zero(6);
    getReferenceVel(name, &this->ReturnVel);
    return this->ReturnVel;
}

bool ControlState::getReferenceVel(const std::string& name, Eigen::VectorXd *velocity) const {
    if (velocity->rows() != 6) { 
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[CtrlState.getReferenceVel] Wrong size of argument vector used when querying for " <<name<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
        return false; 
    }
    std::unordered_map<std::string, Eigen::VectorXd>::const_iterator vel = m_taskSpaceReferenceVel.find(name);
    if (vel == m_taskSpaceReferenceVel.end()) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[CtrlState.getReferenceVel] Element " <<name<<" doesn't exist" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
    }
    else{
        *velocity = m_taskSpaceReferenceVel.at(name);
    }    
    
    return true;
}


Eigen::VectorXd ControlState::getReferenceAcc(const std::string& name) {
    this->ReturnAcc = Eigen::VectorXd::Zero(6);
    getReferenceAcc(name, &(this->ReturnAcc));
    return this->ReturnAcc;
}

bool ControlState::getReferenceAcc(const std::string& name, Eigen::VectorXd *acceleration) const {
    if (acceleration->rows() != 6) { 
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[CtrlState.getReferenceAcc] Wrong size of argument vector used when querying for " <<name<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
        return false; 
    }
    std::unordered_map<std::string, Eigen::VectorXd>::const_iterator acc = m_taskSpaceReferenceAcc.find(name);
    if (acc == m_taskSpaceReferenceAcc.end()) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[CtrlState.getReferenceAcc] Element " <<name<<" doesn't exist" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
    }
    else{
        *acceleration = m_taskSpaceReferenceAcc.at(name);
    }    
    
    return true;
}

bool ControlState::getOtherReference(const std::string& name,Eigen::VectorXd *Info) const {

    std::unordered_map<std::string, Eigen::VectorXd>::const_iterator reference = m_Otherreference.find(name);
    if (reference == m_Otherreference.end()) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[CtrlState.getOtherReference] Element " <<name<<" doesn't exist" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
        *Info = Eigen::VectorXd::Zero(3);
        return false;
    }
    else{
        *Info = m_Otherreference.at(name);
    }

    return true;
}

/// \note Returning (0,0,0) by default
/// \note if the returning vector is Xd with not know lenght, we face real tiem problems
/// \todo rt safe???
Eigen::VectorXd ControlState::getReferenceJoint(const std::string& typeOfReference) const {
    Eigen::VectorXd temp;
    std::unordered_map<std::string, Eigen::VectorXd>::const_iterator ref = m_jointSpaceReference.find(typeOfReference);
    
    if (ref == m_jointSpaceReference.end()) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[CtrlState.getReferenceJoint] Element " <<typeOfReference<<" doesn't exist" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
    }
    else {
        temp.resize(m_jointSpaceReference.at(typeOfReference).size());
        temp = m_jointSpaceReference.at(typeOfReference);
    }

    
    return temp;
}


double ControlState::getControlStepTime() const {
    return m_dt;
}


ControlState::controlMode ControlState::getControlMode() const {
    return m_ctrlMode;
}

Eigen::Matrix3d ControlState::getPelvisOrientation(){
    return m_PelvisOrientation;
}

bool ControlState::setReferencePose(const std::string& name, const Eigen::VectorXd& data) {

    if (data.rows() != 7) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "CtrlState.setRefPose "<<name<<" Tried to set reference of wrong format. See documentation for format information." << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
        return false;
    }
        
    if (m_taskSpaceReferencePose.count(name)>0){
        m_taskSpaceReferencePose.at(name) = data;
    }
    else{
         m_taskSpaceReferencePose.emplace(name,data);
         XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "NEW KEY "<<name<<" ADDED TO THE CONTROLSTATES" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

    }
    
    return true;
}

bool ControlState::setReferenceVel(const std::string& name, const Eigen::VectorXd& data) {
    if (data.rows() != 6) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "CtrlState.setRefVel "<<name<<" Tried to set reference of wrong format. See documentation for format information." << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
        return false;
    }

    if (m_taskSpaceReferenceVel.count(name)>0){
        m_taskSpaceReferenceVel.at(name) = data;
    }
    else{
         m_taskSpaceReferenceVel.emplace(name,data);
         XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[ControlState.steReferenceVel] NEW KEY "<<name<<" ADDED TO THE CONTROLSTATES" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

    }
    
    return true;
}

bool ControlState::setReferenceAcc(const std::string& name, const Eigen::VectorXd& data) {
    if (data.rows() != 7) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "CtrlState.setRefAcc "<<name<<" Tried to set reference of wrong format. See documentation for format information." << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
        return false;
    }
    
    if (m_taskSpaceReferenceAcc.count(name)>0){
        m_taskSpaceReferenceAcc.at(name) = data;
    }
    else{
         m_taskSpaceReferenceAcc.emplace(name,data);
         XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[ControlState.setReferenceAcc] NEW KEY "<<name<<" ADDED TO THE CONTROLSTATES" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

    }
    return true;
}

void ControlState::setReferenceZMP(const std::string& name, const Eigen::VectorXd& data) {

    if (m_taskSpaceReferenceZMP.count(name)>0){
        m_taskSpaceReferenceZMP.at(name) = data;
    }
    else{
         m_taskSpaceReferenceZMP.emplace(name,data);
         XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[ControlState.setReferenceZMP] NEW KEY "<<name<<" ADDED TO THE CONTROLSTATES" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

    }

}

void ControlState::setReferenceJoint(const std::string& typeOfReference, const Eigen::VectorXd& data) {

    if (m_jointSpaceReference.count(typeOfReference)>0){
        m_jointSpaceReference.at(typeOfReference) = data;
    }
    else{
         m_jointSpaceReference.emplace(typeOfReference,data);
         XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[ControlState.setReferenceJoint] NEW KEY "<<typeOfReference<<" ADDED TO THE CONTROLSTATES" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

    }

}

void ControlState::setPelvisOrientation(const Eigen::Vector3d& ref) {
    m_PelvisOrientation=Eigen::Matrix3d::Identity();
    m_PelvisOrientation=m_PelvisOrientation*rotz(ref(2));
    m_PelvisOrientation=m_PelvisOrientation*roty(ref(1));
    m_PelvisOrientation=m_PelvisOrientation*rotx(ref(0));
}

bool ControlState::setOtherReference(const std::string& name, const Eigen::VectorXd& data) {

    if (m_Otherreference.count(name)>0){
        m_Otherreference.at(name) = data;
    }
    else{
         m_Otherreference.emplace(name,data);
         XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "NEW KEY "<<name<<" ADDED TO THE CONTROLSTATES" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
    }

    return true;
}
void ControlState::setControlMode(const controlMode newCtrlMode) {
    
    m_ctrlMode = newCtrlMode;
}

void ControlState::Print() const{

    std::cout <<"m_taskSpaceReferencePose"<< std::endl;
    for ( std::unordered_map<std::string, Eigen::VectorXd >::const_iterator iter = m_taskSpaceReferencePose.begin();
          iter != m_taskSpaceReferencePose.end(); ++iter )
          std::cout << iter->first << '\t' << iter->second.transpose() << '\n';
    std::cout << std::endl;


    std::cout <<"m_taskSpaceReferenceVel"<< std::endl;
    for ( std::unordered_map<std::string, Eigen::VectorXd >::const_iterator iter = m_taskSpaceReferenceVel.begin();
          iter != m_taskSpaceReferenceVel.end(); ++iter )
          std::cout << iter->first << '\t' << iter->second.transpose() << '\n';
    std::cout << std::endl;

    std::cout <<"m_taskSpaceReferenceAcc"<< std::endl;
    for ( std::unordered_map<std::string, Eigen::VectorXd >::const_iterator iter = m_taskSpaceReferenceAcc.begin();
          iter != m_taskSpaceReferenceAcc.end(); ++iter )
          std::cout << iter->first << '\t' << iter->second.transpose() << '\n';
    std::cout << std::endl;



    std::cout <<"m_jointSpaceReference"<< std::endl;
    for ( std::unordered_map<std::string, Eigen::VectorXd >::const_iterator iter = m_jointSpaceReference.begin();
          iter != m_jointSpaceReference.end(); ++iter )
          std::cout << iter->first << '\t' << iter->second.transpose() << '\n';
    std::cout << std::endl;



    std::cout <<"m_taskSpaceReferenceZMP"<< std::endl;
    for ( std::unordered_map<std::string, Eigen::VectorXd >::const_iterator iter = m_taskSpaceReferenceZMP.begin();
          iter != m_taskSpaceReferenceZMP.end(); ++iter )
          std::cout << iter->first << '\t' << iter->second.transpose() << '\n';
    std::cout << std::endl;

}

}
