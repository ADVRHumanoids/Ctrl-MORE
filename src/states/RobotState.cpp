#include <iostream>

#include <locomotion/states/RobotState.hpp>

namespace Locomotion {

RobotState::RobotState() {

    Eigen::VectorXd temp(1);
    m_measuredData.emplace("temp",temp);
    m_measurementTimeStamp.emplace("temp",0);
    m_estimatedData.emplace("temp",temp);
    m_estimateTimeStamp.emplace("temp",0);
}

RobotState::~RobotState() {

}
RobotState::RobotState(RobotState& other){

}

Eigen::VectorXd RobotState::getMeasuredData(const std::string& name) const
{
    std::unordered_map<std::string, Eigen::VectorXd>::const_iterator measuredData = m_measuredData.find(name);
    
    if (measuredData == m_measuredData.end()) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[RobotState.getMeasureData] Element " <<name<<" doesn't exist" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

    }
    
    return measuredData->second;
}

Eigen::VectorXd RobotState::getMeasuredDataTimed(const std::string& name, unsigned int &timestamp) const
{
    std::unordered_map<std::string, Eigen::VectorXd>::const_iterator measuredData = m_measuredData.find(name);
    
    if (measuredData == m_measuredData.end()) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[RobotState.getMeasuredDataTimed] Element " <<name<<" doesn't exist" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
    }
    
    timestamp = m_measurementTimeStamp.at(name);
    
    return measuredData->second;
}

Eigen::VectorXd RobotState::getEstimatedData(const std::string& name) const
{
    std::unordered_map<std::string, Eigen::VectorXd>::const_iterator estimatedData = m_estimatedData.find(name);
    
    if (estimatedData == m_estimatedData.end()) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[RobotState.getEstimatedData] Element " <<name<<" doesn't exist" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
    }
    
    return estimatedData->second;    
}

Eigen::VectorXd RobotState::getEstimatedDataTimed(const std::string& name, unsigned int &timestamp) const
{
    std::unordered_map<std::string, Eigen::VectorXd>::const_iterator estimatedData = m_estimatedData.find(name);
    
    if (estimatedData == m_estimatedData.end()) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[RobotState.getEstimatedDataTimed] Element " <<name<<" doesn't exist" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
    }
    
    timestamp = m_estimateTimeStamp.at(name);
    
    return estimatedData->second;    
}

void RobotState::setMeasuredData(const std::string& name, const Eigen::VectorXd& data, const unsigned int timeStamp)
{

    if (m_measuredData.count(name)>0 && m_measurementTimeStamp.count(name)>0){
        m_measuredData.at(name) = data;
        m_measurementTimeStamp.at(name) = timeStamp;
    }
    else{
        m_measuredData.emplace(name,data);
        m_measurementTimeStamp.emplace(name,timeStamp);
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[RobotState.setMeasuredData] NEW KEY "<<name<<" ADDED TO THE ROBOT STATE" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

    }

}

void RobotState::setEstimatedData(const std::string& name, const Eigen::VectorXd& data, const unsigned int timeStamp)
{
    if (m_estimatedData.count(name)>0 && m_estimateTimeStamp.count(name)>0){
        m_estimatedData.at(name) = data;
        m_estimateTimeStamp.at(name) = timeStamp;
    }
    else{
        m_estimatedData.emplace(name,data);
        m_estimateTimeStamp.emplace(name,timeStamp);
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[RobotState.setEstimatedData]NEW KEY "<<name<<" ADDED TO THE CONTROLSTATES" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

    }


}

void RobotState::Print() const{

    std::cout <<"m_measuredData"<< std::endl;
     for ( std::unordered_map<std::string, Eigen::VectorXd >::const_iterator iter = m_measuredData.begin();
           iter != m_measuredData.end(); ++iter )
           std::cout << iter->first << '\t' << iter->second.transpose() << '\n';
     std::cout << std::endl;

     std::cout <<"m_estimatedData"<< std::endl;
      for ( std::unordered_map<std::string, Eigen::VectorXd >::const_iterator iter = m_estimatedData.begin();
            iter != m_estimatedData.end(); ++iter )
            std::cout << iter->first << '\t' << iter->second.transpose() << '\n';
      std::cout << std::endl;

      std::cout <<"m_measurementTimeStamp"<< std::endl;
       for ( std::unordered_map<std::string, unsigned int>::const_iterator iter = m_measurementTimeStamp.begin();
             iter != m_measurementTimeStamp.end(); ++iter )
             std::cout << iter->first << '\t' << iter->second << '\n';
       std::cout << std::endl;


       std::cout <<"m_estimateTimeStamp"<< std::endl;
        for ( std::unordered_map<std::string, unsigned int>::const_iterator iter = m_estimateTimeStamp.begin();
              iter != m_estimateTimeStamp.end(); ++iter )
              std::cout << iter->first << '\t' << iter->second << '\n';
        std::cout << std::endl;


}

}
