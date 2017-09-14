#include <iostream>
#include <dlfcn.h>

//#include <rbdl/rbdl_math.h>
#include "yaml-cpp/yaml.h"

#include <locomotion/controllers/feedback/FeedbackControllersManager.hpp>


namespace Locomotion {

FeedbackControllersManager::FeedbackControllersManager()
{
    XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt") = XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt");

}
FeedbackControllersManager::FeedbackControllersManager(const std::string configurationFileName)
{
    //1. Load controllers 
    //2. Initialize 
    XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt") = XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt");

    YAML::Node config = YAML::LoadFile(configurationFileName.c_str());

    //Check if controllers list exists
    if (config["loadControllers"])  {
        for (auto controller: config["loadControllers"]) {
            std::string libPath;
            computeAbsolutePath(config[controller.as<std::string>()]["libPath"].as<std::string>(),"/",libPath);
            std::string configFile;
            computeAbsolutePath(config[controller.as<std::string>()]["configFile"].as<std::string>(),"/",configFile);

            if (config[controller.as<std::string>()]) {
                if (!loadSingleController(  controller.as<std::string>(), 
                                            libPath,
                                            configFile)) {
                    XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[FbckCtrlMan.Construtor] " << "Failed loading controller: " << controller.as<std::string>() << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

                }
            }
            else {
                XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[FbckCtrlMan.Construtor] " << "Check configuration file. Details of " << controller.as<std::string>() << " are not present! " << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
            }
        }
    }
    else {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[FbckCtrlMan.Construtor] " << "Controllers list is not present" << XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

    }

}

FeedbackControllersManager::~FeedbackControllersManager()
{
  //Close all libraries - this will cause segfault on the exit from desctructor, as dlclose will destroy objects of librararies, before destroying references to them from the shared pointers. 
//   for (auto controllerHandle : m_controllerLibHandles) {
//     dlclose(controllerHandle);
//   }
//   
//   m_controllerLibHandles.clear();   
}

void FeedbackControllersManager::update(const RobotState& robotState, const ControlState &originalControlState, ControlState *modifiedControlState)
{
    //Return if there are no feedback controllers to execute
    if (m_executionOrder.size() < 1) return;
  
    for (auto controllerIndex: m_executionOrder) {
        if (m_feedbackControllers.at(controllerIndex)->isEnabled()) {

            m_feedbackControllers.at(controllerIndex)->update(robotState, originalControlState, modifiedControlState);
        }
    }


}

void FeedbackControllersManager::addController(const std::shared_ptr<FeedbackController> &newFeedbackController)
{
    m_feedbackControllers.push_back(newFeedbackController);
    m_controllersOrder.insert(std::make_pair<std::string, unsigned int> (newFeedbackController->getName(), m_feedbackControllers.size() - 1));
    m_executionOrder.push_back( m_controllersOrder.at(newFeedbackController->getName()) );

#ifdef VERBOSE
    std::cout << "Added controller: " << newFeedbackController->getName() << std::endl;
#endif
}


void FeedbackControllersManager::enable(const std::string &controllerName)
{
    if (m_controllersOrder.count(controllerName)>0){
        m_feedbackControllers.at( m_controllersOrder.at(controllerName) )->enable();
    }
    else{
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[FbckCtrlMan.enable] " <<"The Controller named: "<<controllerName<<" Doesn't Exist"<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

    }

}


void FeedbackControllersManager::disable(const std::string &controllerName)
{
    if (m_controllersOrder.count(controllerName)>0){
        m_feedbackControllers.at( m_controllersOrder.at(controllerName) )->disable();
    }
    else{
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[FbckCtrlMan.disable] " <<"The Controller named: "<<controllerName<<" Doesn't Exist"<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

    }
}


bool FeedbackControllersManager::isEnabled(const std::string &controllerName)
{
    if (m_controllersOrder.count(controllerName)>0){
        return m_feedbackControllers.at( m_controllersOrder.at(controllerName) )->isEnabled();
    }
    else{
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[FbckCtrlMan.isEenable] " <<"The Controller named: "<<controllerName<<" Doesn't Exist"<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

        return false;
    }
}


void FeedbackControllersManager::pauseAllUsingRobotState(const std::string &robotStateName)
{
    for (auto controller: m_feedbackControllers) {
        if (controller->isUsingRobotState(robotStateName)) {
            controller->pause();
        }
    }
}


void FeedbackControllersManager::pauseAllUsingControlState(const std::string &controlStateName)
{
    for (auto controller: m_feedbackControllers) {
        if (controller->isUsingControlState(controlStateName)) {
            controller->pause();
        }
    }    
}


void FeedbackControllersManager::resume(const std::string &controllerName)
{
    if (m_controllersOrder.count(controllerName)>0){
        m_feedbackControllers.at( m_controllersOrder.at(controllerName) )->resume();
    }
    else{
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[FbckCtrlMan.resume] " <<"The Controller named: "<<controllerName<<" Doesn't Exist"<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();


    }

}


void FeedbackControllersManager::resumeAllUsingRobotState(const std::string &robotStateName)
{
    for (auto controller: m_feedbackControllers) {
        if (controller->isUsingRobotState(robotStateName)) {
            controller->resume();
        }
    }    
}


void FeedbackControllersManager::resumeAllUsingControlState(const std::string &controlStateName)
{
    for (auto controller: m_feedbackControllers) {
        if (controller->isUsingControlState(controlStateName)) {
            controller->resume();
        }
    }
}


bool FeedbackControllersManager::isPaused(const std::string &controllerName)
{
    if (m_controllersOrder.count(controllerName)>0){
        return m_feedbackControllers.at( m_controllersOrder.at(controllerName) )->isPaused();
    }
    else{
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[FbckCtrlMan.isPaused] " <<"The Controller named: "<<controllerName<<" Doesn't Exist"<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

        return false;
    }
}


void FeedbackControllersManager::plot(std::string controllerName) const
{
    if (doesControllerExist(controllerName)) {
        m_feedbackControllers.at(m_controllersOrder.at(controllerName))->plot();
    } else {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[FbckCtrlMan.plot] " <<"The Controller named: "<<controllerName<<" Doesn't Exist"<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

    }   
}


void FeedbackControllersManager::plotAll() const
{
    if (!m_feedbackControllers.empty()) {
        for (auto controller : m_feedbackControllers) {
            controller->plot();
        }
    }     
}


void FeedbackControllersManager::setExecutionOrder(const std::vector<std::string> &executionOrder)
{
    //Make sure the vector is not empty 
    if (executionOrder.empty()) {
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[FbckCtrlMan.setExecutionOrder] " <<"Vector passed to the function is empty."<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

    }
    
    //Check if all controllers in the list exist 
    for (auto ctrlName: executionOrder) {
        if (!doesControllerExist(ctrlName)) {
            XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[FbckCtrlMan.setExecutionOrder] " <<"The Controller named: "<<ctrlName<<" Doesn't Exist"<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();

        }
    }
    
    m_executionOrder.resize(executionOrder.size());
    
    for (unsigned int i = 0; i < executionOrder.size(); i++) {
        m_executionOrder.at(i) = m_controllersOrder.at(executionOrder.at(i));
    }    
}


void FeedbackControllersManager::getControllersNames(std::vector<std::string> *controllersNames) const 
{
    controllersNames->resize(m_feedbackControllers.size());
    
    for (unsigned int i = 0; i < m_feedbackControllers.size(); i++) {
        controllersNames->at(i) = m_feedbackControllers.at(i)->getName();
    }    
}


std::vector<std::string> FeedbackControllersManager::getUsedControlStates() const 
{
    std::vector<std::string> controlStatesNames, tmpNames;
    
    if (m_feedbackControllers.empty()){
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[FbckCtrlMan.getUsedControlStates] " <<"No controllers are load"<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
        return controlStatesNames;
    }
    controlStatesNames = m_feedbackControllers.at(0)->getUsedControlStates();
    
    for (unsigned int i = 1; i<m_feedbackControllers.size(); i++) {
        tmpNames = m_feedbackControllers.at(i)->getUsedControlStates();
        for (auto newName: tmpNames) {
            if (!isNamePresent(controlStatesNames, newName)) {
                controlStatesNames.push_back(newName);
            }
        }
    }
    
    return controlStatesNames;
}


std::vector<std::string> FeedbackControllersManager::getUsedControlStates(const std::string &controllerName) const 
{
    if (doesControllerExist(controllerName)) {
        return m_feedbackControllers.at( m_controllersOrder.at(controllerName) )->getUsedControlStates();
    }
    XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[FbckCtrlMan.getUsedControlStates] " <<"The Controller named: "<<controllerName<<" Doesn't Exist"<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
    std::vector<std::string> NoData;
    return NoData;
}


std::vector<std::string> FeedbackControllersManager::getUsedRobotStates() const 
{
    std::vector<std::string> robotStatesNames, tmpNames;
    
    if (m_feedbackControllers.empty()){
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[FbckCtrlMan.getUsedRobotStates] " <<"No controllers are load"<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
        return robotStatesNames;
    }
    robotStatesNames = m_feedbackControllers.at(0)->getUsedRobotStates();
    
    for (unsigned int i = 1; i<m_feedbackControllers.size(); i++) {
        tmpNames = m_feedbackControllers.at(i)->getUsedRobotStates();
        for (auto newName: tmpNames) {
            if (!isNamePresent(robotStatesNames, newName)) {
                robotStatesNames.push_back(newName);
            }
        }
    }
    
    return robotStatesNames;
}


std::vector<std::string> FeedbackControllersManager::getUsedRobotStates(const std::string &controllerName) const 
{
    if (doesControllerExist(controllerName)) {
        return m_feedbackControllers.at( m_controllersOrder.at(controllerName) )->getUsedRobotStates();
    }
    XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->warning() << "[FbckCtrlMan.getUsedRobotStates] " <<"The Controller named: "<<controllerName<<" Doesn't Exist"<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
    std::vector<std::string> NoData;
    return NoData;
}


bool FeedbackControllersManager::doesControllerExist(const std::string &controllerName) const 
{
    for (auto controller: m_feedbackControllers) {
        if (controller->getName() == controllerName) return true;
    }
    
    return false;
}


bool FeedbackControllersManager::isNamePresent(const std::vector<std::string> &namesVector, const std::string &name) const
{
    for (auto ctrlName: namesVector) {
        if (ctrlName == name) return true;
    }
    
    return false;
}

bool FeedbackControllersManager::loadSingleController(const std::string &ctrlName, const std::string &libPath, const std::string &configFile) {

#ifdef VERBOSE
    std::cout << "ctrlName " << ctrlName << std::endl;
    std::cout << "libPath " << libPath << std::endl;
    std::cout << "configFile " << configFile << std::endl;
#endif
    m_controllerLibHandles.push_back(dlopen(libPath.c_str(), RTLD_NOW));
    if (m_controllerLibHandles.back() == NULL){
        std::cerr << dlerror() << std::endl;
        XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->error() << "[FbckCtrlMan.loadSingleController] " <<"The Controller named: "<<ctrlName<<" couldn't be loaded"<< XBot::ConsoleLogger::getLogger("/tmp/Locomotion_logger.txt")->endl();
        return false;
    }
    std::shared_ptr<FeedbackController> newController( feedbackControllerPrototypes[ctrlName]() );
    newController->initialize();
    newController->loadGains(configFile);
    newController->plot();
    
    addController(newController);
       
    return true;
}



}
