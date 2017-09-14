#include <locomotion/controllers/feedback/FeedbackController.hpp>

namespace Locomotion {

std::unordered_map<std::string, feedbackController_t *> feedbackControllerPrototypes;    
    
FeedbackController::FeedbackController(){
    this->initialize();
}

FeedbackController::~FeedbackController(){}
std::unordered_map<std::string, double> FeedbackController::getGains(){
    return m_gains;
}


bool FeedbackController::loadGains(std::string fileName){
        return false;
}

void FeedbackController::initialize(){
    m_isEnabled=false;
    m_isPaused=true;
    m_usedRobotState.clear();
    m_usedControlState.clear();
    m_gains.clear();
    m_isFirstRun=true;
}

// function to handle the controller state
void FeedbackController::enable()   {m_isEnabled=true;}
void FeedbackController::disable()  {m_isEnabled=false;}
void FeedbackController::pause()    {m_isPaused=true;}
void FeedbackController::resume()   {m_isPaused=false;}
bool FeedbackController::isEnabled(){return m_isEnabled;}
bool FeedbackController::isPaused() {return m_isPaused;}

// functions to provide information to the user
std::string FeedbackController::getName(){
    return m_controllerName;
}
std::vector<std::string> FeedbackController::getUsedControlStates(){
    return m_usedControlState;
}
std::vector<std::string> FeedbackController::getUsedRobotStates(){
    return m_usedRobotState;
}

// function to initialize the controller information
void FeedbackController::addUsedControlState(std::string usedControlStateName){
    m_usedControlState.push_back(usedControlStateName);
}
void FeedbackController::addUsedRobotState(std::string usedRobotName){
    m_usedRobotState.push_back(usedRobotName);
}
void FeedbackController::setName(const std::string controllerName){
    m_controllerName=controllerName;
}
void FeedbackController::plot(){
    std::cout<<"Control's Name:  "<<m_controllerName<<std::endl;

    if(m_isEnabled)
        std::cout<<"state: controller is ENABLED"<<std::endl;
    else
        std::cout<<"state: controller is NOT-ENABLED"<<std::endl;

    std::cout<<"    Used Robot States:"<<std::endl;
    for (std::vector<std::string>::const_iterator i = m_usedRobotState.begin(); i != m_usedRobotState.end(); ++i)
        std::cout <<"       "<< *i << std::endl;

    std::cout<<"    Used Control States:"<<std::endl;
    for (std::vector<std::string>::const_iterator i = m_usedControlState.begin(); i != m_usedControlState.end(); ++i)
        std::cout <<"       "<< *i << std::endl;
    std::cout<<std::endl;
    
}


bool FeedbackController::isUsingRobotState(std::string nameOfRobotState){
    return (std::find(this->m_usedRobotState.begin(), this->m_usedRobotState.end(), nameOfRobotState) !=  this->m_usedRobotState.end());
}
bool FeedbackController::isUsingControlState(std::string nameOfControlState){
    return (std::find(this->m_usedControlState.begin(), this->m_usedControlState.end(), nameOfControlState) !=  this->m_usedControlState.end());
}

}
