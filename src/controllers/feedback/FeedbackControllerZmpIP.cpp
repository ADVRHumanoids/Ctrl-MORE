#include "locomotion/controllers/feedback/FeedbackControllerZmpIP.hpp"

#include "yaml-cpp/yaml.h"

#include <memory>

// When using this template make sure to set the name "MyName" in the dummy class 
// below to the name of your controller. The same name as the name of the library. 
namespace Locomotion
{
    
FeedbackControllerZmpIP::FeedbackControllerZmpIP():
    FeedbackController()
{
    setName("ZmpIP");
    this->addUsedControlState("pelvis");
    this->addUsedRobotState("zmp");
    reset();
}

FeedbackControllerZmpIP::~FeedbackControllerZmpIP()
{
    std::cout << "Destructing the ZMP IP controller " << std::endl;
    
}
void FeedbackControllerZmpIP::update( const RobotState& robotState, const ControlState &originalControlState, ControlState *modifiedControlState)
{
           
    if (!isEnabled() ) return;

    //If paused don't change zmp reference
    std::cout<<"control "<<isPaused()<<std::endl;
    if ( !isPaused() ) {
        m_zmpModification(0) = m_comPid_x->getOutput(-m_pelvisModification(0));
        m_zmpModification(1) = m_comPid_y->getOutput(-m_pelvisModification(1));
    }
    
    //Apply ZMP reference modification 
    m_newZmpReference = modifiedControlState->getZMP();
    m_newZmpReference.head(2) += m_zmpModification.head(2);
    modifiedControlState->setZMP(m_newZmpReference); 
    
    
    //Limit the ZMP error to not overcompensate (in case the reference is out of support polygon
    m_zmpRef2Meas = robotState.getEstimatedData("zmp").head(2) - m_newZmpReference.head(2);
    for(unsigned int axis=0; axis<2; axis++){
        if (m_zmpRef2Meas(axis) > m_zmpErrorLim(axis)) m_zmpRef2Meas(axis) = m_zmpErrorLim(axis);
        if (m_zmpRef2Meas(axis) < -m_zmpErrorLim(axis)) m_zmpRef2Meas(axis) = -m_zmpErrorLim(axis);
    }
    
    //If paused do not update the control action 
    if (!isPaused()) {
        m_pelvisModification(0) = -m_zmpPid_x->getOutput(m_zmpRef2Meas(0)); 
        m_pelvisModification(1) = -m_zmpPid_y->getOutput(m_zmpRef2Meas(1));
    }
    
    //Add the zmp controller action to com reference 
    m_newComRef = modifiedControlState->getCOM();
    m_newComRef.head(2) += m_pelvisModification;
    modifiedControlState->setCOM(m_newComRef);
    
    //Add the same modification to the pelvis reference 
    if (modifiedControlState->referenceExists("pelvis")) {
        m_newPelvisRef = modifiedControlState->getReferencePose("pelvis");
        m_newPelvisRef.head(2) += m_pelvisModification;
        modifiedControlState->setReferencePose("pelvis", m_newPelvisRef);
    }
    
}
    
void FeedbackControllerZmpIP::reset()
{
    //Inverted pendulum controller related attributes 
    m_zmpModification = Eigen::Vector2d::Zero();
    m_newZmpReference = Eigen::Vector3d::Zero();
    
    //Zmp controller related attributes 
    m_zmpRef2Meas        = Eigen::Vector2d::Zero();
    m_zmpErrorLim        = Eigen::Vector2d::Zero();
    m_pelvisModification = Eigen::Vector2d::Zero();
    m_newComRef          = Eigen::Vector3d::Zero();
    m_newPelvisRef       = Eigen::VectorXd::Zero(7);
}

void FeedbackControllerZmpIP::setGains(const std::map<std::string, double> &gains)
{
    std::cout << "Function not supported" << std::endl;
}

void FeedbackControllerZmpIP::initialize()
{
    reset();
    m_controllCycleTime = 0; //?????
}

bool FeedbackControllerZmpIP::loadGains(std::string fileName)
{
    YAML::Node config = YAML::LoadFile(fileName.c_str());
    
    m_gains.clear();
    m_gains.reserve(20);
    m_gains.insert({"comPid_x_p", config["comPid_x"][0].as<double>()});
    m_gains.insert({"comPid_x_i", config["comPid_x"][1].as<double>()}); 
    m_gains.insert({"comPid_x_d", config["comPid_x"][2].as<double>()}); 
    m_gains.insert({"comPid_displLim_x", config["comPid_displLim_x"].as<double>()});
    m_gains.insert({"comPid_displLim_x", config["comPid_displLim_x"].as<double>()});
    m_gains.insert({"comPid_y_p", config["comPid_y"][0].as<double>()});
    m_gains.insert({"comPid_y_i", config["comPid_y"][1].as<double>()});
    m_gains.insert({"comPid_y_d", config["comPid_y"][2].as<double>()});
    m_gains.insert({"comPid_displLim_y", config["comPid_displLim_y"].as<double>()});
    m_gains.insert({"comPid_displLim_y", config["comPid_displLim_y"].as<double>()});
    m_gains.insert({"zmpPid_x_p", config["zmpPid_x"][0].as<double>()});
    m_gains.insert({"zmpPid_x_i", config["zmpPid_x"][1].as<double>()});
    m_gains.insert({"zmpPid_x_d", config["zmpPid_x"][2].as<double>()});
    m_gains.insert({"zmpPid_displLim_x", config["zmpPid_displLim_x"].as<double>()});
    m_gains.insert({"zmpPid_displLim_x", config["zmpPid_displLim_x"].as<double>()});
    m_gains.insert({"zmpPid_y_p", config["zmpPid_y"][0].as<double>()});
    m_gains.insert({"zmpPid_y_i", config["zmpPid_y"][1].as<double>()});
    m_gains.insert({"zmpPid_y_d", config["zmpPid_y"][2].as<double>()});
    m_gains.insert({"zmpPid_displLim_y", config["zmpPid_displLim_y"].as<double>()});
    m_gains.insert({"zmpPid_displLim_y", config["zmpPid_displLim_y"].as<double>()});
                                        
    
    //Initialize  the com PID controllers 
    m_comPid_x = std::make_shared<PIDcontrol> 
                       ( m_gains.at("comPid_x_p"), 
                         m_gains.at("comPid_x_i"), 
                         m_gains.at("comPid_x_d"), 
                         m_controllCycleTime, 
                         m_gains.at("comPid_displLim_x"),
                         -m_gains.at("comPid_displLim_x"));
                       
//     m_comPid_x->setErrorLPFCutOffFreqency(m_gains.at("comPid_errorCutOffF"));
//     m_comPid_x->setDerivativeLPFCutOffFreq(m_gains.at("comPid_errorDtCutOffF"));
    
    m_comPid_y = std::make_shared<PIDcontrol> 
                       ( m_gains.at("comPid_y_p"), 
                         m_gains.at("comPid_y_i"), 
                         m_gains.at("comPid_y_d"), 
                         m_controllCycleTime, 
                         m_gains.at("comPid_displLim_y"), 
                         -m_gains.at("comPid_displLim_y"));

//     m_comPid_x->setErrorLPFCutOffFreqency(m_gains.at("comPid_errorCutOffF"));
//     m_comPid_x->setDerivativeLPFCutOffFreq(m_gains.at("comPid_errorDtCutOffF"));
                       
//     m_comPid_x->smoothBlendIn(1);
//     m_comPid_y->smoothBlendIn(1);
        
    
    m_zmpPid_x = std::make_shared<PIDcontrol> (
                        m_gains.at("zmpPid_x_p") * m_controllCycleTime * 1000.0, 
                        m_gains.at("zmpPid_x_i") * m_controllCycleTime * 1000.0, 
                        m_gains.at("zmpPid_x_d") , 
                        m_controllCycleTime,  
                        m_gains.at("zmpPid_displLim_x"), 
                        -m_gains.at("zmpPid_displLim_x") );
    m_zmpPid_y = std::make_shared<PIDcontrol> (
                        m_gains.at("zmpPid_y_p") * m_controllCycleTime * 1000.0, 
                        m_gains.at("zmpPid_y_i") * m_controllCycleTime * 1000.0, 
                        m_gains.at("zmpPid_y_d") , 
                        m_controllCycleTime,  
                        m_gains.at("zmpPid_displLim_y"), 
                        -m_gains.at("zmpPid_displLim_y")); 
//     m_zmpPid_x->smoothBlendIn(2);
//     m_zmpPid_y->smoothBlendIn(2);
    
    std::cout << "ZmpIP controller gains loading was successful" << std::endl;
    
    return true;
}




// -----------------------------------------------------------------------------
// 
//  Code below is a part of the Feedback Controller Prototype. 
//
// -----------------------------------------------------------------------------


extern "C" {
FeedbackController* feedbackControllerMaker(){
   return new FeedbackControllerZmpIP();
}
}

//A dummy class created just to add 
class dummyZmpIp {
public:
   dummyZmpIp(){
      // register the maker with the feedback controller prototypes 
      feedbackControllerPrototypes.insert({"ZmpIP", feedbackControllerMaker});
   }
};

dummyZmpIp localDummy;

}
