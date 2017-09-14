#include "locomotion/controllers/feedback/CoMPID/CoMPIDcontroller.hpp"

#include "yaml-cpp/yaml.h"

#include <memory>

// When using this template make sure to set the name "MyName" in the dummy class
// below to the name of your controller. The same name as the name of the library.
namespace Locomotion
{


/// \brief The default constructor: Inheritead from FeedbackController class
///
///  is aim to initialize the controler name and used variables.
/// this will be use inside the loop to describe the controller
/// and look for the related feedbacks and output signals
CoMPIDcontroller::CoMPIDcontroller():
    FeedbackController()//, PIDcontrol()
{


    m_controllCycleTime = 0.001;
    /// \brief setName: FeedbackController function, name that identifies the controller
    setName("CoMPIDcontroller");

    /// \brief addUsedControlState: FeedbackController function, name of the variables
    /// modified by the controller (control effort)
    this->addUsedControlState("com");

    /// \brief addUsedRobotState: FeedbackController function, name of the variables
    /// used as feedback by the controller (reference, measures, estimations)
    this->addUsedRobotState("com");
    comreference.resize(7);
    /// \note These functions must be used since are the ones that inform the user
    /// about the control variables
    reset();
}

CoMPIDcontroller::~CoMPIDcontroller()
{
    std::cout << "Destructing My controller " << std::endl;

}

/// Define the controllers update function
void CoMPIDcontroller::update( const RobotState& robotState, const ControlState &originalControlState, ControlState *modifiedControlState)
{
   // if not enable do nothing
    if (!isEnabled() ){
        std::cout<<"is not enabled."<<std::endl;
        return;
    }

    m_newComRef = modifiedControlState->getCOM();
    originalControlState.getReferencePose("com",&comreference);
    modifiedControlState->getReferencePose("com",&comreference);




	

    m_newComRef(0)=comreference(0)+m_CoMPID_x->getOutput(m_newComRef(0)-comreference(0));
    m_newComRef(1)=comreference(1)+m_CoMPID_y->getOutput(m_newComRef(1)-comreference(1));
    m_newComRef(2)=comreference(2)+m_CoMPID_z->getOutput(m_newComRef(2)-comreference(2));
    modifiedControlState->setCOM(m_newComRef);
	
	std::cout<<"new ref:  "<<m_newComRef.transpose()<<std::endl;

	
}

void CoMPIDcontroller::reset()
{
    /// initialize variables at zero
    m_newComRef          = Eigen::Vector3d::Zero();

}
///
/// \brief This permits the online gain manipulation.
/// \param[in] gains: map of gains containig the new values.
///         The developer and user should guarantee compatibility
void CoMPIDcontroller::setGains(const std::map<std::string, double> &gains)
{
    /// This permits the online gain manipulation.
    std::cout << "Function not supported" << std::endl;
}

/// \brief initialize the controller
void CoMPIDcontroller::initialize()
{
    reset();
    m_controllCycleTime = 0.001; //?????
}

/// \brief Permit to load the gains of the controller from a yaml file
///        the yaml file may contain all the controller requirements
///        and also link to other files  \see AttitudeMPCstabilizer.hpp
/// \param fileName: path of yaml file to be load
///        \note This is handle by the controller manager
/// \return
bool CoMPIDcontroller::loadGains(std::string fileName)
{
    YAML::Node config = YAML::LoadFile(fileName.c_str());
    /// Once the file is load, store the yaml variables into the corresponding
    /// ones in your controller
    m_gains.clear();
    m_gains.reserve(15);
    m_gains.insert({"CoMPID_x_p", config["CoMPID_x"][0].as<double>()});
    m_gains.insert({"CoMPID_x_i", config["CoMPID_x"][1].as<double>()});
    m_gains.insert({"CoMPID_x_d", config["CoMPID_x"][2].as<double>()});
    m_gains.insert({"CoMPID_lower_x",config["lowerBoundx"].as<double>()});
    m_gains.insert({"CoMPID_upper_x",config["upperBoundx"].as<double>()});

    m_gains.insert({"CoMPID_y_p", config["CoMPID_y"][0].as<double>()});
    m_gains.insert({"CoMPID_y_i", config["CoMPID_y"][1].as<double>()});
    m_gains.insert({"CoMPID_y_d", config["CoMPID_y"][2].as<double>()});
    m_gains.insert({"CoMPID_lower_y",config["lowerBoundy"].as<double>()});
    m_gains.insert({"CoMPID_upper_y",config["upperBoundy"].as<double>()});

    m_gains.insert({"CoMPID_z_p", config["CoMPID_z"][0].as<double>()});
    m_gains.insert({"CoMPID_z_i", config["CoMPID_z"][1].as<double>()});
    m_gains.insert({"CoMPID_z_d", config["CoMPID_z"][2].as<double>()});
    m_gains.insert({"CoMPID_lower_z",config["lowerBoundz"].as<double>()});
    m_gains.insert({"CoMPID_upper_z",config["upperBoundz"].as<double>()});


    //Initialize  the com PID controllers

    m_CoMPID_x = std::make_shared<PIDcontrol> (
                        m_gains.at("CoMPID_x_p"),
                        m_gains.at("CoMPID_x_i"),
                        m_gains.at("CoMPID_x_d"),
                        m_controllCycleTime,
                        m_gains.at("CoMPID_lower_x"),
                        m_gains.at("CoMPID_upper_x") );


    m_CoMPID_y = std::make_shared<PIDcontrol> (
                        m_gains.at("CoMPID_y_p"),
                        m_gains.at("CoMPID_y_i"),
                        m_gains.at("CoMPID_y_d"),
                        m_controllCycleTime,
                        m_gains.at("CoMPID_lower_y"),
                        m_gains.at("CoMPID_upper_y"));

    m_CoMPID_z = std::make_shared<PIDcontrol> (
                        m_gains.at("CoMPID_z_p"),
                        m_gains.at("CoMPID_z_i"),
                        m_gains.at("CoMPID_z_d"),
                        m_controllCycleTime,
                        m_gains.at("CoMPID_lower_z"),
                        m_gains.at("CoMPID_upper_z"));


//    std::cout << "CoMPID controller gains loading was successful" << std::endl;

    return true;
}




// -----------------------------------------------------------------------------
//
//  Code below is a part of the Feedback Controller Prototype.
//
// -----------------------------------------------------------------------------

/// This is mandatory to load single libraries when compiling according to the
/// feedback controller manarer yaml file
extern "C" {
FeedbackController* CoMPIDMaker(){

   return new CoMPIDcontroller();
}
}

///A dummy class created just to add the controller to the manager
/// the dummy name is independent for each controller
class dummyCoMPID {
public:
   dummyCoMPID(){
      /// register the maker with the feedback controller prototypes
      feedbackControllerPrototypes.insert({"CoMPIDcontroller", CoMPIDMaker});
      /// the name parameter should agree with the one used in the constructor
   }
};

dummyCoMPID localDummyCoMPID;

}
