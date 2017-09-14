#include "locomotion/controllers/feedback/ControlExample/ControlExample.hpp"

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
ControlExample::ControlExample():
    FeedbackController()
{
    /// \brief setName: FeedbackController function, name that identifies the controller
    setName("MyController");

    /// \brief addUsedControlState: FeedbackController function, name of the variables
    /// modified by the controller (control effort)
    this->addUsedControlState("pelvis");

    /// \brief addUsedRobotState: FeedbackController function, name of the variables
    /// used as feedback by the controller (reference, measures, estimations)
    this->addUsedRobotState("zmp");

    /// \note These functions must be used since are the ones that inform the user
    /// about the control variables
    reset();
}

ControlExample::~ControlExample()
{
    std::cout << "Destructing My example controller " << std::endl;

}

/// Define the controllers update function
void ControlExample::update( const RobotState& robotState, const ControlState &originalControlState, ControlState *modifiedControlState)
{
    /// if not enable do nothing
    if (!isEnabled() ) return;


    ///Apply ZMP reference modification

    /// Read robot state or modified controller state in case of cascade controllers
    m_newZmpReference = modifiedControlState->getZMP();
    m_newZmpReference.head(2) += m_zmpModification.head(2);
    /// set the new control state (control effor, new reference, etc.)
    modifiedControlState->setZMP(m_newZmpReference);


    //Limit the ZMP error to not overcompensate (in case the reference is out of support polygon
    m_zmpRef2Meas = robotState.getEstimatedData("zmp").head(2) - m_newZmpReference.head(2);
    for(unsigned int axis=0; axis<2; axis++){
        if (m_zmpRef2Meas(axis) > m_zmpErrorLim(axis)) m_zmpRef2Meas(axis) = m_zmpErrorLim(axis);
        if (m_zmpRef2Meas(axis) < -m_zmpErrorLim(axis)) m_zmpRef2Meas(axis) = -m_zmpErrorLim(axis);
    }

    //If paused do not update the control in this case used pass reference
    if (!isPaused()) {
        /// Used previous control action in the delta control effort
        m_pelvisModification(0) = -m_zmpPid_x->getOutput(m_zmpRef2Meas(0));
        m_pelvisModification(1) = -m_zmpPid_y->getOutput(m_zmpRef2Meas(1));
    }

    //Add the controller action to com reference
    /// The accomulative control effort read the present variable state
    m_newComRef = modifiedControlState->getCOM();
    /// The accomulative control effort added the delta (oputut from the PID)
    m_newComRef.head(2) += m_pelvisModification;
    /// modified (update) the ControlState CoM
    modifiedControlState->setCOM(m_newComRef);

    //Add the same modification to the pelvis reference
    if (modifiedControlState->referenceExists("pelvis")) {
        m_newPelvisRef = modifiedControlState->getReferencePose("pelvis");
        m_newPelvisRef.head(2) += m_pelvisModification;
        /// modified (update) the ControlState "pelvis"
        modifiedControlState->setReferencePose("pelvis", m_newPelvisRef);
    }

}

void ControlExample::reset()
{
    /// initialize variables at zero
    m_pelvisModification = Eigen::Vector2d::Zero();
    m_newComRef          = Eigen::Vector3d::Zero();
    m_newPelvisRef       = Eigen::VectorXd::Zero(7);
}
///
/// \brief This permits the online gain manipulation.
/// \param[in] gains: map of gains containig the new values.
///         The developer and user should guarantee compatibility
void ControlExample::setGains(const std::map<std::string, double> &gains)
{
    /// This permits the online gain manipulation.
    std::cout << "Function not supported" << std::endl;
}

/// \brief initialize the controller
void ControlExample::initialize()
{
    reset();
    m_controllCycleTime = 0; //?????
}

/// \brief Permit to load the gains of the controller from a yaml file
///        the yaml file may contain all the controller requirements
///        and also link to other files  \see AttitudeMPCstabilizer.hpp
/// \param fileName: path of yaml file to be load
///        \note This is handle by the controller manager
/// \return
bool ControlExample::loadGains(std::string fileName)
{
    YAML::Node config = YAML::LoadFile(fileName.c_str());
    /// Once the file is load, store the yaml variables into the corresponding
    /// ones in your controller
    m_gains.clear();
    m_gains.reserve(10);
    m_gains.insert({"zmpPid_x_p", config["zmpPid_x"][0].as<double>()});
    m_gains.insert({"zmpPid_x_i", config["zmpPid_x"][1].as<double>()});
    m_gains.insert({"zmpPid_x_d", config["zmpPid_x"][2].as<double>()});
    m_gains.insert({"zmpPid_y_p", config["zmpPid_y"][0].as<double>()});
    m_gains.insert({"zmpPid_y_i", config["zmpPid_y"][1].as<double>()});
    m_gains.insert({"zmpPid_y_d", config["zmpPid_y"][2].as<double>()});

    m_gains.insert({"zmpPid_displLim_x", config["zmpPid_displLim_x"].as<double>()});
    m_gains.insert({"zmpPid_displLim_x", config["zmpPid_displLim_x"].as<double>()});

    m_gains.insert({"zmpPid_displLim_y", config["zmpPid_displLim_y"].as<double>()});
    m_gains.insert({"zmpPid_displLim_y", config["zmpPid_displLim_y"].as<double>()});


    //Initialize  the com PID controllers
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

    std::cout << "ZmpIP controller gains loading was successful" << std::endl;

    return true;
}




// -----------------------------------------------------------------------------
//
//  Code below is a part of the Feedback Controller Prototype.
//
// -----------------------------------------------------------------------------

// This is mandatory to load single libraries when compiling according to the
// feedback controller manarer yaml file
extern "C" {
FeedbackController* feedbackExampleMaker(){

   return new ControlExample();
}
}

///A dummy class created just to add the controller to the manager
/// the dummy name is independent for each controller
class dummyExample {
public:
   dummyExample(){
      /// register the maker with the feedback controller prototypes
      feedbackControllerPrototypes.insert({"MyController", feedbackExampleMaker});
      /// the name parameter should agree with the one used in the constructor
   }
};

dummyExample localDummyExample;

}
