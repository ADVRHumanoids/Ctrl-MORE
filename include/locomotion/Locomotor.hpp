/**
 * License HERE
*/

/** \file       Locomotor.hpp
 * \brief       This is a brief description (1 sentence) of the template class.
 * \details     This is a more detailed description of the class:
 *              what it does, how it works, etc.
 * \authors     Przemyslaw Kryczka(przemyslaw.kryczka@iit.it)
 * \date        2016
 * \version     1.0
 * \copyright   GNU Public License
*/

#ifndef LOCOMOTION_LOCOMOTOR_HPP_
#define LOCOMOTION_LOCOMOTOR_HPP_


#include <iostream> 
#include <vector>

#include <Eigen/Dense>

#include <locomotion/states/ControlState.hpp>
#include <locomotion/states/RobotState.hpp>
#include <locomotion/controllers/feedback/FeedbackControllersManager.hpp>
#include <locomotion/controllers/locomotion/LocomotionReference.hpp>
#include <locomotion/controllers/locomotion/LocomotionControllerManager.hpp>
#include <locomotion/utils/utils.hpp>


/// \namespace Locomotion
/// Locomotion module namespace
namespace Locomotion {

/// \class Locomotor Locomotor.hpp locomotion/controllers/locomotion/Locomotor.hpp
///
/// \brief This is the highest level abstration of the locomotion module.
///
/// 
///
///
///    
///
/// \example examples/controllers/locomotion/Locomotor.cpp
/// This is an example of how to use the Locomotor class. 
class Locomotor {

public:

    /// \brief Basic constructor
    Locomotor();

    /// \brief Constructor with configuration path
    Locomotor(const std::string configurationFileName);
    ~Locomotor();

    /// \brief Initiate locomotion controller to perform a walk as specified in 
    /// \p reference.
    void walk(const std::string locomotionControllerName, const LocomotionReference &reference, const RobotState &robotState);
    
    /// \brief Use to send customized commands to the locomotion module.
    void command(const std::string locomotionControllerName, std::string commandName);

    /// \brief Iterate all controllers and update output. 
    /// 
    /// Used to update internal state of the locomotion controller, execute 
    /// active locomotion controller and apply selected feedback controllers. 
    /// After applying this function user can get new task or joint space 
    /// references. 
    /// \param[in] robotState Contains all estimated states and feedback information
    /// \param[in] originalControlState Contains all estimated states and feedback information before any modification
    /// \param[out] modifiedControlState Contains control variables that will be overwritten by the feedback controllers.
    void update(const RobotState& robotState, ControlState *modifiedControlState);

    
    /// \brief Returns COM reference 
    /// \return Vector 9x1 containing position, velocity and acceleration of COM.
    Eigen::VectorXd getRefCom();
    
    /// \brief Returns ZMP reference 
    /// \return Vector 3x1 containing position of reference ZMP in 3D space
    Eigen::VectorXd getRefZmp();
    
    /// \brief Returns linear momentum reference 
    /// \return Vector 3x1 containing linear momentum values in 3D space. 
    Eigen::VectorXd getRefLinearMomentum();
    
    /// \brief Returns angular momentum reference 
    /// \return Vector 3x1 containing angular momentum values in 3D space. 
    Eigen::VectorXd getRefAngularMomentum();
    
    /// \brief Returns a chosen task space reference (pos, vel, acc). 
    /// \return Vector 18x1 containing pose(6x1), velocity(6x1), acceleration(6x1) 
    Eigen::VectorXd getRefTask(std::string nameOfReference);
    
    /// \brief Returns a chosen joint space reference (position, torque) 
    /// \return Vector nx1 with whole body joint references 
    Eigen::VectorXd getRefJoint(std::string nameOfReference);
    std::shared_ptr<FeedbackControllersManager> m_feedbackControllers; //!< Manages execution of feedback controllers
    std::shared_ptr<LocomotionControllersManager> m_locoControllers;//!< Manages execution of Locomotion controllers


private:
    // put the private methods and attributes here and respect the 
    // same order as above.
    // Constructors, Destructor, pure virtual methods, virtual methods,
    // methods, setters, getters, operator overloading, member variables.
    

    ControlState m_locomotionControlState; //!< control states that result from locomotion controller
    ControlState m_finalControlState; //!<modified by feedback controllers used for execution

    RobotState m_robotStates;  //!< Present robot States;

    //std::shared_ptr<FeedbackControllersManager> m_feedbackControllers; //!< Manages execution of feedback controllers
};

}

#endif
