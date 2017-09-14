/**
 * License HERE
*/

/** \file       FeedbackController.hpp
 * \brief       This is the base class for the Feedback contronller in locomotion
 *
 * \details     as base class, this interface provides the general flags, active, pause ..
 *              and the general functionalities that the different feedback controllers
 *              must have
 *
 * \authors     Juan Alejandro Castano (juan.castano@iit.it)
 * \date        2016
 * \version     1.0

 * \copyright   GNU Public License
*/

#ifndef LOCOMOTION_FeedbackController_HPP_
#define LOCOMOTION_FeedbackController_HPP_

/// first, include standard libraries
#include <fstream>
#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <map>
#include <memory>
// #include <algorithm>

/// second, include other libraries like Eigen, Boost,...
#include <Eigen/Dense> ///To change according to the EIGEN path

#include <locomotion/states/ControlState.hpp>
#include <locomotion/states/RobotState.hpp>

/// \namespace Locomotion
/// Global namespace fot the Locomotion project, for more info read
///         " ???.cpp"

namespace Locomotion {


class FeedbackController {
public:
//     enum controllerState {
//         paused,             //!< paused
//         running,            //!< paused
//         stopped};           //!< paused

    FeedbackController();

    /// \brief notice that there are virtual and pure virtual methods
    ///
    virtual ~FeedbackController();

    /// \brief Apply a control action to the robot's states 
    ///        according with the reference states
    ///
    /// \param[in] robotState   Present state of the robot 
    /// \param[in] originalControlState Present control states provided by locomotion controller
    /// \param[out] odifiedControlState Modified version of the control state
    virtual void update(const RobotState& robotState, const ControlState &originalControlState, ControlState *modifiedControlState) = 0;
       
    /// \brief Resets all the internal states of the controller
    virtual void reset() = 0;

    /// \brief Sets values of specified gains.
    /// \param[in] gains   Map of gains for the controller
    virtual void setGains(const std::map<std::string, double> &gains) = 0;
    
    /// \brief Load internal controller parameters from a file. 
    /// 
    /// Load gains, initialize internal memory etc.
    /// \param[in] fileName String with the path to the file where the initialization info is
    ///                     stored, might contain: offsets, limits, gains ...
    virtual void initialize();

    /// \brief Loads the set of gains of the controller according to
    ///        what the child class implements. According to the parser
    ///        functionalities, this might change or desapear
    /// \param[in] fileName Path to the gains file
    /// \return     False when there is no file to be loaded or is not propertly defined
    ///             i.e. wrong size, null data ..., true when load are load propertly
    virtual bool loadGains(std::string fileName);
    
    /// \brief Plots information about the controller
    ///
    /// ControlName
    /// state: controller is ENABLED (NOT-ENABLED)
    ///     Used Robot States:
    ///         A
    ///         B
    ///     Used Control States:
    ///         a
    ///         b
    void plot();

    /// \brief Get the gains information of the specific controller.
    /// \return A map composed of the gains name and it's value/s.
    std::unordered_map<std::string, double> getGains();
    
    
    /// \brief Returns controller's name. 
    /// \return Controller name.
    std::string getName();

    /// \brief Returns a list of names of control states that are being changed by the controller.
    /// \return Vector of string containing the names.
    std::vector<std::string> getUsedControlStates();
    
    /// \brief Checks if the controller is using a particular control state
    bool isUsingControlState(std::string nameOfControlState);
    
    /// \brief Returns a list of names of robot states that are being used by the controller.
    /// \return Vector of string containing the names.
    std::vector<std::string> getUsedRobotStates();
    
    /// \brief Checks if the controller is using a particular robot state
    bool isUsingRobotState(std::string nameOfRobotState);
    /// \brief Tells if the controller is being used
    bool isEnabled();

    /// \brief Tells if the controller is paused
    bool isPaused();

    /// \brief Enables controller for execution
    void enable();
    
    /// \brief  Disables controller. When trying to update the controller no change 
    ///         to controlStates will happen. 
    void disable();

    /// \brief  Pause execution of the controller. When trying to update the controller 
    ///         the last control action will be applied to the control states. 
    void pause();
    
    /// \brief Resume routine
    void resume();
        
protected:    
    /// \brief Adds to the list a name of the control state that is being modified by the controller. 
    void addUsedControlState(std::string usedControlStateName);
    
    /// \brief Adds to the list a name of the robot state that is being used by the controller. 
    void addUsedRobotState(std::string usedRobotName);

    /// \brief Sets controller's name. 
    void setName(const std::string controllerName);
    
    std::unordered_map<std::string ,double> m_gains;  //!< Controller gains
    bool m_isFirstRun;//!< permits soft control starting
    
private:        
    std::vector<std::string> m_usedRobotState;     //!< Vector of names of used robot states
    std::vector<std::string> m_usedControlState;   //!< Vector of names of used control states 
    
    bool m_isEnabled; //!< indicates wheather or not the controller is enabled
    bool m_isPaused;  //!< indicates wheather or not the controller is paused
    std::string m_controllerName;//!< Contains the controllers unique name

    
//     controllerState m_controllerState;    
    };
///
/// \brief Controller type variable to handle the different controller
///         as libraries
/// \return A pointer to a Feedback controller controller
///
typedef FeedbackController* feedbackController_t(void);

/// \brief Map of controller prototypes
extern std::unordered_map<std::string, feedbackController_t *> feedbackControllerPrototypes;
} 



#endif
