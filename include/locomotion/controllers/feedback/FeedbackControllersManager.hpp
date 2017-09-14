/**
 * License HERE
*/

/** \file       FeedbackControllersManager.hpp
 * \brief       This is a brief description (1 sentence) of the template class.
 * \details     This is a more detailed description of the class:
 *              what it does, how it works, etc.
 * \authors     Przemyslaw Kryczka (przemyslaw.kryczka@iit.it)
 * \date        2016
 * \version     1.0
 * \bug         Description of the bugs. If no bugs, remove this line.
 * \todo        Specify the controllers order in configuration file.
 * \copyright   GNU Public License
*/

#ifndef LOCOMOTION_FEEDBACKCONTROLLERSMANAGER_HPP_
#define LOCOMOTION_FEEDBACKCONTROLLERSMANAGER_HPP_

#include <iostream> 
#include <vector>
#include <unordered_map>
#include <memory>

#include <Eigen/Dense>

#include <locomotion/controllers/feedback/FeedbackController.hpp>
#include <locomotion/states/RobotState.hpp>
#include <locomotion/states/ControlState.hpp>
#include <locomotion/utils/Logger.hpp>

namespace Locomotion {

/// \class FeedbackControllersManager FeedbackControllersManager.hpp locomotion/FeedbackControllersManager.hpp
///
/// \brief Manages the feedback controllers which based on the RobotStates modify the ControlStates.
/// 
/// The user should add all feedback controllers that he wants to use through 
/// execution of the program to the FeedbackControllersManager. After setting the 
/// order of execution and enabling controllers of interest the user needs only to 
/// call a single member function to execute all selected feedback controlelrs and 
/// apply the controll efforts to the ControlStates. 
///    
/// Use FeedbackControllersManager::addController to pass pointers to the controllers that should be used 
/// throughout execution of the program. To control order of execution use 
/// FeedbackControllersManager::setExecutionOrder. To execute feedback 
/// controllers and apply them to the control states use 
/// FeedbackControllersManager::update. To see which of the robot sates and 
/// control states are used by the particular controller use FeedbackControllersManager::getUsedRobotStates
/// and FeedbackControllersManager::getUsedControlStates, respectively.
///
/// \example examples/controllers/feedback/FeedbackControllersManagerExample.cpp
/// This is a minimal example of how to use FeedbackControllersManager.
class FeedbackControllersManager {

public:
    /// \brief Default Constructor
    FeedbackControllersManager();
    /// \brief Default Destructor
    FeedbackControllersManager(const std::string configurationFileName);

    ~FeedbackControllersManager();

    /// \brief Updates all controllers.
    ///
    /// Calls all enabled and running feedback controllers in a selected order.
    /// \param[in] robotState Contains all estimated states and feedback information
    /// \param[in] originalControlState Contains all estimated states and feedback information before any modification
    /// \param[out] modifiedControlState Contains control variables that will be overwritten by the feedback controllers.
    void update(const RobotState& robotState, const ControlState &originalControlState, ControlState *modifiedControlState);
    
    /// \brief Adds the feedback controller to the internal container 
    /// \param[in] newFeedbackController Pointer to the newly created feedback controller 
    void addController(const std::shared_ptr<FeedbackController> &newFeedbackController);

    /// \brief Enables controller for execution
    void enable(const std::string &controllerName);
    
    /// \brief  Disables the controller. It will not be called on 
    void disable(const std::string &controllerName); 
    
    /// \brief Checks if the controller of interest is enabled or disabled. 
    /// @see enable()
    /// @see disable()
    bool isEnabled(const std::string &controllerName);
    
    
    /// \brief  Pause execution of the controller. When trying to update the controller 
    ///         the last control effort will be applied to the control states. 
    void pause(const std::string &controllerName);
    
    /// \brief  Pause all the controllers using a specified robot state.
    /// \param[in] robotStateName Name of the robot state used to select the controllers
    void pauseAllUsingRobotState(const std::string &robotStateName);
    
    /// \brief  Pause all the controllers using a specified robot state.
    /// \param[in] controlStateName Name of the control state used to select the controllers
    void pauseAllUsingControlState(const std::string &controlStateName);
    
    /// \brief Resume routine
    void resume(const std::string &controllerName);
    
    /// \brief  Resume all the controllers using a specified robot state.
    /// 
    /// \warning It only resumes controllers which were specified in the \p setExecutionOrder.
    /// \param[in] robotStateName Name of the robot state used to select the controllers
    void resumeAllUsingRobotState(const std::string &robotStateName);
    
    /// \brief  Resume all the controllers using a specified control state.
    /// 
    /// \warning It only resumes controllers which were specified in the \p setExecutionOrder.
    /// \param[in] controlStateName Name of the control state used to select the controllers
    void resumeAllUsingControlState(const std::string &controlStateName);
    
    /// \brief Checks if the controller of interest is puased or running.
    /// @see pause()
    /// @see resume()
    bool isPaused(const std::string &controllerName);
    
    /// \brief  Plots all data related to the selected controller.
    void plot(std::string controllerName) const;
    
    /// \brief  Plots data related to all controllers.
    void plotAll() const;
 
    
    /// \brief Sets order of feedback controllers execution 
    /// 
    /// If the order is not set they will be executed in order in which they 
    /// were added to the manager. Calling this function will overwrites 
    /// the default order. 
    ///
    /// \param[in] executionOrder names of controllers in order in which they 
    /// are supposed to be executed. Not specified controllers, will not be 
    /// executed.
    /// \note Dynamic memory allocation.
    void setExecutionOrder(const std::vector<std::string> &executionOrder);
    
    /// \brief Returns feedback controllers in the order of execution
    /// \param[in] controllersNames  
    void getControllersNames(std::vector<std::string> *controllersNames) const;
    
    /// \brief Returns list of control states that will be overwritten by all enabled controllers
    /// \note Dynamic memory allocation.
    std::vector<std::string> getUsedControlStates() const;
    
    /// \brief Returns list of control states that will be overwritten by the specified controller
    /// \param[in] controllerName Name of the controller of interest.  
    /// \note Dynamic memory allocation.
    std::vector<std::string> getUsedControlStates(const std::string &controllerName) const;
    
    /// \brief Returns list of robot states that will be used by the specified controller
    /// \note Dynamic memory allocation.    
    std::vector<std::string> getUsedRobotStates() const;
    
    /// \brief Returns list of robot states that will be used by all enabled controllers
    /// \param[in] controllerName Name of the controller of interest.  
    /// \note Dynamic memory allocation.    
    std::vector<std::string> getUsedRobotStates(const std::string &controllerName) const;
     
    
private:

    /// \brief Verifies if an spescific controller is in the list
    /// \param[in] controllerName: controller's name to be verified
    /// \return true if controller exist false otherwise
    bool doesControllerExist(const std::string &controllerName) const ;

    /// \brief Verified if a controllers name exist within a names vector
    /// \param[in] namesVector: Vector of strings
    /// \param[in] name: name to be verified
    /// \return true if neme exist false otherwise
    bool isNamePresent(const std::vector<std::string> &namesVector, const std::string &name) const ;

    /// \brief load a single feedback controller
    /// \param[in] ctrlName: controlName to be load
    /// \param[in] libPath: path to the library of the controller
    /// \param[in] configFile: YAML config file path
    /// \return true if controller succesfully load, false otherwise
    bool loadSingleController(const std::string &ctrlName, const std::string &libPath, const std::string &configFile);

    std::vector< std::shared_ptr<FeedbackController> > m_feedbackControllers;     //!< Container of pointers to feedback controllers.
    std::unordered_map<std::string, unsigned int> m_controllersOrder;    //!< Map from controller name to index inside the m_feedbackControllers vector.
    std::vector<int> m_executionOrder;                          //!< Indeces of feedback controllers to be executed.
    std::vector<void *> m_controllerLibHandles;
    
};


} 

#endif
