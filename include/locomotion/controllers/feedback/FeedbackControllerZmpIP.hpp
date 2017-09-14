/**
 * License HERE
*/

/** \file       FeedbackControllerZmpIP.hpp
 * \brief       This is a brief description (1 sentence) of the template class.
 * \details     This is a more detailed description of the class:
 *              what it does, how it works, etc.
 * \authors     Name1 (name1@iit.it), Name2 (name2@iit.it)
 * \date        2016
 * \version     1.0
 * \copyright   GNU Public License
*/

#ifndef PROJECTNAME_MYTEMPLATECLASS_HPP_
#define PROJECTNAME_MYTEMPLATECLASS_HPP_


// second, include other libraries like Eigen, Boost,...
#include <Eigen/Dense>

#include "locomotion/controllers/feedback/FeedbackController.hpp"
#include "locomotion/controllers/feedback/PIDcontrol.hpp"

namespace Locomotion {

/// \class FeedbackControllerZmpIP FeedbackControllerZmpIP.hpp locomotion/FeedbackControllerZmpIP.hpp
///
/// \brief Description of what this class does. 
class FeedbackControllerZmpIP: public FeedbackController{

public:
    /// \brief Basic constructor
    FeedbackControllerZmpIP();

    virtual ~FeedbackControllerZmpIP();

    //You have to extend all of these functions !!
    virtual void update( const RobotState& robotState, const ControlState &originalControlState, ControlState *modifiedControlState);
    virtual void reset();
    virtual void setGains(const std::map<std::string, double> &gains);
    virtual void initialize();
    virtual bool loadGains(std::string fileName);
    
protected:

private:
    //Inverted pendulum controller related attributes 
    Eigen::Vector2d m_zmpModification;
    Eigen::Vector3d m_newZmpReference;
    std::shared_ptr<PIDcontrol> m_comPid_x, m_comPid_y;

    //Zmp controller related attributes 
    Eigen::Vector2d m_zmpRef2Meas, m_zmpErrorLim, m_pelvisModification;
    Eigen::Vector3d m_newComRef;
    Eigen::VectorXd m_newPelvisRef;
    
    std::shared_ptr<PIDcontrol> m_zmpPid_x, m_zmpPid_y;
    
    double m_controllCycleTime;
    
};


} 

#endif
