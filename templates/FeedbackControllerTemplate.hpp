/**
 * License HERE
*/

/** \file       FeedbackControllerMyName.hpp
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

namespace Locomotion {

/// \class FeedbackControllerMyName FeedbackControllerMyName.hpp locomotion/FeedbackControllerMyName.hpp
///
/// \brief Description of what this class does. 
class FeedbackControllerMyName: public FeedbackController{

public:

    /// \brief Basic constructor
    FeedbackControllerMyName();

    virtual ~FeedbackControllerMyName();

    //You have to extend all of these functions !!
    virtual void update( const RobotState& robotState, ControlState *controlState);
    virtual void reset();
    virtual void getGains(std::map<std::string, double> *gains);
    virtual void setGains(const std::map<std::string, double> &gains);
    virtual void initialize();
    virtual bool loadGains(std::string fileName);
    
protected:

private:

};


} 

#endif
