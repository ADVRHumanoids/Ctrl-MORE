#include "templates/FeedbackControllerTemplate.hpp"
#include <memory>

// When using this template make sure to set the name "MyName" in the dummy class 
// below to the name of your controller. The same name as the name of the library. 
namespace Locomotion
{
    
FeedbackControllerMyName::FeedbackControllerMyName()
{
    setName("MyName");
}

FeedbackControllerMyName::~FeedbackControllerMyName()
{
    
}
void FeedbackControllerMyName::update( const RobotState& robotState, ControlState *controlState)
{
}
    
void FeedbackControllerMyName::reset()
{
}

void FeedbackControllerMyName::getGains(std::map<std::string, double> *gains)
{
}

void FeedbackControllerMyName::setGains(const std::map<std::string, double> &gains)
{
}

void FeedbackControllerMyName::initialize()
{
}

bool FeedbackControllerMyName::loadGains(std::string fileName)
{
}




// -----------------------------------------------------------------------------
// 
//  Code below is a part of the Feedback Controller Prototype. 
//  Change only "MyName" and "FeedbackControllerMyName"!  
//
// -----------------------------------------------------------------------------


extern "C" {
std::shared_ptr<FeedbackController> feedbackControllerMaker(){
   return std::make_shared<FeedbackControllerMyName>();
}
}

//A dummy class created just to add 
class dummy {
public:
   dummy(){
      // register the maker with the feedvack controller prototypes 
      feedbackControllerPrototypes.insert({"MyName", feedbackControllerMaker});
   }
};

dummy localDummy;

}