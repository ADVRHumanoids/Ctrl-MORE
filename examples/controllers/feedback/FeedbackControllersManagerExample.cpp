#include <memory>

#include "locomotion/controllers/feedback/FeedbackController.hpp"
#include "locomotion/controllers/feedback/FeedbackControllersManager.hpp"
#include "locomotion/controllers/feedback/AttitudeMPCstabilizer/AttitudeMPCstabilizer.hpp"


using namespace std;

int main()
{
    shared_ptr<Locomotion::FeedbackController> controllerA(new Locomotion::AttitudeMPCstabilizer);
//     Locomotion::FeedbackController *controllerB;
    
    //Here we will use constructor of one of the impleneted feedback controllers
    //which is extending FeedbackController class. For the moment let's use
    //FeedbackController class directly (it will not compile, since it has pure
    //virutal functions inside).
//    controllerA = make_shared<Locomotion::AttitudeMPCstabilizer>();
//     controllerB = new FeedbackController;
    
    
    Locomotion::FeedbackControllersManager feedbackControllers;
    
    //Add controllers to the controller manager
    feedbackControllers.addController(controllerA);
//     feedbackControllers.addController(controllerB);
    
    // This is just a temporary variable. It should be filled in by the user.
    // For content refer to documentation Locomotion::RobotState
    Locomotion::RobotState robotState;
    
    // This is just a temporary variable. It should be filled in by the locomotion
    // controller. For details refer to documentation Locomotion::ControlState
    Locomotion::ControlState controlState;
    
    //Let's assume that this is our control loop
    while(1) {
    
        //The user will update the present state of the robot
        //updateRobotState(&robotState);
        
        //And execute another step of the locomotion controller
        //runLocomotionControllerStep(&controlState);
                
        // Pass to the class the present state of the robot and lastest controll states.
        // The controll states will be modified by all enabled controllers
//         feedbackControllers.update(robotState, &controlState);
        
        // And now it can be executed on the robot.
        //executeOnTheRobot(controlState);
    }
    
}
