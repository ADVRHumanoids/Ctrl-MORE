#include "locomotion/controllers/feedback/FeedbackController.hpp"
#include "locomotion/controllers/feedback/FeedbackControllersManager.hpp"


using namespace Locomotion;
using namespace std;

int main() 
{
    
    //We initialize the feedback controllers manager with configuration file 
    
    string fdbkConfFiles = "../configs/fdbkControllers.yaml";
    Locomotion::FeedbackControllersManager feedbackControllers(fdbkConfFiles); 

    
    // This is just a temporary variable. It should be filled in by the user. 
    // For content refer to documentation Locomotion::RobotState
    Locomotion::RobotState robotState;
    
    // This is just a temporary variable. It should be filled in by the locomotion 
    // controller. For details refer to documentation Locomotion::ControlState
    Locomotion::ControlState controlState; 
    
    //Let's assume that this is our control loop 
//     while(1) {
    
        //The user will update the present state of the robot 
        //updateRobotState(&robotState);
        
        //And execute another step of the locomotion controller 
        //runLocomotionControllerStep(&controlState);
                
        // Pass to the class the present state of the robot and lastest controll states.
        // The controll states will be modified by all enabled controllers 
//         feedbackControllers.update(robotState, &controlState);
        
        // And now it can be executed on the robot.
        //executeOnTheRobot(controlState);
//     }
    cout << "Exiting main! " << endl;
    
}