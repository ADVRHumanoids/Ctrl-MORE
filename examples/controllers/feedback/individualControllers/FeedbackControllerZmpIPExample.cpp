#include <memory>

#include "locomotion/controllers/feedback/FeedbackController.hpp"
#include "locomotion/controllers/feedback/FeedbackControllersManager.hpp"



using namespace std;
using namespace Eigen;

int main()
{
    //We initialize the feedback controllers manager with configuration file 
    
    string fdbkConfFiles = "/home/przemek/src/robotology-superbuild/build/robots/walking/inputs/settings_walkman_simulation.yaml";
    Locomotion::FeedbackControllersManager feedbackControllers(fdbkConfFiles); 
    feedbackControllers.enable("ZmpIP");
    
    // This is just a temporary variable. It should be filled in by the user. 
    // For content refer to documentation Locomotion::RobotState
    Locomotion::RobotState robotState;
    
    // This is just a temporary variable. It should be filled in by the locomotion 
    // controller. For details refer to documentation Locomotion::ControlState
    Locomotion::ControlState controlState, controlState_mod; 
    
    
    
    
    static Vector3d comCtrlPelvisCorr;
    

    VectorXd com(9);
    com=VectorXd::Zero(9);
    robotState.setEstimatedData("zmp", Eigen::Vector3d::Zero(), 0);
    cout<<"com" << com <<"    "<<endl;
//     cout<<"estCOM "<< m_estCOMpos_glob << endl << endl;
    
//    com.segment(3,0) = Vector3d::Zero();//m_estCOMpos_glob;
//    com.segment(3,3) = Vector3d::Zero();
//    com.segment(3,6) = Vector3d::Zero();
//     cout<<"new estCOM "<<m_estCOMpos_glob<<endl<<endl;
//     robotState.setEstimatedData("CoM", com, 0);


    controlState.setReferencePose("pelvis", Eigen::VectorXd::Zero(3));
    controlState.setCOM(Eigen::VectorXd::Zero(3));

    /// before update controller
    /// the modified states takes the present ones
    controlState_mod=controlState;

    
    
    feedbackControllers.update(robotState, controlState, &controlState_mod);

//     m_presentPelvRef_mod = controlState_mod.getReferencePose("pelvis");

//     m_presentCOMRef_mod = controlState_mod.getCOM();
    
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
