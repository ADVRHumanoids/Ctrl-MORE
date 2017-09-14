#include <iostream>
#include <boost/concept_check.hpp>

#include <locomotion/Locomotor.hpp>
#include <locomotion/states/RobotState.hpp>
#include <locomotion/states/ControlState.hpp>

using namespace std;
using namespace Eigen;

int main()
{
   Locomotion::Locomotor loco;
   
   Locomotion::RobotState rState;
   Locomotion::ControlState cState;
   
   VectorXd pose(7);
   
   VectorXd lAnkle(6), rAnkle(6), pelvis(6), wholeBody(31), goalPose(6); 
   lAnkle << 0, 0.14, 0, 0, 0, 0;
   rAnkle << 0,-0.14, 0, 0, 0, 0;
   pelvis << 0, 0, 0.9, 0, 0, 0;
   
     
 
   goalPose << 0.2, 0, 0, 0, 0, 0;
    
     
   for (unsigned int i=0;i<1e10; i++) {
      loco.update(rState, &cState);

//       }
   }
   
   std::cout << "Hello" << std::endl;
   return 0;
}
