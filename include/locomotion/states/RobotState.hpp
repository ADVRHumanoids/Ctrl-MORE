/**
 * License HERE
*/

/** \file       RobotState.hpp
 * \brief       This is a brief description (1 sentence) of the template class.
 * \details     This is a more detailed description of the class:
 *              what it does, how it works, etc.
 * \authors     Chengxu Zhou (chengxu.zhou@iit.it) and Przemyslaw Kryczka (przemyslaw.kryczka@iit.it)
 * \date        2016
 * \version     1.0
 * \copyright   GNU Public License
*/

#ifndef LOCOMOTION_ROBOTSTATE_HPP_
#define LOCOMOTION_ROBOTSTATE_HPP_


// first, include standard libraries
#include <unordered_map>

// second, include other libraries like Eigen, Boost,...
#include <Eigen/Dense>
#include <locomotion/utils/Logger.hpp>

namespace Locomotion {



/// \class RobotState RobotState.hpp locomotion/RobotState.hpp
///
/// \brief Class contains all measured and estimated data related to the robot. 
///
/// The minimum set of data that has to be set by the user in measuredData: 
/// \li 'jointPosition' - joint position in [rad]
/// \li 'jointTorque' - joint torque [Nm]
/// \li 'FT_xx' - vector 6x1 containing force [N] and torque [Nm] data. xx should be substituted by description of the data (e.g. 'L', 'R' in case of a biped) 
/// \li 'IMU_xx' - imu data 10x1 containing orientation (quaternion ??specify order of the data here??), accelerometer [m/s^2], gyroscope [rad/s] data.
/// \li ''
/// 
/// Example of data that can be set and names that should be used in estimatedData:
/// \li 'zmp' - 3x1 vector containing (x, y, z) position of ZMP. 
/// \li 'com' - 9x1 vector containing estimated information about pos, vel, acc of the com.
/// \li 'waistRot' - 4x1 quaternion describing estimated orientation of the waist.
///     
/// \example include/locomotion/RobotState.hpp
/// RobotState* temp = RobotState(param)
/// double p = temp.get(param)
class RobotState {

public:

    /// \enum Support State
    /// \brief Support State
    enum ContactState {
        Nowhere,    // 0
        InAir,      // 1
        OnRight,    // 2
        OnBoth,     // 3
        OnLeft,     // 4
    };


    /// \brief Basic constructor
    RobotState();

    /// \brief Copy constructor
    RobotState(RobotState& other);

    /// \brief Destructor.
    ~RobotState();

    /// \brief add measured data to map
    void setMeasuredData(const std::string& name, const Eigen::VectorXd& data, const unsigned int timeStamp);

    /// \brief get measured data to map
    Eigen::VectorXd getMeasuredData(const std::string& name) const;
    
    /// \brief get measured data to map
    Eigen::VectorXd getMeasuredDataTimed(const std::string& name, unsigned int &timestamp) const;

    
    /// \brief add measured data to map
    void setEstimatedData(const std::string& name, const Eigen::VectorXd& data, const unsigned int timeStamp);

    /// \brief get measured data to map
    Eigen::VectorXd getEstimatedData(const std::string& name) const;
    
    /// \brief get measured data to map
    Eigen::VectorXd getEstimatedDataTimed(const std::string& name, unsigned int &timestamp) const;
    
    /// \brief Stamp the estimated and measure data contained in RobotState
    void Print() const;

protected:

private:

    /// \brief measured data.
    std::unordered_map<std::string, Eigen::VectorXd>    m_measuredData; //<! map of measured data
    std::unordered_map<std::string, unsigned int>       m_measurementTimeStamp; //<! time stamp for measured data
    
    /// \brief estimated data.
    std::unordered_map<std::string, Eigen::VectorXd>    m_estimatedData; //<! Map of stimated data
    std::unordered_map<std::string, unsigned int>       m_estimateTimeStamp;//<! stimated data time stamp

};



}

#endif
