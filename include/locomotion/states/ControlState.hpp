/**
 * License HERE
*/

/** \file       ControlState.hpp
 * \brief       This is a brief description (1 sentence) of the template class.
 * \details     This is a more detailed description of the class:
 *              what it does, how it works, etc.
 * \authors     Chengxu Zhou (chengxu.zhou@iit.it) and Przemyslaw Kryczka (przemyslaw.kryczka@iit.it)
 * \date        2016
 * \version     1.0
 * \copyright   GNU Public License
*/

#ifndef LOCOMOTION_CONTROLSTATE_HPP_
#define LOCOMOTION_CONTROLSTATE_HPP_


// first, include standard libraries
#include <unordered_map>

// second, include other libraries like Eigen, Boost,...
#include <Eigen/Dense>
#include<locomotion/utils/utils.hpp>
#include <locomotion/utils/Logger.hpp>
namespace Locomotion {



/// \class ControlState ControlState.hpp locomotion/ControlState.hpp
///
/// \brief This is a container for control variables to be passed between 
/// locomotion controllers, feedback controllers and the end user.   
///
/// The minimum set of task space control states should be: 
/// \li lAnkle, 
/// \li rAnkle. 
///     
/// Other recommended names of task references: 
/// \li pelvis 
/// \li lWrist 
/// \li rWrist 
/// 
/// 
/// 
///     
class ControlState {

public:


    enum controlMode {
        /// \brief Task space control mode, only task space references should be used.
        task,   
        /// \brief Joint space position control mode, only the jointspace position references should be used.
        jointPosition,
        /// \brief Joint space torque control mode, only the jointspace torque references should be used.
        jointTorque   
    };

    /// \brief Basic constructor
    ControlState();

    /// \brief Copy constructor
    ControlState(ControlState& other);

    /// \brief Destructor. If your class has one virtual function,
    /// the destructor should also be virtual.
    virtual ~ControlState();

    /// \brief Checks weather specified task space reference exists 
    /// \param[in] name Name of the task space reference 
    bool referenceExists(const std::string& name) const;
    
    
    /// \brief Get  a 7x1 pose vector with id name
    /// \param[in] name Name of the task space reference 
    /// \return 7x1 vector representing a task space pose. 
    /// Components 1:3 correspond to x, y, z position of the coordinate frame 
    /// (CF) in the world inertial CF, while components 4-7 correspond to 
    /// the quaternion (x, y, z, w) describing orientation of the CF in the world CF.  
    /// \warning DEPRICATED - not RT safe! 
    Eigen::VectorXd getReferencePose(const std::string& name);
    /// \brief Add reference pose - RT safe
    bool getReferencePose(const std::string& name, Eigen::VectorXd *pose) const;
    
    /// \brief Get  a 6x1 vector containing the linea and angular velocities
    /// \param[in] name Name of the task space reference 
    /// \return 6x1 vector representing a task space velocities. 
    /// Components 1:3 correspond to x, y, z velocities expressed in the world 
    /// inertial CF, while components 4-6 correspond to the angular velocity of 
    /// the body CF expressed as ???.
    /// \warning DEPRICATED - not RT safe! 
    Eigen::VectorXd getReferenceVel(const std::string& name) ;
    /// \brief Add reference vel - RT safe
    bool getReferenceVel(const std::string& name, Eigen::VectorXd *vel) const;

    /// \brief Get  a 6x1 vector containing the linea and angular accelerations
    /// \param[in] name Name of the task space reference 
    /// \return 6x1 vector representing a task space accelerations. 
    /// Components 1:3 correspond to x, y, z accelerations expressed in the world 
    /// inertial CF, while components 4-6 correspond to the angular acceleration of 
    /// the body CF expressed as ???.
    /// \warning DEPRICATED - not RT safe! 
    Eigen::VectorXd getReferenceAcc(const std::string& name);
    /// \brief Get reference acc - RT safe
    bool getReferenceAcc(const std::string& name, Eigen::VectorXd *acc) const;
    
    /// \brief Returns a selected joint space reference
    /// \param[in] typeOfReference Descriptor of the kind of reference ('Position', 'Velocity', 'Torque', etc.)
    /// \return data Reference information 
    Eigen::VectorXd getReferenceJoint(const std::string& typeOfReference) const;
    

    /// \brief Get referenoce of non specific type
    /// \param[in] name Name of the task space reference
    /// \param[out] Vector containig the information
    /// \return true if reference exist.

    bool getOtherReference(const std::string& name,Eigen::VectorXd *Info) const;

    /// \brief Returns control step time in [s].
    double getControlStepTime() const;
    
    /// \brief Get control mode.
    controlMode getControlMode() const;
    
    /// \brief This return the member variable IMU.
    Eigen::Matrix3d getPelvisOrientation();

    /// \brief This return the member variable COM.
    inline Eigen::Vector3d getCOM() const { 
        return m_com;
    }

    /// \brief This return the member variable linearMomentum.
    inline Eigen::Vector3d getLinearMomentum() const { 
        return m_linearMomentum;
    }

    /// \brief This return the member variable angularMomentum.
    inline Eigen::Vector3d getAngularMomentum() const { 
        return m_angularMomentum;
    }

    /// \brief This return the member variable ZMP.
    inline Eigen::Vector3d getZMP() const { 
        return m_zmp;
    }




    /// \brief Set reference ZMP to map
    /// \param[in] name Name of the task space reference
    /// \param[in] data 6x1 vector representing a task space velocities.
    /// Components 1:3 correspond to x, y, z velocities expressed in the world
    /// inertial CF, while components 4-6 correspond to the angular velocity of
    /// the body CF expressed as ???.
    void setReferenceZMP(const std::string& name, const Eigen::VectorXd& data);

    /// \brief Set reference pose to map
    /// \param[in] name Name of the task space reference 
    /// \param[in] data 7x1 vector representing a task space pose. 
    /// Components 1:3 correspond to x, y, z position of the coordinate frame 
    /// (CF) in the world inertial CF, while components 4-7 correspond to 
    /// the quaternion (x, y, z, w) describing orientation of the CF in the world CF.  
    bool setReferencePose(const std::string& name, const Eigen::VectorXd& data);
    
    /// \brief Set reference vel to map
    /// \param[in] name Name of the task space reference 
    /// \param[in] data 6x1 vector representing a task space velocities. 
    /// Components 1:3 correspond to x, y, z velocities expressed in the world 
    /// inertial CF, while components 4-6 correspond to the angular velocity of 
    /// the body CF expressed as ???.    
    bool setReferenceVel(const std::string& name, const Eigen::VectorXd& data);

    /// \brief Set reference acc to map
    /// \param[in] name Name of the task space reference 
    /// \param[in] data 6x1 vector representing a task space accelerations. 
    /// Components 1:3 correspond to x, y, z accelerations expressed in the world 
    /// inertial CF, while components 4-6 correspond to the angular acceleration of 
    /// the body CF expressed as ???.
    bool setReferenceAcc(const std::string& name, const Eigen::VectorXd& data);
    
    /// \brief Set the joint space reference
    /// \param[in] typeOfReference Descriptor of the kind of reference ('Position', 'Velocity', 'Torque', etc.)
    /// \param[in] data Reference information 
    void setReferenceJoint(const std::string& typeOfReference, const Eigen::VectorXd& data);

    /// \brief Set reference WITHOUT SPECIFIC TYPE  to a map. i.e MPC references
    /// \param[in] name: Name of the task space reference
    /// \param[in] data: a vector of non-known lengh
    /// \return true if success
    bool setOtherReference(const std::string& name, const Eigen::VectorXd& data);


    /// \brief This set the member variable COM.
    inline void setCOM(const Eigen::Vector3d& ref) { 
        m_com = ref;
    }

    /// \brief This set the member variable linearMomentum.
    inline void setLinearMomentum(const Eigen::Vector3d& ref) { 
        m_linearMomentum = ref;
    }

    /// \brief This set the member variable angularMomentum.
    inline void setAngularMomentum(const Eigen::Vector3d& ref) { 
        m_angularMomentum = ref;
    }

    /// \brief This set the member variable ZMP.
    inline void setZMP(const Eigen::Vector3d& ref) { 
        m_zmp = ref;
    }

    /// \brief This set the member variable IMU using a rotational matrix.
    inline void setPelvisOrientation(const Eigen::Matrix3d& ref) {
        m_PelvisOrientation = ref;
    }

    /// \brief This set the member variable IMU. using a RPY representation
    void setPelvisOrientation(const Eigen::Vector3d& ref);


    /// \brief Set mode for which the references are given. 
    /// 
    /// The locomotion module should use this setting to let the upper classes 
    /// know which of the references should be used. For example homing to be done 
    /// in joint control mode and locomotion done in task space control mode. 
    void setControlMode(const controlMode newCtrlMode);

    /// \brief Stamp the estimated and measure data contained in ContolState
    void Print() const;


protected:

private:

    /// \brief Task space reference pose.
    std::unordered_map<std::string, Eigen::VectorXd> m_taskSpaceReferencePose;

    /// \brief Task space reference vel.
    std::unordered_map<std::string, Eigen::VectorXd> m_taskSpaceReferenceVel;

    /// \brief Task space reference acc.
    std::unordered_map<std::string, Eigen::VectorXd> m_taskSpaceReferenceAcc;
    
    /// \brief Task space reference acc.
    std::unordered_map<std::string, Eigen::VectorXd> m_jointSpaceReference;

    std::unordered_map<std::string, Eigen::VectorXd> m_taskSpaceReferenceZMP;

    std::unordered_map<std::string, Eigen::VectorXd> m_Otherreference;

    /// \brief COM Position.
    Eigen::Vector3d m_com;

    /// \brief Pelvis orientation [rpy].
    Eigen::Matrix3d m_PelvisOrientation;

    /// \brief linear Momentumn.
    Eigen::Vector3d m_linearMomentum;

    /// \brief angular Momentum.
    Eigen::Vector3d m_angularMomentum;

    /// \brief ZMP Position.
    Eigen::Vector3d m_zmp;

    /// \brief control step time.
    double m_dt;
    
    controlMode m_ctrlMode;

    Eigen::VectorXd tmp7vect, tmp6vect; ///<! Used in functions that return value to avoid dynamic allocation of the memory

    Eigen::VectorXd ReturnPose;
    Eigen::VectorXd ReturnVel;

    Eigen::VectorXd ReturnAcc;

};



}

#endif
