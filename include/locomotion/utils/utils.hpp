/**
 * License HERE
*/

/** \file       SignalFilters.hpp
 * \brief       This Class contains a set of basic modicable filters for single
 *              signals
 *
 *
 * \details     The user can modify kind of filter, cutoff frequency and is used only
 *              for SISO signals. Vectors and matrix are not supported
 *
 *
 * \authors     Juan Alejandro Castano (juan.castano@iit.it)
 * \date        2016
 * \version     1.0

 * \copyright   GNU Public License
*/

#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>

#include <kdl/kdl.hpp>
#include <kdl/rotational_interpolation.hpp>

#include <string>
#include <stdlib.h> //GetEnv
namespace Locomotion {
static Eigen::MatrixXd MytimeMatrix;
//Eigen::Matrix<double,6,1> coeffsAux;
Eigen::Matrix3d rotx(double angle);
Eigen::Matrix3d roty(double angle);
Eigen::Matrix3d rotz(double angle);
Eigen::Matrix3d rpy2rotKDL(const Eigen::Vector3d &in);
Eigen::Matrix3d rpy2rotEigen(const Eigen::Vector3d &in);
Eigen::Vector3d skewVel2velVector(Eigen::Matrix3d &skewVelMatrix);
Eigen::Matrix3d InterpOfRot(double t0, double te, double t, Eigen::Matrix3d Rstart, Eigen::Matrix3d Rend);
Eigen::Vector3d InterpOfRotRPY(double t0, double te, double t, Eigen::Matrix3d Rstart, Eigen::Matrix3d Rend);
Eigen::Vector3d rot2rpyKDL(const Eigen::Matrix3d &M);
Eigen::Matrix3d rodriguesRot(Eigen::Vector3d w,double theta);
double Poly3(double time,double t_0, double t_end, double startPos,double endPos);
KDL::Rotation eigenRot2KDL(Eigen::Matrix3d rot);
void KDL2Eigen(KDL::Frame frame, Eigen::Vector3d& Pos, Eigen::Matrix3d& Rot);
inline bool Mypolynomial_interpolation(int tDuration, double startPos, double startVel, double startAcc, double endPos, double endVel, double endAcc, Eigen::VectorXd *position)
{



    Eigen::VectorXd coeffsAux(6);
    tDuration--;
    coeffsAux(0) = -0.5*(12*startPos-12*endPos+6*startVel*tDuration+6*endVel*tDuration+startAcc*pow((double)tDuration,2)-endAcc*pow((double)tDuration,2))/pow((double)tDuration,5);
    coeffsAux(1) = 0.5*(30*startPos-30*endPos+16*startVel*tDuration+14*endVel*tDuration+3*startAcc*pow((double)tDuration,2)-2*endAcc*pow((double)tDuration,2))/pow((double)tDuration,4);
    coeffsAux(2) = -0.5*(20*startPos-20*endPos+12*startVel*tDuration+8*endVel*tDuration+3*startAcc*pow((double)tDuration,2)-endAcc*pow((double)tDuration,2))/pow((double)tDuration,3);
    coeffsAux(3) = 0.5*startAcc;
    coeffsAux(4) = startVel;
    coeffsAux(5) = startPos;
    tDuration++;
    std::cout<<tDuration<<std::endl;
    // cout << "Coeficients" << endl << coeffsAux << endl;
    position->resize(tDuration);
    (*position) = 	coeffsAux(0) * MytimeMatrix.block(0,4,tDuration,1).array() +
                    coeffsAux(1) * MytimeMatrix.block(0,3,tDuration,1).array() +
                    coeffsAux(2) * MytimeMatrix.block(0,2,tDuration,1).array() +
                    coeffsAux(3) * MytimeMatrix.block(0,1,tDuration,1).array() +
                    coeffsAux(4) * MytimeMatrix.block(0,0,tDuration,1).array() +
                    coeffsAux(5);

    return true;
}

bool computeAbsolutePath (  const std::string& input_path,
                            const std::string& middle_path,
                            std::string& absolute_path);
}
#endif // UTILS_HPP
