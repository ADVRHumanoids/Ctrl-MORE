#include <locomotion/utils/utils.hpp>

namespace Locomotion {
Eigen::Matrix3d rotx(double angle)
{
    //Matrix defined as stattic to accelerate calculation
    Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
    double ct;
    double st;

    ct = cos(angle);
    st = sin(angle);

    result(1,1) = ct;
    result(1,2) = -st;
    result(2,1) = st;
    result(2,2) = ct;

    return result;
}

Eigen::Matrix3d roty(double angle)
{
    //Matrix defined as stattic to accelerate calculation
    Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
    double ct;
    double st;

    ct = cos(angle);
    st = sin(angle);

    result(0,0) = ct;
    result(0,2) = st;
    result(2,0) = -st;
    result(2,2) = ct;

    return result;
}

Eigen::Matrix3d rotz(double angle)
{
    //Matrix defined as stattic to accelerate calculation
    Eigen::Matrix3d result = Eigen::Matrix3d::Identity();
    double ct;
    double st;

    ct = cos(angle);
    st = sin(angle);

    result(0,0) = ct;
    result(0,1) = -st;
    result(1,0) = st;
    result(1,1) = ct;

    return result;
}

Eigen::Matrix3d rpy2rotKDL(const Eigen::Vector3d &in)
{
    Eigen::Matrix3d  rot_out=Eigen::Matrix3d::Identity();
    
	//rot_out = rot_out*rotz(in(2));
    //rot_out = rot_out*roty(in(1));
    //rot_out = rot_out * rotx(in(0));
	std::cout<<"mmm"<<std::endl<<rot_out<<std::endl;

    return rot_out;

}

Eigen::Vector3d skewVel2velVector(Eigen::Matrix3d &skewVelMatrix)
{
    Eigen::Vector3d angularVelocity;
    angularVelocity(0) = skewVelMatrix(2,1);
    angularVelocity(1) = skewVelMatrix(0,2);
    angularVelocity(2) = skewVelMatrix(1,0);
    return angularVelocity;
}

Eigen::Matrix3d InterpOfRot(double t0, double te, double t, Eigen::Matrix3d Rstart, Eigen::Matrix3d Rend)
{
    Eigen::Matrix3d OutRotation;
    Eigen::Matrix3d Rs2e, RodriguesMatrix;
    Eigen::Vector3d axis;
    double angle,anglet;

    Rs2e=Rstart.transpose()*Rend;

    double tmp = (Rs2e(0,0)+Rs2e(1,1)+Rs2e(2,2)-1.0)*0.5;

    if (tmp > 1.0) tmp = 1.0;
    if (tmp < -1.0) tmp = -1.0;

    angle=acos(tmp);

    if (angle<1e-3)
    {
        OutRotation=Rend;
    }
    else
    {
        axis<<0.5/sin(angle)*(Rs2e(2,1)-Rs2e(1,2)), 0.5/sin(angle)*(Rs2e(0,2)-Rs2e(2,0)), 0.5/sin(angle)*(Rs2e(1,0)-Rs2e(0,1));
        anglet=angle/(te-t0)*(t-t0);
        RodriguesMatrix=rodriguesRot(axis,anglet);

        OutRotation=Rstart*RodriguesMatrix;
    }

    return OutRotation;
}

Eigen::Vector3d InterpOfRotRPY(double t0, double te, double t, Eigen::Matrix3d Rstart, Eigen::Matrix3d Rend)
{
    Eigen::Matrix3d OutRotation;
    Eigen::Matrix3d Rs2e, RodriguesMatrix;
    Eigen::Vector3d axis;
    double angle,anglet;

    Rs2e=Rstart.transpose()*Rend;

    double tmp = (Rs2e(0,0)+Rs2e(1,1)+Rs2e(2,2)-1.0)*0.5;

    if (tmp > 1.0) tmp = 1.0;
    if (tmp < -1.0) tmp = -1.0;

    angle=acos(tmp);

    if (angle<1e-3)
    {
        OutRotation=Rend;
    }
    else
    {
        axis<<0.5/sin(angle)*(Rs2e(2,1)-Rs2e(1,2)), 0.5/sin(angle)*(Rs2e(0,2)-Rs2e(2,0)), 0.5/sin(angle)*(Rs2e(1,0)-Rs2e(0,1));
        anglet=angle/(te-t0)*(t-t0);
        RodriguesMatrix=rodriguesRot(axis,anglet);

        OutRotation=Rstart*RodriguesMatrix;
    }

    return rot2rpyKDL(OutRotation);
}
double Poly3(double time,double t_0, double t_end, double startPos,double endPos){
    if (t_end==t_0){
        return startPos;
    }

    else if (time<t_0 ){
        return startPos;
    }
    else if (time>t_end){
        return endPos;
    }
    else{
        double a0=startPos;
        double a2=3/pow((t_end-t_0),2)*(endPos-startPos);
        double a3=-2/pow((t_end-t_0),3)*(endPos-startPos);
        double t=time-t_0;
        return a0+a2*pow(t,2)+a3*pow(t,3);
    }
}

Eigen::Matrix3d rodriguesRot(Eigen::Vector3d w,double theta)
{
    //w should be column vector of size 3, unit vector
    //you can either specify w and dt or directly use theta=w*dt to get
    //rotational angle
    Eigen::Vector3d wn;
    Eigen::Matrix3d w_wedge, R, Reye;
    Reye = Eigen::Matrix3d::Identity(3,3);
    if (w.norm()<1e-6)
    {
        wn<<0, 0, 0;
    }
    else
    {
        wn = w/w.norm();// normarized vector
    }
    w_wedge <<0, -wn(2), wn(1),
                wn(2), 0, -wn(0),
                -wn(1), wn(0), 0;

    R = Reye + w_wedge*sin(theta) + w_wedge*w_wedge*(1-cos(theta));
    return R;
    // the rotational operation around the vector w
}

Eigen::Vector3d rot2rpyKDL(const Eigen::Matrix3d &M)
{
    Eigen::Vector3d res;
    KDL::Rotation rot;

    rot = eigenRot2KDL(M);

    rot.GetRPY(res(0), res(1), res(2));

    return res;
}

KDL::Rotation eigenRot2KDL(Eigen::Matrix3d matrix) {
    KDL::Rotation rot_kdl;
    for (int i=0; i<matrix.rows(); i++ ) {
        for (int j=0; j<matrix.cols(); j++)
          rot_kdl(i,j) = matrix(i,j) ;

    }

    return rot_kdl;
}

bool computeAbsolutePath (  const std::string& input_path,
                            const std::string& middle_path,
                            std::string& absolute_path)
{
    // if not an absolute path
    if(!(input_path.at(0) == '/')) {
        // if you are working with the Robotology Superbuild
        const char* env_p = std::getenv("ROBOTOLOGY_ROOT");
        // check the env, otherwise error
        if(env_p) {
            std::string current_path(env_p);
            // default relative path when working with the superbuild
            current_path += middle_path;
            current_path += input_path;
            absolute_path = current_path;
            return true;
        }
        else {
            std::cerr << "ERROR in " << __func__ << " : the input path  " << input_path << " is neither an absolute path nor related with the robotology superbuild. Download it!" << std::endl;
            return false;
        }
    }
    // already an absolute path
    absolute_path = input_path;
    return true;
}

void KDL2Eigen(KDL::Frame frame, Eigen::Vector3d& Pos, Eigen::Matrix3d& Rot){
	for (int i=0; i<3; i++ ) {
		Pos(i)=frame.p(i);
        for (int j=0; j<3; j++)
          Rot(i,j)=frame.M(i,j);

    }
	
}
}
