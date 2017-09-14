#include "locomotion/controllers/feedback/PIDcontrol.hpp"

namespace Locomotion {

PIDcontrol::PIDcontrol(double kp, double ki, double kd, double dt, double lowerBound, double upperBound)
{
    setGains(kp, ki, kd);
    m_bound_minus = lowerBound;
    m_bound_plus = upperBound;
    m_dt = dt;

    m_errorLpfFreq = 1/m_dt;
    m_errorDtLpfFreq = 1/m_dt;
    std::cout<<"freq" <<m_errorLpfFreq<<" "<<m_dt<<std::endl;
    m_errorLpf.butterworth(m_dt,m_errorLpfFreq/3,1);
    m_errorDtLpf.butterworth(m_dt,m_errorLpfFreq/3,1);

    init();
    
}
PIDcontrol::PIDcontrol()
{

}
PIDcontrol::~PIDcontrol()
{
    
}
void PIDcontrol::init(){
    m_prev_error=m_err=m_err_d=m_err_i=0;
    doUpdateDerivative=1;
    m_output = 0;
}

/// \brief Permits to modified the gains set from single double variables
/// \param Kp_in    Proportional gain
/// \param Ki_in    integral gain
/// \param Kd_in    Differential gain
void PIDcontrol::setGains(double kp, double ki, double kd)
{
    m_kp=kp;
    m_ki=ki;
    m_kd=kd;
    init();
    
}

/// \brief Set gains with Eigen vector
/// \param newGains vector with (0) proportional gain
///                             (1) integral gain
///                             (2) differential gain
void PIDcontrol::setGains(Eigen::VectorXd newGains) 
{
    setGains(newGains(0),newGains(1),newGains(2));
    
}

/// \brief Set the saturation bounds 
/// \param[in] lowerBound   upper bound
/// \param[in] upperBound   lower bound
void PIDcontrol::setBounds(double lowerBound, double upperBound)
{
    m_bound_minus=lowerBound;
    m_bound_plus=upperBound;

}

/// \brief Set control effor bounds with vector 
/// \param[in] bounds Vector that contains the lower and upper bounds consecutively
void PIDcontrol::setBounds(Eigen::Vector2d bounds)
{
    setBounds(bounds(0),bounds(1));
}

/// \brief Updates the normal, integral and derivative errors
/// \param[in] error Measured error
void PIDcontrol::setError(double error)
{
    m_prev_error = error;
    m_err = error;
    if ( doUpdateDerivative ) //Do not update derivative if support foot was changed
        m_err_d = m_errorDtLpf.applyFilter((m_err - m_prev_error)/m_dt);  //Differential with average filter
    m_err_i += m_err;
    
}

/// \brief Set error LPF cut off frequency and enable the filter 
/// \param[in] frequency Cut off frequency [Hz]
void PIDcontrol::setErrorLPFCutOffFreqency(int frequency)
{
    m_errorLpfFreq=frequency;
    m_errorLpf.butterworth(m_dt,frequency,1);
}

/// \brief Set error derivative LPF cut off frequency and enable the filter 
/// \param[in] frequency Cut off frequency [Hz]
void PIDcontrol::setDerivativeLPFCutOffFreq(int frequency)
{
    m_errorDtLpfFreq=frequency;
    m_errorDtLpf.butterworth(m_dt,frequency,1);
}

/// \brief Returns last control effort. 
///
/// Virtual function with standard
/// \f$ m_{output} = Kp*err + Kd*err_d + Ki*err_i; \f$
/// output. Is set as virtual to let the possibility to use
/// anti windups or other saturation methods
/// \param[in] error measure error
/// \return Control Effort
double PIDcontrol::getOutput(double error)
{
    setError(error);
    m_output = m_kp*m_err + m_kd*m_err_d + m_ki*m_err_i;

    if (m_output > m_bound_plus) m_output = m_bound_plus;
    if (m_output < m_bound_minus) m_output = m_bound_minus;

    return m_output;

}


void PIDcontrol::setKp(double gain){
    m_kp=gain;
}
void PIDcontrol::setKi(double gain){
    m_ki=gain;
}
void PIDcontrol::setKd(double gain){
    m_kd=gain;
}

double PIDcontrol::getKp(){

    return m_kp;

}
double PIDcontrol::getKi(){

    return m_ki;

}
double PIDcontrol::getKd(){

    return m_kd;

}
} 
