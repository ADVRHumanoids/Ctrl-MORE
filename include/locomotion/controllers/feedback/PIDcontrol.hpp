/**
 * License HERE
*/

/** \file       PIDcontrol.hpp
 * \brief       This is the base class for the PID controllers in locomotion
 *
 * \authors     Juan Alejandro Castano (juan.castano@iit.it) and Przemyslaw Kryczka (przemyslaw.kryczka@iit.it)
 * \date        2016
 * \version     1.0
 * \copyright   GNU Public License
*/

#ifndef LOCOMOTION_PIDcontrol_HPP_
#define LOCOMOTION_PIDcontrol_HPP_

/// first, include standard libraries

/// second, include other libraries like Eigen, Boost,...
#include <Eigen/Dense> 

/// \namespace Locomotion
/// Global namespace fot the Locomotion project, for more info read
///         " main.cpp"

#include "locomotion/utils/signalFilters/SignalFilters.hpp"

namespace Locomotion {

/// \note this header and future implementations can takes directly
///          Przemyslaw Kryczka code
class PIDcontrol {

    
    double m_prev_error;    //!< Previous error
    double m_err;           //!< Present error 
    double m_err_d;         //!< Error derivative 
    double m_err_i;         //!< Integral of an error 

    
    double m_dt;            //!< Sampling time
    double m_kp;            //!< Proportional gain
    double m_ki;            //!< Integral gain 
    double m_kd;            //!< Derivative gain 

    
    double m_bound_plus;    //!< Lower stauration bound
    double m_bound_minus;   //!< Upper stauration bound
    
    double m_output;        //!< Control effort
    
    bool m_doUseErrorLPF;       //!< Enables low pass filter on the signal error
    bool m_doUseErrorDiffLPF;   //!< Enables low pass filter on derivative of an error 
    bool doUpdateDerivative;

    double m_errorLpfFreq;
    double m_errorDtLpfFreq;

    SignalFilters m_errorLpf;
    SignalFilters m_errorDtLpf;

public:
    /// \brief Contructor with defalut set of gains and outbut bounds 
    /// \param kp Proportional gain
    /// \param ki integral gain
    /// \param kd Differential gain
    /// \param dt Sampling time [s]
    /// \param lowerBound Lower bound
    /// \param upperBound Upper bound
    PIDcontrol(double kp, double ki, double kd, double dt, double lowerBound, double upperBound);

    PIDcontrol();
    virtual ~PIDcontrol();

    /// \brief Permits to modified the gains set from single double variables
    /// \param Kp_in    Proportional gain
    /// \param Ki_in    integral gain
    /// \param Kd_in    Differential gain
    void setGains(double kp, double ki, double kd);

    /// \brief Set gains with Eigen vector
    /// \param newGains vector with (0) proportional gain
    ///                             (1) integral gain
    ///                             (2) differential gain
    void setGains(Eigen::VectorXd newGains) ;

    /// \brief Set the saturation bounds 
    /// \param[in] lowerBound   upper bound
    /// \param[in] upperBound   lower bound
    void setBounds(double lowerBound, double upperBound);

    /// \brief Set control effor bounds with vector 
    /// \param[in] bounds Vector that contains the lower and upper bounds consecutively
    void setBounds(Eigen::Vector2d bounds);

    /// \brief Updates the normal, integral and derivative errors
    /// \param[in] error Measured error
    void setError(double error);

    /// \brief Set error LPF cut off frequency and enable the filter 
    /// \param[in] frequency Cut off frequency [Hz]
    void setErrorLPFCutOffFreqency(int frequency);
    
    /// \brief Set error derivative LPF cut off frequency and enable the filter 
    /// \param[in] frequency Cut off frequency [Hz]
    void setDerivativeLPFCutOffFreq(int frequency);
    
    /// \brief Returns last control effort. 
    ///
    /// Virtual function with standard
    /// \f$ m_{output} = Kp*err + Kd*err_d + Ki*err_i; \f$
    /// output. Is set as virtual to let the possibility to use
    /// anti windups or other saturation methods
    /// \param[in] error measure error
    /// \return Control Effort
    virtual double getOutput(double error);

    virtual void init();


    void setKp(double gain);
    void setKi(double gain);
    void setKd(double gain);
    double getKp();
    double getKi();
    double getKd();
};

} 

#endif
