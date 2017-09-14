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

#ifndef SIGNALFILTERS_HPP
#define SIGNALFILTERS_HPP

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
/// second, include other libraries like Eigen, Boost,...
///#include </usr/include/eigen3/Eigen/Dense> ///To change according to the EIGEN path

/// \def Maximum legnght for the filter
#define MAX_FILTER_LENGTH 8

/// \namespace Locomotion
/// Global namespace fot the Locomotion project, for more info read
///         " main.cpp"
using namespace Eigen;

namespace Locomotion {

    class SignalFilters {
    public:
        /// \brief Default Constructor/Destructor
        SignalFilters();
        ~SignalFilters();

        /// \remarks Use on loop functions

        ///
        /// \brief applyFilter: This function returns the filter value of the signal X
        /// \param [in] X desired measure or variable to be filter
        /// \return Filter value of X given the filter initialization and past values
        ///
        double applyFilter(double X);

        ///
        /// \brief differentiator: This is the butterworth differentiator function
        /// \param [in] X desired measure or variable to be differentiated
        /// \return differentiated value of X given the initialization and past values
        ///
        double differentiator(double X);

        /// \remarks Use on the initialization
        ///

        ///
        /// \brief clear_filter: set the memory of the filter and it's
        ///                      coefficients to 0
        ///
        void clear_filter();

        /// \remarks Filter initialization:
        ///                                 cutoff is the cutoff frequency in Hz
        ///                                 N is the order of the filter

        ///
        /// \brief least_squares_filter: Initialize the internal parameters of the filter
        /// \param T[in]: is the sampleTime in second
        /// \param N[in]: is the order of the filter default 2, possibles 4 and 8
        ///
        void least_squares_filter(double T, int N);

        ///
        /// \brief moving_average_filter:Initialize the internal parameters of the filter
        /// \param N[in]: is the order of the filter default 2, possibles 4 and 8
        ///
        void moving_average_filter(int N);

        ///
        /// \brief butterworth:Initialize the internal parameters of the filter
        /// \param T[in]: is the sampleTime in second
        /// \param cutoff[in]: is the cutoff frequency in Hz
        /// \param N[in]: is the order of the filter default 1, possibles 1-4
        ///
        void butterworth(double T, double cutoff, int N);

        ///
        /// \brief butterDifferentiator:Initialize the internal parameters of the filter
        /// \param T[in]: is the sampleTime in second
        /// \param cutoff[in]: is the cutoff frequency in Hz
        /// \param N[in]: is the order of the filter default 1, possibles 1-4
        ///
        void butterDifferentiator(double T, double cutoff, int N);

        ///
        /// \brief initialValue: This function set the internal memory of the filter to the
        ///                      desired initialState
        /// \param initialState: Desired value to start the filter's memory
        ///
        void initialValue(double initialState);


        //Define Variables
        //************************************************************************************
        /// \brief Ncoeff: Number of coefficients
        int m_Ncoeff;

        /// \brief x Circular buffer of incoming values to be filtered
        double m_x[MAX_FILTER_LENGTH];

        /// \brief y Result and accumulators
        double m_y[MAX_FILTER_LENGTH];

        /// \brief a accomulator Y  coefficients
        double m_a[MAX_FILTER_LENGTH];

        /// \brief b incoming values coefficients
        double m_b[MAX_FILTER_LENGTH];

    };
}
#endif // REACTIVEZMPTRAJECTORY_HPP
