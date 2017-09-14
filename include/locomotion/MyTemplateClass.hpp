/**
 * License HERE
*/

/** \file       MyTemplateClass.hpp
 * \brief       This is a brief description (1 sentence) of the template class.
 * \details     This is a more detailed description of the class:
 *              what it does, how it works, etc.
 * \authors     Name1 (name1@iit.it), Name2 (name2@iit.it)
 * \date        2016
 * \version     1.0
 * \copyright   GNU Public License
*/

#ifndef PROJECTNAME_MYTEMPLATECLASS_HPP_
#define PROJECTNAME_MYTEMPLATECLASS_HPP_

// Macros
/// \def ABS(x)
/// Computes the absolute value of \a x.
#define ABS(x) (((x)>0)?(x):-(x))

// first, include standard libraries
#include <iostream> 
#include <vector>

// second, include other libraries like Eigen, Boost,...
#include <Eigen/Dense>

// third, include your libraries/headers
// #include "locomotion/path/to/header.hpp"

// GENERAL RULES
// - Do not use: using namespace <namespace>
// - Indent your code with 4 spaces (no tabs!)
// - The name of the class should start with a capital letter

/// \namespace Locomotion
/// Description of the namespace, this should be described only once.
/// So you can probably remove the description.
namespace Locomotion {

/// \class MyTemplateClass MyTemplateClass.hpp locomotion/MyTemplateClass.hpp
///
/// \brief Description of what this class does.
///
/// \example include/locomotion/MyTemplateClass.hpp
/// MyTemplateClass* temp = MyTemplateClass(param)
/// double p = temp.get(param)
class MyTemplateClass {

public:
    /// \typedef ulong
    /// \brief My typedef
    typedef unsigned long ulong;

    /// \enum Color
    /// \brief My enum
    enum Color {
        /// \brief red color
        red,

        /// \brief green color
        green, 

        /// \brief blue color
        blue};

    /// \brief Basic constructor
    MyTemplateClass();

    /// \brief Overloaded constructor
    MyTemplateClass(int x, double y);

    /// \brief Copy constructor
    MyTemplateClass(MyTemplateClass& other);

    /// \brief Destructor. If your class has one virtual function,
    /// the destructor should also be virtual.
    virtual ~MyTemplateClass();

    /// \brief This is a pure virtual function (notice = 0 at the end
    /// of the method). This means that it needs to be implemented in 
    /// the child class.
    /// \param[in]  paramIn  description of the parameter
    /// \param[out] paramOut description of the parameter
    virtual void pureVirtualFunction(int paramIn, int& paramOut) = 0;

    /// \brief This is a virtual function, which by default calls
    /// MyTemplate::myMethod().
    /// \param[in, out]  param  description of the parameter
    /// \throw <exception-object> Exception raised if param is NULL
    /// \return true if the parameter is bigger than 2.
    /// \note Time complexity: O(1)
    /// \see MyTemplate::myMethod
    virtual bool virtualFunction(double *param);

    /// \brief this method does something.
    /// \todo need to correct this method because ...
    /// At the end, there should be no todos in any methods!
    void myMethod();

    /// \brief template method that returns the max value between
    /// 2 given parameters.
    /// \param[in] a the first parameter.
    /// \param[in] b the second parameter.
    /// \return the max value.
    template<typename T>
    T max(const T& a, const T& b);

    /// \brief after defining the above methods, we can finally 
    /// define the set and get methods.
    void setX(int x);

    /// \brief This return the member variable x.
    /// It is an inline function.
    /// There is a const at the end to specify the compiler that we 
    /// do not modify any member variables.
    inline int getX() const {
        return m_x;
    }

    /// \brief my public attribute which doesn't respect encapsulation
    /// \warning NEVER DO THIS!
    float m_badPublicVariable;

protected:
    // put the protected methods and attributes here and respect the 
    // same order as above.
    // Constructors, Destructor, pure virtual methods, virtual methods,
    // methods, setters, getters, operator overloading, member variables.

    /// \brief This method does something.
    void myMethod2();

private:
    // put the private methods and attributes here and respect the 
    // same order as above.
    // Constructors, Destructor, pure virtual methods, virtual methods,
    // methods, setters, getters, operator overloading, member variables.
    
    int m_x;        //!< description of the member variable.

    double m_y;     //!< description of the member variable.

};

// Implements templates here.
template<typename T> 
T MyTemplateClass::max(const T& a, const T& b) {
    return a > b ? a : b;    
}


} 

#endif
