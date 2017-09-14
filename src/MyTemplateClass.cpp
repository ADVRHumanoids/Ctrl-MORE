#include <iostream>

#include <locomotion/MyTemplateClass.hpp>

//don't use "using namespace xxx" 

// definition of the methods in the same order than in the header file
// don't need to rewrite the documentation here

namespace Locomotion {

MyTemplateClass::MyTemplateClass() {    
}

MyTemplateClass::MyTemplateClass(int x, double y) : 
    m_x(x),
    m_y(y) {
        std::cout << "Hello World" << std::endl;
}

MyTemplateClass::MyTemplateClass(MyTemplateClass& other) {
    m_x = other.getX();
    m_y = 2.0;
}

MyTemplateClass::~MyTemplateClass() {
    // Free the allocated memory here.
    // If you used the 'new' operator use the 'delete' operator here.
    std::cout << "Destruction!!" << std::endl;
}

// We don't implement the pure virtual method here
// It is the child class that should implement it.

bool MyTemplateClass::virtualFunction(double *param) {
    if (param == NULL) {
        //throw MyException("NULL parameter given...");
    }

    // don't hesitate to comment your code!
    if (*param <= 2.0) {
        return false;
    }
    
    return true;
}

void MyTemplateClass::myMethod() {
    std::cout << "foo" << std::endl;
}

void MyTemplateClass::setX(int x) {
    m_x = x;
}

void MyTemplateClass::myMethod2() {
    //Comment what the for is doing 
    for (int i=0; i<10; ++i) {
        std::cout << i << " ";
    }
    
    //Comment what the for is doing 
    for (int i = 10; i < 20; ++i) {
        std::cout << i << " ";
    }
    
    std::cout << std::endl;
}

}