# 
#  Copyright (C) 2014 Walkman
#  Author: Przemyslaw Kryczka
#  email:  przemyslaw.kryczka@iit.it
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program. If not, see <http://www.gnu.org/licenses/>
#

# ------------------------------------------------------------------------------
#                       CMAKE CONFIGURATION
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 2.8.12)

#configure default policies 
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW) #Don't split absolut path of library into search path and library name
    cmake_policy(SET CMP0005 NEW) 
    cmake_policy(SET CMP0017 NEW) #Prefer files from the CMake module directory when including from there.
endif(COMMAND cmake_policy)

#Configure build type 
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo
MinSizeRel."
       FORCE )
endif()

project(locomotion)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

#Set basic make flags 
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreturn-type") #Show warning if function does not return a value 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -pedantic")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")  #Flag necessary to use object library when global variables are present

set(BUILD_PATH "${CMAKE_CURRENT_BINARY_DIR}") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")  #Flag necessary to use object library when global variables are present
# ------------------------------------------------------------------------------
#                       PROJECT RELATED
# ------------------------------------------------------------------------------



FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)
FIND_PACKAGE(XCM REQUIRED)

add_definitions(-DVERBOSE)

# all source files names 
# FILE(GLOB main_source        examples/controllers/feedback/FeedbackControllersManagerExample.cpp)
# FILE(GLOB main_source        examples/controllers/feedback/individualControllers/FeedbackControllerZmpIPExample.cpp)
FILE(GLOB main_source        src/main.cpp)

FILE(GLOB locomotion_source  src/Locomotor.cpp
                             src/locomotion/*.cpp
                             )

FILE(GLOB controllers_source src/controllers/*.cpp
                             src/controllers/feedback/PIDcontrol.cpp
                             src/controllers/feedback/FeedbackController.cpp
                             src/controllers/feedback/FeedbackControllersManager.cpp
                             src/controllers/feedback/CoMPID/*.cpp
                            src/controllers/feedback/ControlExample/*.cpp
                            )

FILE(GLOB states_source      src/states/*.cpp)
FILE(GLOB utils_source       src/utils/*.cpp
                             src/utils/signalFilters/*.cpp)
 
# all header files names 
FILE(GLOB locomotion_headers  include/locomotion/*.hpp
                              include/locomotion/LocomotionController.hpp
                              include/locomotion/LocomotionControllerManager.hpp
)



FILE(GLOB controllers_headers include/locomotion/controllers/feedback/*.hpp
                              include/locomotion/controllers/feedback/CoMPID/*.hpp
                              )
FILE(GLOB states_headers      include/locomotion/states/*.hpp)
FILE(GLOB utils_headers       include/locomotion/utils/*.hpp
                              include/locomotion/utils/signalFilters/*.hpp
                              )
FILE(GLOB main_header         include/*.hpp*)
FILE(GLOB_RECURSE main_header include/ *.hpp*)

# add include directories
INCLUDE_DIRECTORIES(include
                    ${EIGEN3_INCLUDE_DIR}
                    ${orocos_kdl_DIRS}
                    ${XCM_INCLUDE_DIRS}
                    ) 

SET(LIBRARIES
    stdc++
    pthread 
    dl
    yaml-cpp 
    ${XCM_LIBRARIES}
) 


                      
SET(HEADER_FILES 
    ${locomotion_headers}
    ${controllers_headers}
    ${states_headers}
    ${utils_headers}
    )               

add_library(locomotionObjects OBJECT ${locomotion_source}
                ${controllers_source} 
                ${states_source}                
                ${utils_source}
                )
    
ADD_EXECUTABLE( ${PROJECT_NAME} 
                ${main_source}                
                ${main_header}
                $<TARGET_OBJECTS:locomotionObjects> 
                )  
                
TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
                      ${LIBRARIES}
                      ${orocos_kdl_LIBRARIES}
                      )
                    
# set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")


add_library(fbkCtrlzmpip  SHARED ${controllers_source}
                                 src/controllers/feedback/FeedbackControllerZmpIP.cpp
                                 src/controllers/feedback/PIDcontrol.cpp
                                 ${utils_source}
                                 ${states_source})

add_library(fbkCtrlExample  SHARED ${controllers_source}
                                   src/controllers/feedback/ControlExample/ControlExample.cpp
                                   src/controllers/feedback/PIDcontrol.cpp
                                   ${utils_source}
                                   ${states_source})

add_library(fbkCtrlCoMPID  SHARED ${controllers_source}
                                   src/controllers/feedback/CoMPID/CoMPIDcontroller.cpp
                                   src/controllers/feedback/PIDcontrol.cpp
                                   ${utils_source}
                                   ${states_source})

########################################################################
# use YCM to export locomotion_lib so that it can be found using find_package #
########################################################################
add_library(locomotion_lib SHARED $<TARGET_OBJECTS:locomotionObjects>)
TARGET_LINK_LIBRARIES(locomotion_lib
                      ${LIBRARIES}   
                      )

set(VARS_PREFIX "locomotion_lib")

set(locomotion_lib_MAJOR_VERSION 0)
set(locomotion_lib_MINOR_VERSION 0)
set(locomotion_lib_PATCH_VERSION 1)
set(locomotion_lib_VERSION ${locomotion_lib_MAJOR_VERSION}.${locomotion_lib_MINOR_VERSION}.${locomotion_lib_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

target_include_directories(locomotion_lib PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
                                      "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set_target_properties(locomotion_lib   PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                            SOVERSION ${${VARS_PREFIX}_VERSION})

install(DIRECTORY include/ 
        DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}"
        FILES_MATCHING PATTERN "*.h*")

install(TARGETS locomotion_lib  
        EXPORT locomotion_lib
#         ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
#         RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib)

install(TARGETS fbkCtrlCoMPID
        EXPORT fbkCtrlCoMPID
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}/locomotion" COMPONENT shlib)

install(TARGETS fbkCtrlzmpip
        EXPORT fbkCtrlzmpip
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}/locomotion" COMPONENT shlib)

#enabling it will add all locomotion_lib dependencies as dependencies for third party users
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS locomotion_lib)


include(InstallBasicPackageFiles)
install_basic_package_files(locomotion_lib     VARS_PREFIX ${VARS_PREFIX}
                                    VERSION ${${VARS_PREFIX}_VERSION}
                                    COMPATIBILITY SameMajorVersion
                                    TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                    NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)

                                 
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

