
##########################################################
############## RobotToolKit stuff ########################
##########################################################
##########################################################
########### Add Package info below!#######################
##########################################################
# here you need to list the robot modules and world 
# modules that live in your package, for example:
# set(RobotModules MyAwsomeRobotModule1 MyAwsomeRobotModule2 ...)
# set(WorldModules MyAwsomeWorldModule1 MyAwesomeWorldModule2 ...)
set(RobotModules SimAllegroInterface)

set(WorldModules )

# if any of your modules depend on libraries in other rospackages,
# you define this in the manifest.xml file, and rosbuild will 
# make sure everything is properly linked. 

# If you want to, you can have libraries inside your package
# but then you have to declare them here.
# example: set(PackageLibraries MyAwesomeLibrary1 MyAwesomeLibrary2 ...)
# NOTE: if you libB depends on libA, you must 
# set them in that order: set(PackageLibraries libA libB)

set(PackageLibraries MultiFingersObjImpController AllegroHandObjImpController)

# all modules in your package will be linked to these libraries. 
# also a library links to any other library specified to the left of it

# if you want too see some warnings when compiling you may comment/change 
# this line
set(ROS_COMPILE_FLAGS " ")

##########################################################
##### That's it! That is all you have to do. #############
########## Now leave this file please... #################
##########################################################


##########################################################
############## rosbuild stuff ############################
##########################################################

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

include_directories(${PROJECT_SOURCE_DIR}/include)


##########################################################
############## RobotToolKit stuff, ctd.. #################
##########################################################


#find path to RobotToolKit stack..
#just to get the path to some package that we know excatly where it is..
rosbuild_find_ros_package(RTK_pkg_tools)
# this file will take care of the rest. You can of course remove this
# and write your own CMakeLists.txt if you prefer.
include(${RTK_pkg_tools_PACKAGE_PATH}/RTKbuild_package.cmake)
rosbuild_add_executable(ControlTest ControlTest/src/ControlTest.cpp)
rosbuild_add_executable(VirtualFrameRecordExtractor AllegroHandObjImpController/src/VirtualFrameRecordExtractor.cpp)
rosbuild_add_executable(SynTouchDataExtractor Utilities/src/BulbDemoRecordExtractor.cpp)
rosbuild_add_executable(ObjImpControllerNode ObjImpControllerNode/src/main.cpp)
target_link_libraries(ControlTest AllegroHandObjImpController)
target_link_libraries(ObjImpControllerNode AllegroHandObjImpController)

