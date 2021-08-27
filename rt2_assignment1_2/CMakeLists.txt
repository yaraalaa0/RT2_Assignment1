cmake_minimum_required(VERSION 3.5)
project(rt2_assignment1_2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(assignment_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)



add_library(state_machine_component SHARED
  src/state_machine.cpp)
  
add_library(position_service_component SHARED
  src/position_service.cpp)



rclcpp_components_register_node(state_machine_component
  PLUGIN "rt2_assignment1_2::StateMachine"
  EXECUTABLE comp_state_machine)
  
rclcpp_components_register_node(position_service_component
  PLUGIN "rt2_assignment1_2::RandPosServer"
  EXECUTABLE comp_position_service)
  
  
  
ament_target_dependencies(state_machine_component rclcpp rclcpp_components assignment_interfaces)

ament_target_dependencies(position_service_component rclcpp rclcpp_components assignment_interfaces)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#add_executable(state_machine src/state_machine.cpp)

#ament_target_dependencies(state_machine rclcpp assignment_interfaces)

   
#add_executable(position_service src/position_service.cpp)

#ament_target_dependencies(position_service rclcpp assignment_interfaces)


install(TARGETS 
        state_machine_component
        comp_state_machine
        position_service_component
        comp_position_service
        #state_machine 
        #position_service
        DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib)
    
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
