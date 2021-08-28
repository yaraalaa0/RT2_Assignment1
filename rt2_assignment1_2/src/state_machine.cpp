#include <memory>
#include <chrono>
#include <cinttypes>
#include <inttypes.h>
#include <cstdlib>

//add the ROS header
#include "rclcpp/rclcpp.hpp"
//header required for the component
#include "rclcpp_components/register_node_macro.hpp"

//add the headers for messages and services
#include "assignment_interfaces/srv/command.hpp"
#include "assignment_interfaces/srv/position.hpp"
#include "assignment_interfaces/srv/random_position.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using RandomPosition = assignment_interfaces::srv::RandomPosition;
using Position = assignment_interfaces::srv::Position;
using Command = assignment_interfaces::srv::Command;


namespace rt2_assignment1_2{
class StateMachine : public rclcpp::Node
{
public:
  //class constructor
  StateMachine(const rclcpp::NodeOptions & options)
  : Node("state_machine", options)
  { 
    
    //initialize the client for the random_position service
    client_rp = this->create_client<RandomPosition>("/position_server");
    
    //initialize the client for the go_to_point service
    client_p = this->create_client<Position>("/go_to_point");
    
    //Initialize the user interface server
    service_ = this->create_service<Command>("/user_interface",std::bind(&StateMachine::user_interface, this, _1, _2, _3));
    
    //Initiaize the timer for the main callback function
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&StateMachine::call_main, this));
    
    //Wait for the random_position service to be available
    while (!client_rp->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service rand position to appear...");
    }
    
    //Wait for the go_to_point service to be available
    while (!client_p->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service go to point to appear...");
    }
    
    //Initialize the services request and response variables 
    this->request_rp = std::make_shared<RandomPosition::Request>();
    this->response_rp = std::make_shared<RandomPosition::Response>();
    this->request_p = std::make_shared<Position::Request>();
  }
  
private:
 
  //function to call the random position service
  void call_server_rp(){
  
  //define the minimum and maximum limits for the random position
  this->request_rp->x_max = 5;
  this->request_rp->x_min = -5;
  this->request_rp->y_max = 5;
  this->request_rp->y_min = -5;
  
  //callback function that gets executed when the responce to this service request is received
  using ServiceResponseFutureRP =
    rclcpp::Client<RandomPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFutureRP future) {
      //print info on the screen
      RCLCPP_INFO(this->get_logger(), "Got result: x = [%" PRId64 "] y = [%" PRId64 "] theta = [%" PRId64 "]", future.get()->x, future.get()->y, future.get()->theta);
      //update the response variable
      this->response_rp = future.get();
      //update the flag
      this->flag_rp = 1;
    };
    
  //sending the request to the service
  auto future_result = this->client_rp->async_send_request(this->request_rp, response_received_callback);
  
  
  }
  
  //function to call the go to point service
  void call_server_p(){
  
    //set the request to the last received random position response
    this->request_p->x = this->response_rp->x;
    this->request_p->y = this->response_rp->y;
    this->request_p->theta = this->response_rp->theta;
    
    //callback function that gets executed when the responce to this service request is received
    using ServiceResponseFutureP =
    rclcpp::Client<Position>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFutureP future) {
      //print info on the screen
      RCLCPP_INFO(this->get_logger(), "Target Reached");
      //update the flag
      this->flag_p = 1;
    };
    
    //sending the request to the service
    auto future_result = this->client_p->async_send_request(this->request_p, response_received_callback);
  

  }
  

  //Callback function of the user_interface server
  bool user_interface(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<Command::Request> request,
  const std::shared_ptr<Command::Response> response)
  {
    (void) request_header;
    
    RCLCPP_INFO(this->get_logger(), "UserInterfaceServer: received a request");
    
    //Check the command received from the client and update the flag accordingly
    if(request->command == "start"){
      this->start = true;
    }
    else{
      this->start = false;
    }
  
    return true;
  }
  
  //Main callback function to manage calling the services
  void call_main(){
  
    //Check that the start is active
    if(this->start){
      
      //Check if the previous target has been reached or we are in the start of simulation
      //In this case, call the random position server to get a new target position
      if(this->flag_rp==0 && this->flag_p){
        //reset the flag to false in order not to execute this again
        this->flag_p = 0;
        //Request a new random position
        this->call_server_rp();
      }
      
      //Check if the random position client has received a response
      //In this case, call the go_to_point server to drive the robot to the goal position
      if(this->flag_rp && this->flag_p == 0){
        //reset the flag to false in order not to execute this again
        this->flag_rp = 0;
        //Call the go_to_point service
        this->call_server_p();
      }
      
    }
  
  }


  rclcpp::TimerBase::SharedPtr timer_; //timer for executing the main callback function
  rclcpp::Client<RandomPosition>::SharedPtr client_rp; //random position client
  rclcpp::Client<Position>::SharedPtr client_p; //go to point client
  rclcpp::Service<Command>::SharedPtr service_;  //user interface server
  std::shared_ptr<RandomPosition::Request> request_rp;  //the request variable to random position server
  std::shared_ptr<RandomPosition::Response> response_rp;  //the response variable to random position server
  std::shared_ptr<Position::Request> request_p;  //the request variable to go_to_point server
  
  bool start = false; //the start flag to indicate the user command
  int flag_rp = 0; //flag to indicate receiving a random position response
  int flag_p = 1; //flag to indicate receving a go_to_point response
  
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1_2::StateMachine) 

