#include <inttypes.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "assignment_interfaces/srv/random_position.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using RandomPosition = assignment_interfaces::srv::RandomPosition;


namespace rt2_assignment1_2{

class RandPosServer : public  rclcpp::Node
{
public:
  //class constructor
  RandPosServer(const rclcpp::NodeOptions & options)
  : Node("random_position_server", options)
  {
    //Create the server of the random position service /position_server
    service_r = this->create_service<RandomPosition>("/position_server", std::bind(&RandPosServer::myrandom, this, _1, _2, _3));
  
  }
private:
  //A function to generate a random number between a minimum and maximum value
  double randMToN(double M, double N){     
  return M + (rand() / ( RAND_MAX / (N-M) ) ); 
  }
  
  //The callback to the server service_r
  bool myrandom (const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<RandomPosition::Request> request,
  const std::shared_ptr<RandomPosition::Response> response)
  {
    (void) request_header;
    //Get the random x, y, and theta values in the requested ranges
    response->x = this->randMToN(request->x_min, request->x_max);
    response->y = this->randMToN(request->y_min, request->y_max);
    response->theta = this->randMToN(-3.14, 3.14);
    //Print info on screen
    RCLCPP_INFO(this->get_logger(), "RandomPositionServer: Received a request");
    return true;
}

  rclcpp::Service<RandomPosition>::SharedPtr service_r;   //Initialize the server to the random position service

};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1_2::RandPosServer) 

