#include <stdio.h>
#include <math.h>

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"


#include "RosAria.h"
#include <sstream>

/*
 * Main Program
 */


int main( int argc, char** argv )
{
  ros::init(argc,argv, "RosAria");
  ros::NodeHandle n(std::string("~"));
  Aria::init();

  RosAriaNode *node = new RosAriaNode(n);

  if( node->Setup() != 0 )
  {
    printf( "setup failed... \n" );
    return -1;
  }

  node->spin();

  delete node;

  printf( "\nQuitting... \n" );
  return 0;

}
