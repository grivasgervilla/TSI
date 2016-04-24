
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "costmap_2d/costmap_2d_ros.h"




int main(int argc, char** argv){
 ros::init(argc, argv, "miscostmaps_node");
 ros::NodeHandle nh;
 tf::TransformListener tf(ros::Duration(10));
 costmap_2d::Costmap2DROS localcostmap("local_costmap", tf);
 costmap_2d::Costmap2DROS globalcostmap("global_costmap", tf);

double res;
   nh.getParam("local_costmap/resolution",res);
   ROS_INFO("La resolución del costmap local es %f", res);
   ros::spin();

  return(0);
 }
