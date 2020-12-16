#include "dvs_integrator/integrator.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dvs_integrator");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  dvs_integrator::Integrator integrator(nh, nh_private);

  ros::spin();

  return 0;
}
