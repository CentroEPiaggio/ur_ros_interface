#include "ur_interface_pkg/ur_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur_manager_node");

    URManager manager;
    manager.run();

    return 0;
}