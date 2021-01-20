#include "ur_interface_pkg/ur_manager_group.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur_manager_group_node");

    URManagerGroup manager;
    manager.run();

    return 0;
}