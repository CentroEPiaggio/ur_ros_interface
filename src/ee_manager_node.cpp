#include "ur_interface_pkg/ee_manager.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "left_manager_node");

    device_list dl;
    
    device_params soft_hand_params;
    soft_hand_params.ID = 3;
    soft_hand_params.min = 0;
    soft_hand_params.max = 18000;
    soft_hand_params.topic = "/left_hand_cmd";
    
    dl["soft_hand"] = soft_hand_params;

    EndEffectorsManager EEM(dl,"/dev/ttyUSB0");
    EEM.run();

    return 0;
}