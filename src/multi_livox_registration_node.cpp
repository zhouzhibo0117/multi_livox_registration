
#include "multi_livox_registration.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "MultiLivoxRegistration");

    MutilLivoxRegistration mlr;

    ros::spin();
    return 0;
}