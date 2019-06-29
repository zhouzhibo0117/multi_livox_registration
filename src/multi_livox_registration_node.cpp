
#include "multi_livox_registration.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "MultiLivoxRegistration");

    MutilLivoxRegistration mlr;

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}