#include "pr2_head_manager/pr2_head_manager.h"

int main(int argc, char *argv[]){
    ros::init(argc,argv,"pr2_head_manager");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    std::vector<std::string> plugins;
    for(int i = 1; i < argc; i++)
      plugins.push_back(std::string(argv[i]));

    Pr2HeadManager mgr(nh, plugins);

    std::thread th(&Pr2HeadManager::run, &mgr);

    ros::spin();

    th.join();
}
