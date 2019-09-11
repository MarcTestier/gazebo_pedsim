#include "pedsim_test/pedsim_test.hpp"


PedSimTest::PedSimTest() : nh_()
{
    initSetup();
}

PedSimTest::~PedSimTest()
{

}

void PedSimTest::initSetup()
{
    ROS_INFO("Starting pedsim test");

    // Setup
    Ped::Tscene *pedscene = new Ped::Tscene(-200, -200, 400, 400);
    Ped::Twaypoint *w1 = new Ped::Twaypoint(-100, 0, 24);
    Ped::Twaypoint *w2 = new Ped::Twaypoint(+100, 0, 12);
    Ped::Tobstacle *o = new Ped::Tobstacle(0, -50,  0, +50);
    pedscene->addObstacle(o);
    for (int i = 0; i < 10; i++) {
        Ped::Tagent *a = new Ped::Tagent();
        a->addWaypoint(w1);
        a->addWaypoint(w2);
        a->setPosition(-50 + rand()/(RAND_MAX/80)-40, 0 + rand()/(RAND_MAX/20) -10, 0);
        pedscene->addAgent(a);
    }

    // Move all agents for 7 steps
    for (int i = 0; i < 7; ++i) {
        pedscene->moveAgents(0.3);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));

        vector<Ped::Tagent*> agent_array = pedscene->getAllAgents();
        for (int j = 0; j < agent_array.size(); j++) {
            Ped::Tvector pos = agent_array[5]->getPosition();
            ROS_INFO("Position of agent %d : (%f, %f, %f)", j, pos.x, pos.y, pos.z);
        }
    }
    // Cleanup
    for (Ped::Tagent* agent : pedscene->getAllAgents()) delete agent;
    delete pedscene;
    delete w1;
    delete w2;
    delete o;

    ROS_INFO("Stoping pedsim test");
}

int main(int argc, char **argv)
{
    // Initialize node and nodehandles
    ros::init(argc, argv, "pedsim_test");

    PedSimTest node;

    ros::spin();

    return 0;
}
