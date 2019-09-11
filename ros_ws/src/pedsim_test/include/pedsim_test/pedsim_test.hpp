#ifndef PEDSIM_TEST_HPP
#define PEDSIM_TEST_HPP

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include "pedsim/ped_includes.h"
#include "pedsim/ped_outputwriter.h"

class PedSimTest
{
public:
    PedSimTest();
    ~PedSimTest();

private:
    void initSetup();
    
private:
    ros::NodeHandle nh_;

};


#endif  // PEDSIM_TEST_HPP
