#pragma once

#include "ros/ros.h"
#include <vector>
#include <map>
#include "RoboyBehaviourXmlParser.hpp"
#include "common_utilities/Initialize.h"
#include "common_utilities/Steer.h"
#include "common_utilities/CommonDefinitions.h"
#include "common_utilities/ControllerState.h"
#include "common_utilities/SetTrajectory.h"
#include "common_utilities/Trajectory.h"

using namespace std;

class Roboy_Ros_Trajectory{
public:
    Roboy_Ros_Trajectory();
    ~Roboy_Ros_Trajectory();

    void loadBehaviour(QString name);
    void uploadBehaviour(int ID);
    void playBehaviour(int ID);
    void stopBehaviour(int ID);
    void pauseBehaviour(int ID);
    void loopBehaviour(int ID, bool loop);
    void motorStatus( const common_utilities::ControllerState::ConstPtr& msg);
    bool initializeMotors(vector<int8_t> &motors, vector<uint8_t> &controlMode);
    map<int, Behaviour> behaviours;
private:

    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::ServiceClient initialize_srv;
    map<int,ros::ServiceClient> trajectory_srv;
    ros::Publisher steer_pub;
    map<int, ros::Subscriber> motor_status;
    RoboyBehaviourXmlParser parser;
};

int main(int argc, char* argv[]);