#include "roboy_ros_trajectory.hpp"

Roboy_Ros_Trajectory::Roboy_Ros_Trajectory() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy_ros_trajectory",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    initialize_srv = nh->serviceClient<common_utilities::Initialize>("/roboy/initialize");
    steer_pub = nh->advertise<common_utilities::Steer>("/roboy/steer", 1);
}

Roboy_Ros_Trajectory::~Roboy_Ros_Trajectory() {

}

void Roboy_Ros_Trajectory::loadBehaviour(QString name) {
    Behaviour behaviour;
    parser.readRoboyBehavior(name, behaviour);
    behaviours[behaviour.id] = behaviour;
}

void Roboy_Ros_Trajectory::uploadBehaviour(int ID) {
    if (behaviours.find(ID) != behaviours.end()) {
        for (auto trajectory:behaviours[ID].trajectories) {
            common_utilities::SetTrajectory msg;
            msg.request.trajectory.id = ID;
            msg.request.trajectory.samplerate = behaviours[ID].samplerate;
            msg.request.trajectory.waypoints = trajectory.second.setPoints;
            trajectory_srv[trajectory.first].call(msg);
        }
    } else {
        ROS_WARN_STREAM("behaviour " << ID << " hasnt been loaded yet");
    }
}

void Roboy_Ros_Trajectory::playBehaviour(int ID) {
    common_utilities::Steer msg;
    msg.steeringCommand = PLAY_TRAJECTORY;
    msg.id = ID;
    steer_pub.publish(msg);
}

void Roboy_Ros_Trajectory::stopBehaviour(int ID) {
    common_utilities::Steer msg;
    msg.steeringCommand = STOP_TRAJECTORY;
    msg.id = ID;
    steer_pub.publish(msg);
}

void Roboy_Ros_Trajectory::pauseBehaviour(int ID) {
    common_utilities::Steer msg;
    msg.steeringCommand = PAUSE_TRAJECTORY;
    msg.id = ID;
    steer_pub.publish(msg);
}

void Roboy_Ros_Trajectory::motorStatus(const common_utilities::ControllerState::ConstPtr &msg) {
    ROS_INFO_STREAM("motor id: " << msg->id << " state: " << msg->state);
}

bool Roboy_Ros_Trajectory::initializeMotors(vector<int8_t> &motors, vector<uint8_t> &controlMode) {
    common_utilities::Initialize msg;
    msg.request.controlmode = controlMode;
    msg.request.idList = motors;
    for (auto motor:motors) {
        // we have to subscribe to the controller status
        char topic[100];
        sprintf(topic, "/roboy/status_motor%d", motor);
        motor_status[motor] = nh->subscribe(topic, 1000, &Roboy_Ros_Trajectory::motorStatus, this);
        // we set up the trajectory services for every motor
        sprintf(topic, "/roboy/trajectory_motor%d", motor);
        trajectory_srv[motor] = nh->serviceClient<common_utilities::SetTrajectory>(topic);
    }
    return initialize_srv.call(msg);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "roboy_ros_trajectory");

    Roboy_Ros_Trajectory roboy_ros_trajectory;
    // the behaviour we want to play is called test and located in ~/.roboy_gui/behaviours
    roboy_ros_trajectory.loadBehaviour("test");
    // it uses 16 motors
    vector<int8_t> motors = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    // all in position control
    vector<uint8_t> controlmode = {POSITION_CONTROL, POSITION_CONTROL, POSITION_CONTROL, POSITION_CONTROL,
                                   POSITION_CONTROL, POSITION_CONTROL, POSITION_CONTROL, POSITION_CONTROL,
                                   POSITION_CONTROL, POSITION_CONTROL, POSITION_CONTROL, POSITION_CONTROL,
                                   POSITION_CONTROL, POSITION_CONTROL, POSITION_CONTROL, POSITION_CONTROL};
    // first we need to initialize the ros_control position controllers via a service call
    if (roboy_ros_trajectory.initializeMotors(motors, controlmode)) {
        // when this was successful, the requested position controllers are running and waiting for trajectories
        int id = roboy_ros_trajectory.behaviours.begin()->first;
        // we upload the trajectories to the position controllers, who process them
        roboy_ros_trajectory.uploadBehaviour(id);
        // we send a play steering message
        roboy_ros_trajectory.playBehaviour(id);
    } else {
        ROS_ERROR("failed to initialize motors, did you launch roboy?\n $ roslaunch roboy_hardware roboy.launch");
    }


    return 0;
}