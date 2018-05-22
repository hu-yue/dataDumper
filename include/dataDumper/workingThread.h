#ifndef WORKINGTHREAD_H
#define WORKINGTHREAD_H

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Bottle.h>
#include <math.h>

#include "robotDriver.h"
#include "actionClass.h"


// ******************** THE THREAD
class WorkingThread: public yarp::os::RateThread {
private:

    yarp::os::Stamp timestamp;
public:
    robotDriver                               *driver;
    actionClass                               actions;
    yarp::os::Semaphore                       mutex;
    std::string                               robotName;
    std::string                               carrier;
    std::string                               name;
    
    // for PID
    int                                       period;
    
    // for data dumping
    bool                                      dump_robot;
    bool                                      dump_simulator;
    bool                                      dump_enc;
    bool                                      dump_enc_speed;
    bool                                      dump_menc;
    bool                                      dump_menc_speed;
    bool                                      dump_currents;
    bool                                      dump_pwm;
    bool                                      dump_imu;
    bool                                      dump_walking;
    
    // all files to write
    // both robot and gazebo
    FILE*                                     enc_data_file;
    FILE*                                     enc_speed_data_file;
    FILE*                                     imu_data_file;
    
    // robot only
    FILE*                                     menc_data_file;
    FILE*                                     menc_speed_data_file;
    FILE*                                     currents_file;
    FILE*                                     pwm_file;
    
    // for mencoders
    yarp::sig::Vector                         gear_ratio_ll;
    yarp::sig::Vector                         gear_ratio_rl;
    yarp::sig::Vector                         gear_ratio_to;
    yarp::sig::Vector                         enc_offset_ll;
    yarp::sig::Vector                         enc_offset_rl;
    yarp::sig::Vector                         enc_offset_to;
    
    // walking files
    FILE*                                     walking_feet_file;
    FILE*                                     walking_com_file;
    FILE*                                     walking_joints_file;
    
    
    yarp::os::BufferedPort<yarp::os::Bottle>  imu_left_foot;
    yarp::os::BufferedPort<yarp::os::Bottle>  imu_right_foot;
    
    // walking ports
    yarp::os::BufferedPort<yarp::sig::Vector>  walking_left_foot;
    yarp::os::BufferedPort<yarp::sig::Vector>  walking_right_foot;
    yarp::os::BufferedPort<yarp::sig::Vector>  walking_joints;
    yarp::os::BufferedPort<yarp::sig::Vector>  walking_com;

    WorkingThread(int period=5);
    ~WorkingThread();
    void attachRobotDriver(robotDriver *p);
    bool threadInit();
    void dump_data(int j);
    void run();
};

#endif