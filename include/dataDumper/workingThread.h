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

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/yarp/YARPConversions.h>

#include <Eigen/Core>

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
    bool                                      dump_ft;
    bool                                      dump_dcmwalking;
    
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
    
    // DCM walking
    std::string                               current_base;
    bool                                      base_switch;
    yarp::os::BufferedPort<yarp::sig::Vector> walking_base;
    
    // walking ports
    yarp::os::BufferedPort<yarp::sig::Vector>  walking_left_foot;
    yarp::os::BufferedPort<yarp::sig::Vector>  walking_right_foot;
    yarp::os::BufferedPort<yarp::sig::Vector>  walking_joints;
    yarp::os::BufferedPort<yarp::sig::Vector>  walking_com;
    
    
    yarp::os::BufferedPort<yarp::sig::Vector>  wbd_left_foot;
    yarp::os::BufferedPort<yarp::sig::Vector>  wbd_right_foot;
    
    //****** FeetOrtTest ******
    // FeetOrtTest data
    bool                                      feetOrtTest;
    FILE*                                     l_foot_ort_file;
    FILE*                                     r_foot_ort_file;
    FILE*                                     l_foot_ort_imu_file;
    FILE*                                     r_foot_ort_imu_file;
    
    // kindyn computations
    std::string                                model_name;
    iDynTree::ModelLoader                      model_loader;
    iDynTree::KinDynComputations               m_kinDyn;
    iDynTree::Rotation                         imuToStrain; // fixed from CAD https://github.com/robotology-playground/icub-model-generator/issues/91
    iDynTree::Rotation                         lEarthToBase; // computed only in the first run
    iDynTree::Rotation                         rEarthToBase; // computed only in the first run
    iDynTree::Transform                        w_H_b;
    
    // Orientation jumps
    iDynTree::Vector3                          prevLStrainRot;
    iDynTree::Vector3                          prevLStrainRotIMU;
    iDynTree::Vector3                          prevRStrainRot;
    iDynTree::Vector3                          prevRStrainRotIMU;
    iDynTree::Vector3                          lStrainRotOffset;
    iDynTree::Vector3                          lStrainRotIMUOffset;
    iDynTree::Vector3                          rStrainRotOffset;
    iDynTree::Vector3                          rStrainRotIMUOffset;
    
    // FT
    yarp::os::BufferedPort<yarp::os::Bottle>   ft_left_leg;
    yarp::os::BufferedPort<yarp::os::Bottle>   ft_right_leg;
    yarp::os::BufferedPort<yarp::os::Bottle>   ft_left_foot;
    yarp::os::BufferedPort<yarp::os::Bottle>   ft_right_foot;
    FILE*                                      l_foot_ft_file;
    FILE*                                      r_foot_ft_file;
    FILE*                                      l_leg_ft_file;
    FILE*                                      r_leg_ft_file;
    

    WorkingThread(int period=5);
    ~WorkingThread();
    void attachRobotDriver(robotDriver *p);
    bool threadInit();
    void dump_data(int j);
    void computeFeetOrt(int j, yarp::sig::Vector& lleg, yarp::sig::Vector& rleg, yarp::os::Bottle* lort, yarp::os::Bottle* rort);
    void run();
};

#endif