#ifndef ROBOTDRIVER_H
#define ROBOTDRIVER_H

#include <yarp/os/Property.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Matrix.h>
#include <iCub/ctrl/math.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iostream>


// ******************** ROBOT DRIVER CLASS
class robotDriver
{
public:
    bool verbose;
    bool connected;
    yarp::os::Property          drvOptions_ll;
    yarp::os::Property          drvOptions_rl;
    yarp::os::Property          drvOptions_to;
    yarp::dev::PolyDriver       *drv_ll;
    yarp::dev::PolyDriver       *drv_rl;
    yarp::dev::PolyDriver       *drv_to;
    iCub::iDyn::iCubWholeBody   *icub_dyn;
    
    // to export data
    yarp::dev::IEncoders        *ienc_ll;
    yarp::dev::IEncoders        *ienc_rl;
    yarp::dev::IEncoders        *ienc_to;
    
    // robot only
    yarp::dev::IMotorEncoders   *imenc_ll;
    yarp::dev::IMotorEncoders   *imenc_rl;
    yarp::dev::IMotorEncoders   *imenc_to;
    yarp::dev::IAmplifierControl *iampl_ll;
    yarp::dev::IAmplifierControl *iampl_rl;
    yarp::dev::IAmplifierControl *iampl_to;
    yarp::dev::IMotor           *imot_ll;
    yarp::dev::IMotor           *imot_rl;
    yarp::dev::IMotor           *imot_to;

public:
    robotDriver();
//     yarp::sig::Matrix compute_transformations (actionStruct act);
    bool configure(std::string robotName);
    bool init(bool dump_robot=false);
    ~robotDriver();
};


#endif
