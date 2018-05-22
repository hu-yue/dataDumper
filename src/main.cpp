/*
 * Copyright (C)2013  iCub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * Last Modified by: Yue Hu
 * email:  marco.randazzo@iit.it, yue.hu@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Thread.h>

#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include "constants.h"
#include "scriptModule.h"

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;

int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("dataDumperCustom");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--name               <moduleName>: set new module name" << endl;
        cout << "\t--robot              <robotname>:  robot name"          << endl;
        cout << "\t--sim                if using Gazebo" << endl;
        cout << "\t--period             <period>: the period in ms of the internal thread (default 5)"  << endl;
        cout << "\t--dump               <what to dump>: encoders, mencoders, encoder speeds, currents, pwm, imu, walking" << endl;
        cout << "\t--carrier            <carrie>: upd, tcp" << endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        cout << "ERROR: yarp.checkNetwork() failed."  << endl;
        return -1;
    }

    scriptModule mod;

    return mod.runModule(rf);
}



