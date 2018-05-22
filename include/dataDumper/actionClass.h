#ifndef ACTIONCLASS_H
#define ACTIONCLASS_H


#include <stdio.h>
#include <vector>
#include <string.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdlib.h>
#include <yarp/os/RFModule.h>
#include <assert.h>
#include <deque>

#include "constants.h"


// ******************** ACTION CLASS
struct actionStruct
{
    int         counter;
    double      time;
    double      q;
    std::string tag;

    public:
    actionStruct();
};

class actionClass
{
    public:
    size_t                          current_action;
    int                             current_status;
    std::vector <actionStruct>      action_vector;

    /**
     *  Class containing data structures for the different types of data parsed by this module. When playing back walking trajectories in position mode, the parsed data is stored in the variable action_vector, which will contain the trajectories for both legs and the torso. Instead, when playing back trajectories suitable for the torqueBalancing module, the parsed files are stored in the variable action_vector_torqueBalancing.
     */
    actionClass();
    
};

#endif
