#include "robotDriver.h"

using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

robotDriver::robotDriver() {
    drvOptions_ll.clear();
    drvOptions_rl.clear();
    drvOptions_to.clear();
    drv_ll  = 0;
    drv_rl  = 0;
    drv_to  = 0;
    // to dump data
    ienc_ll = 0;
    ienc_rl = 0;
    ienc_to = 0;
    imenc_ll = 0;
    imenc_rl = 0;
    imenc_to = 0;
    iampl_ll = 0;
    iampl_rl = 0;
    iampl_to = 0;
    // for pwm control
    imot_ll = 0;
    imot_rl = 0;
    imot_to = 0;
    
    verbose = 1;
    connected=false;
    iCub::iDyn::version_tag tag;
    icub_dyn = new iCub::iDyn::iCubWholeBody(tag);
}

bool robotDriver::configure(string robotName) {
    bool ret=true;

    drvOptions_ll.put("device","remote_controlboard");
    drvOptions_rl.put("device","remote_controlboard");
    drvOptions_to.put("device","remote_controlboard");

    string remote;
    string local;
    remote = string("/") + robotName + string("/left_leg");
    local  = string("/") + string("dataDumperCustom") + string("/left_leg");
    drvOptions_ll.put("remote",remote.c_str());
    drvOptions_ll.put("local",local.c_str());
    remote = string("/") + robotName + string("/right_leg");
    local  = string("/") + string("dataDumperCustom") + string("/right_leg");
    drvOptions_rl.put("remote",remote.c_str());
    drvOptions_rl.put("local",local.c_str());
    remote = string("/") + robotName + string("/torso");
    local  = string("/") + string("dataDumperCustom") + string("/torso");
    drvOptions_to.put("remote",remote.c_str());
    drvOptions_to.put("local",local.c_str());

    if (verbose)
    {
        std::cout << "right leg driver options:\n" << drvOptions_rl.toString().c_str();
        std::cout << "left  leg driver options:\n" << drvOptions_ll.toString().c_str();
        std::cout << "torso     driver options:\n" << drvOptions_to.toString().c_str();
    }

    return ret;
    }

bool robotDriver::init(bool dump_robot) {
    //return true; //@@@@@

    this->drv_ll=new PolyDriver(this->drvOptions_ll);
    this->drv_rl=new PolyDriver(this->drvOptions_rl);
    this->drv_to=new PolyDriver(this->drvOptions_to);

    if(dump_robot)
    {
      if (this->drv_ll->isValid() && this->drv_rl->isValid() && this->drv_to->isValid())
          connected = this->drv_ll->view(ienc_ll) &&
                      this->drv_ll->view(imenc_ll) && this->drv_ll->view(iampl_ll) && this->drv_ll->view(imot_ll) &&
                      this->drv_rl->view(ienc_rl) &&
                      this->drv_rl->view(imenc_rl) && this->drv_rl->view(iampl_rl) && this->drv_rl->view(imot_rl) &&
                      this->drv_to->view(ienc_to) &&
                      this->drv_to->view(imenc_to) && this->drv_to->view(iampl_to) && this->drv_to->view(imot_to);
      else
          connected=false;
    }
    else
    {
      if (this->drv_ll->isValid() && this->drv_rl->isValid() && this->drv_to->isValid())
          connected = this->drv_ll->view(ienc_ll) &&
                      this->drv_rl->view(ienc_rl) &&
                      this->drv_to->view(ienc_to);
      else
          connected=false;
    }

    if (!connected)
    {
        if (this->drv_ll)
        {
            delete this->drv_ll;
            this->drv_ll=0;
        }
        if (this->drv_rl)
        {
            delete this->drv_rl;
            this->drv_rl=0;
        }
        if (this->drv_to)
        {
            delete this->drv_to;
            this->drv_to=0;
        }
    }
    return connected;
}

robotDriver::~robotDriver() {
    if (this->drv_ll)
    {
        delete this->drv_ll;
        this->drv_ll=0;
    }
    if (this->drv_rl)
    {
        delete this->drv_rl;
        this->drv_rl=0;
    }
    if (this->drv_to)
    {
        delete this->drv_to;
        this->drv_to=0;
    }
}