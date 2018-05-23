#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <string.h>

#include "scriptModule.h"

using namespace std;
using namespace yarp::os;

scriptModule::scriptModule()
{
    verbose=true;
}

bool scriptModule::configure(ResourceFinder &rf) {
    Time::turboBoost();
    
    this->rfCopy = rf;

    if (rf.check("name"))
        name=string("/")+rf.find("name").asString().c_str();
    else
        name="/dataDumperCustom";
    
    thread.name = name;

    contextPath=rf.getContext().c_str();
    fprintf(stderr,"||| contextPath: %s\n", contextPath.c_str());

    rpcPort.open((name+"/rpc").c_str());
    attach(rpcPort);
    
    thread.robotName = rf.check("robot",yarp::os::Value("icub")).asString();
    
    Property portProp;
    portProp.put("robot", thread.robotName);  
    
    if(rf.check("sim"))
    {
      thread.dump_robot = false;
      thread.dump_simulator = true;
    } else
    {
      thread.dump_robot = true;
      thread.dump_simulator = false;
    }
    
    if(rf.check("carrier"))
    {
      thread.carrier = rf.check("carrier");
    }
    
    //check data dumping first in order to (un)enable 
    if (rf.check("dump")==true)
    {
      yarp::os::Bottle* dumpList = rf.find("dump").asList();
      //encoders, mencoders, encoder speeds, currents, pwm, imu
      if(dumpList->size() > 0)
      {
        for(unsigned int i = 0; i < dumpList->size(); i++)
        {
          if(dumpList->get(i).asString()=="enc")
          {
            cout << "##### Dumping encoders" << endl;
            thread.dump_enc = true;
          }
          if(dumpList->get(i).asString()=="encspeeds")
          {
            cout << "##### Dumping encoders speeds" << endl;
            thread.dump_enc_speed = true;
          }
          if(dumpList->get(i).asString()=="menc")
          {
            cout << "##### Dumping motor encoders" << endl;
            thread.dump_menc = true;
          }
          if(dumpList->get(i).asString()=="mencspeeds")
          {
            cout << "##### Dumping motor encoder speeds" << endl;
            thread.dump_menc_speed = true;
          }
          if(dumpList->get(i).asString()=="curr")
          {
            cout << "##### Dumping currents" << endl;
            thread.dump_currents = true;
          }
          if(dumpList->get(i).asString()=="pwm")
          {
            cout << "##### Dumping pwm" << endl;
            thread.dump_pwm = true;
          }
          if(dumpList->get(i).asString()=="imu")
          {
            cout << "##### Dumping IMU" << endl;
            thread.dump_imu = true;
          }
          if(dumpList->get(i).asString()=="walking")
          {
            cout << "##### Dumping walking trajectories" << endl;
            thread.dump_walking = true;
          }
        }
      } else
      {
        thread.dump_enc = true;
        thread.dump_enc_speed = true;
        thread.dump_menc = true;
        thread.dump_menc_speed = true;
        thread.dump_currents = true;
        thread.dump_pwm = true;
        thread.dump_imu = true;
        thread.dump_walking = true;
        cout << "Warning: no dumping part specified, dumping everything." << endl;
      }
    }else
    {
      thread.dump_enc = true;
      thread.dump_menc = true;
      thread.dump_menc = true;
      thread.dump_menc_speed = true;
      thread.dump_currents = true;
      thread.dump_pwm = true;
      thread.dump_imu = true;
      thread.dump_walking = true;
      cout << "Warning: no dumping part specified, dumping everything." << endl;
    }
    
    if (rf.check("period")==true)
    {
      thread.period = rf.find("period").asInt();
      cout << "Thread period set to "<<thread.period<< "ms" <<endl;
      thread.setRate(thread.period);
    }
    
    // open files for data dumping
    if(thread.dump_robot)
    {
      if(thread.dump_enc)
        thread.enc_data_file = fopen ("encoders.txt", "w");
      if(thread.dump_enc_speed)
        thread.enc_speed_data_file = fopen ("encoders_speeds.txt", "w");
      if(thread.dump_menc)
        thread.menc_data_file = fopen ("mencoders.txt", "w");
      if(thread.dump_menc_speed)
        thread.menc_speed_data_file = fopen ("mencoders_speeds.txt", "w");
      if(thread.dump_currents)
        thread.currents_file = fopen ("currents.txt", "w");
      if(thread.dump_pwm)
        thread.pwm_file = fopen ("pwm_out.txt", "w");
      if(thread.dump_imu)
        thread.imu_data_file = fopen ("imu_out.txt", "w");
      if(thread.dump_walking)
      {
        thread.walking_com_file = fopen("walking_com.txt", "w");
        thread.walking_joints_file = fopen("walking_joints.txt", "w");
        thread.walking_feet_file = fopen("walking_feet.txt", "w");
      }
    } else if(thread.dump_simulator)
    {
      if(thread.dump_enc)
        thread.enc_data_file = fopen ("encoders.txt", "w");
      if(thread.dump_enc_speed)
        thread.enc_speed_data_file = fopen ("encoders_speeds.txt", "w");
      if(thread.dump_imu)
        thread.imu_data_file = fopen ("imu_out.txt", "w");
      if(thread.dump_walking)
      {
        thread.walking_com_file = fopen("walking_com.txt", "w");
        thread.walking_joints_file = fopen("walking_joints.txt", "w");
        thread.walking_feet_file = fopen("walking_feet.txt", "w");
      }
    }
    
    
    //**** Feet ort test
    if(rf.check("feetOrtTest"))
    {
      thread.feetOrtTest = true;
    }
    if(thread.feetOrtTest)
    {
      thread.model_name = rf.check("model",yarp::os::Value("model.urdf")).asString();
      thread.l_foot_ort_file = fopen("l_foot_ort.txt","w");
      thread.r_foot_ort_file = fopen("r_foot_ort.txt","w");
      thread.l_foot_ort_imu_file = fopen("l_foot_ort_imu.txt","w");
      thread.r_foot_ort_imu_file = fopen("r_foot_ort_imu.txt","w");
    }

    //*** start the robot driver
    if (!robot.configure(thread.robotName))
    {
        cerr<<"Error configuring position controller, check parameters"<<endl;
        return false;
    }
    else {
        cout << "Configuration done." << endl;
    }

    if (!robot.init(thread.dump_robot))
    {
        cerr<<"Error cannot connect to remote ports"<<endl;
        return false;
    }
    else {
        cout << "Initialization done." << endl;
    }

    //*** attach the robot driver to the thread and start it
    thread.attachRobotDriver(&robot);
    if (!thread.start())
    {
        cerr<<"ERROR: Thread did not start, queue will not work"<<endl;
    }
    else
    {
        cout<<"Thread started"<<endl;
    }

    cout << "Using parameters:" << endl << rf.toString() << endl;
    cout << "Module successfully configured. ready." << endl;
    return true;
}

bool scriptModule::respond(const Bottle &command, Bottle &reply)
{
    bool ret=true;
    this->thread.mutex.wait();

    if (command.size()!=0)
    {
        string cmdstring = command.get(0).asString().c_str();
        {
            if  (cmdstring == "help")
                {
                    cout << "Available commands:"          << endl;
                    cout << "start" << endl;
                    cout << "stop"  << endl;
                    cout << "quit" << endl;
                    reply.addVocab(Vocab::encode("ack"));
                }
            else if  (cmdstring == "start")
                {
                    if (this->thread.actions.current_action == 0)
                        this->thread.actions.current_status = ACTION_START;
                    else
                        this->thread.actions.current_status = ACTION_RUNNING;
                    reply.addVocab(Vocab::encode("ack"));
                }
            else if  (cmdstring == "stop")
                {
                    this->thread.actions.current_status = ACTION_IDLE;
                    cout << "Stop data dumping." << endl;
                    reply.addVocab(Vocab::encode("ack"));
                }
            else if  (cmdstring == "quit")
                {
                  this->thread.actions.current_status = ACTION_QUIT;
                  reply.addVocab(Vocab::encode("ack"));
                  stopModule();
                }
            else
                {
                    reply.addVocab(Vocab::encode("nack"));
                    ret = false;
                }
        }
    }
    else
    {
        reply.addVocab(Vocab::encode("nack"));
        ret = false;
    }

    this->thread.mutex.post();
    return ret;
}

bool scriptModule::close()
{
    rpcPort.interrupt();
    rpcPort.close();

    return true;
}

double getPeriod()    { return 1.0;  }
bool   updateModule() { return true; }
