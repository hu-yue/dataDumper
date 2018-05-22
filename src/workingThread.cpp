#include <yarp/os/RateThread.h>
#include <cmath>
#include <stdio.h>

#include "workingThread.h"

using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

const double WP_RAD2DEG = 180.0 / M_PI;

WorkingThread::WorkingThread(int period): RateThread(period)
{    
    // data dumping
    dump_robot = false;
    dump_simulator = false;
    
    dump_enc = false;
    dump_enc_speed = false;
    dump_menc = false;
    dump_menc_speed = false;
    dump_currents = false;
    dump_pwm = false;
    dump_imu = false;
    dump_walking = false;
    
    carrier = "udp";
    
    // files
    enc_data_file = NULL;
    enc_speed_data_file = NULL;
    imu_data_file = NULL;
    
    menc_data_file = NULL;
    menc_speed_data_file = NULL;
    currents_file = NULL;
    pwm_file = NULL;
    
    walking_com_file = NULL;
    walking_feet_file = NULL;
    walking_joints_file = NULL;
}

WorkingThread::~WorkingThread()
{
    cout << "Closing files and ports." << endl;
  
    if(enc_data_file!=NULL)
      fclose(enc_data_file);
    if(enc_speed_data_file!=NULL)
      fclose(enc_speed_data_file);
    if(menc_data_file!=NULL)
      fclose(menc_data_file);
    if(currents_file!=NULL)
      fclose(currents_file);
    if(pwm_file!=NULL)
      fclose(pwm_file);
    if(imu_data_file!=NULL)
      fclose(imu_data_file);
    if(walking_com_file!=NULL)
      fclose(walking_com_file);
    if(walking_joints_file!=NULL)
      fclose(walking_joints_file);
    if(walking_feet_file!=NULL)
      fclose(walking_feet_file);
    
    if(dump_imu)
    {
      imu_left_foot.close();
      imu_right_foot.close();
    }
    
    if(dump_walking)
    {
      walking_com.close();
      walking_joints.close();
      walking_left_foot.close();
      walking_right_foot.close();
    }
}

void WorkingThread::attachRobotDriver(robotDriver *p)
{
    if (p)  driver=p;
}

bool WorkingThread::threadInit()
{
    if (!driver)
        return false;
    
    if(dump_imu)
    {
      imu_left_foot.open("/" + name + "/right_leg/imu/measures:i");
      imu_right_foot.open("/" + name + "/left_leg/imu/measures:i");
      
      if(!yarp::os::Network::connect("/" + robotName + "/left_leg/imu/measures:o", "/" + name + "/left_leg/imu/measures:i", carrier)){
        cout << "Unable to connect to LEFT leg IMU port." << endl;
        return EXIT_FAILURE;
      }
      if(!yarp::os::Network::connect("/" + robotName + "/right_leg/imu/measures:o", "/" + name + "/right_leg/imu/measures:i", carrier)){
        cout << "Unable to connect to RIGHT leg IMU port." << endl;
        return EXIT_FAILURE;
      }
    }
    
    if(dump_walking)
    {
      walking_left_foot.open("/" + name + "/walking-coordinator/leftFoot:i");
      walking_right_foot.open("/" + name + "/walking-coordinator/rightFoot:i");
      walking_com.open("/" + name + "/walking-coordinator/com:i");
      walking_joints.open("/" + name + "/walking-coordinator/joints:i");
      
      if(!yarp::os::Network::connect("/walking-coordinator/leftFoot:o", "/" + name + "/walking-coordinator/leftFoot:i", carrier)){
        cout << "Unable to connect to left foot walking-coordinator." << endl;
        dump_walking = false;
        walking_left_foot.close();
      }
      if(!yarp::os::Network::connect("/walking-coordinator/rightFoot:o", "/" + name + "/walking-coordinator/rightFoot:i", carrier)){
        cout << "Unable to connect to right foot walking-coordinator." << endl;
        dump_walking = false;
        walking_right_foot.close();
      }
      if(!yarp::os::Network::connect("/walking-coordinator/com:o", "/" + name + "/walking-coordinator/com:i", carrier)){
        cout << "Unable to connect to com walking-coordinator." << endl;
        dump_walking = false;
        walking_com.close();
      }
      if(!yarp::os::Network::connect("/walking-coordinator/joints:o", "/" + name + "/walking-coordinator/joints:i", carrier)){
        cout << "Unable to connect to joints walking-coordinator." << endl;
        dump_walking = false;
        walking_joints.close();
      }
    }
    
    // acquire pid and motor data
    gear_ratio_ll.resize(6);
    gear_ratio_rl.resize(6);
    gear_ratio_to.resize(3);
    enc_offset_ll.resize(6);
    enc_offset_rl.resize(6);
    enc_offset_to.resize(3);
    
    if(dump_robot && dump_menc)
    {
      // acquire encoder offsets
      driver->imenc_ll->getMotorEncoders(enc_offset_ll.data());
      driver->imenc_rl->getMotorEncoders(enc_offset_rl.data());
      driver->imenc_to->getMotorEncoders(enc_offset_to.data());
      
      // acquire gear ratio
      double temp = 0;
      for(int i = 0; i < 6; i++)
      {
        driver->imot_ll->getGearboxRatio(i,&temp);
        gear_ratio_ll[i] = temp;
        driver->imot_rl->getGearboxRatio(i,&temp);
        gear_ratio_rl[i] = temp;
      }
      for(int i = 0; i < 3; i++)
      {
        driver->imot_to->getGearboxRatio(i,&temp);
        gear_ratio_to[i] = temp;
      }
    }
    
    return true;
}

void WorkingThread::dump_data(int j)
{
    // IMU data vectors
    yarp::os::Bottle* imu_left_bottle = NULL;
    yarp::os::Bottle* imu_right_bottle = NULL;
    yarp::os::Bottle* gyro;
    yarp::os::Bottle* acc;
    yarp::os::Bottle* magn;
    yarp::os::Bottle* ort;
    
    // Walking data vectors
    yarp::sig::Vector walk_com_vec;
    yarp::sig::Vector walk_left_foot_vec;
    yarp::sig::Vector walk_right_foot_vec;
    yarp::sig::Vector walk_joints_vec;
  
    Vector data_ll;
    Vector data_rl;
    Vector data_to;
    
    data_ll.resize(6);
    data_rl.resize(6);
    data_to.resize(3);
    
    if(dump_enc)
    {
      // encoders    
      driver->ienc_ll->getEncoders(data_ll.data());
      driver->ienc_rl->getEncoders(data_rl.data());
      driver->ienc_to->getEncoders(data_to.data());
          

      for(unsigned int i = 0; i < 3; i++)
      {
        fprintf(enc_data_file, "%e\t", data_to[i]);
      }
      for(unsigned int i = 0; i < 6; i++)
      {
        fprintf(enc_data_file, "%e\t", data_ll[i]);
        fprintf(enc_data_file, "%e\t", data_rl[i]);
      }
      fprintf(enc_data_file, "\n");
    }
    
    if(dump_enc_speed)
    {
      // encoders speed    
      driver->ienc_ll->getEncoderSpeeds(data_ll.data());
      driver->ienc_rl->getEncoderSpeeds(data_rl.data());
      driver->ienc_to->getEncoderSpeeds(data_to.data());
      
      for(unsigned int i = 0; i < 3; i++)
      {
        fprintf(enc_speed_data_file, "%e\t", data_to[i]);
      }
      for(unsigned int i = 0; i < 6; i++)
      {
        fprintf(enc_speed_data_file, "%e\t", data_ll[i]);
        fprintf(enc_speed_data_file, "%e\t", data_rl[i]);
      }
      fprintf(enc_speed_data_file, "\n");
    }
    
    if(dump_imu)
    {
      imu_left_bottle = imu_left_foot.read();
      imu_right_bottle = imu_right_foot.read();
      
      // we want to read the first 4 bottles
      if(imu_left_bottle->size() < 4)
      {
        cout << "Error in reading the IMU on LEFT leg" << endl;
      }
      if(imu_right_bottle->size() < 4)
      {
        cout << "Error in reading the IMU on RIGHT leg" << endl;
      }
      
      gyro = imu_left_bottle->get(0).asList()->get(0).asList()->get(0).asList()->get(0).asList();
      acc = imu_left_bottle->get(1).asList()->get(0).asList()->get(0).asList()->get(0).asList();
      magn = imu_left_bottle->get(2).asList()->get(0).asList()->get(0).asList()->get(0).asList();
      ort = imu_left_bottle->get(3).asList()->get(0).asList()->get(0).asList()->get(0).asList();
      
      for(unsigned int i = 0; i < 3; i++)
      {
        fprintf(imu_data_file, "%e, ", gyro->get(i).asDouble());
      }
      for(unsigned int i = 0; i < 3; i++)
      {
        fprintf(imu_data_file, "%e, ", acc->get(i).asDouble());
      }
      for(unsigned int i = 0; i < 3; i++)
      {
        fprintf(imu_data_file, "%e, ", magn->get(i).asDouble());
      }
      for(unsigned int i = 0; i < 2; i++)
      {
        fprintf(imu_data_file, "%e, ", ort->get(i).asDouble());
      }
      fprintf(imu_data_file, "%e\n", ort->get(2).asDouble());
    }
    
    if(dump_walking)
    {
      walk_com_vec =  *walking_com.read();
      walk_joints_vec =  *walking_joints.read();
      walk_left_foot_vec =  *walking_left_foot.read();
      walk_right_foot_vec =  *walking_right_foot.read();
      
      for(unsigned int i = 0; i < walk_com_vec.size(); i++)
      {
        fprintf(walking_com_file, "%e, ", walk_com_vec(i));
      }
      fprintf(walking_com_file, "\n");
      
      for(unsigned int i = 0; i < walk_joints_vec.size(); i++)
      {
        fprintf(walking_joints_file, "%e, ", walk_joints_vec(i));
      }
      fprintf(walking_joints_file, "\n");
      
      for(unsigned int i = 0; i < walk_left_foot_vec.size(); i++)
      {
        fprintf(walking_feet_file, "%e, ", walk_left_foot_vec(i));
      }
      for(unsigned int i = 0; i < walk_right_foot_vec.size(); i++)
      {
        fprintf(walking_feet_file, "%e, ", walk_right_foot_vec(i));
      }
      fprintf(walking_feet_file, "\n");
    }
    
    if(dump_robot)
    {
      if(dump_menc)
      {
        // motor encoders
        driver->imenc_ll->getMotorEncoders(data_ll.data());
        driver->imenc_rl->getMotorEncoders(data_rl.data());
        driver->imenc_to->getMotorEncoders(data_to.data());
          
        for(unsigned int i = 0; i < 3; i++)
        {
          fprintf(menc_data_file, "%e\t", (data_to[i] - enc_offset_to[i])/gear_ratio_to[i]);
        }
        for(unsigned int i = 0; i < 6; i++)
        {
          fprintf(menc_data_file, "%e\t", (data_ll[i] - enc_offset_ll[i])/gear_ratio_ll[i]);
          fprintf(menc_data_file, "%e\t", (data_rl[i] - enc_offset_rl[i])/gear_ratio_rl[i]);
        }
        fprintf(menc_data_file, "\n");
      }
      
      if(dump_menc_speed)
      {
        // motor encoders speeds
        driver->imenc_ll->getMotorEncoderSpeeds(data_ll.data());
        driver->imenc_rl->getMotorEncoderSpeeds(data_rl.data());
        driver->imenc_to->getMotorEncoderSpeeds(data_to.data());
          
        for(unsigned int i = 0; i < 3; i++)
        {
          fprintf(menc_speed_data_file, "%e\t", data_to[i]);
        }
        for(unsigned int i = 0; i < 6; i++)
        {
          fprintf(menc_speed_data_file, "%e\t", data_ll[i]);
          fprintf(menc_speed_data_file, "%e\t", data_rl[i]);
        }
        fprintf(menc_speed_data_file, "\n");
      }
      
      if(dump_currents)
      {
        // currents
        driver->iampl_ll->getCurrents(data_ll.data());
        driver->iampl_rl->getCurrents(data_rl.data());
        driver->iampl_to->getCurrents(data_to.data());
          
        for(unsigned int i  = 0; i < 3; i++)
        {
          fprintf(currents_file, "%e\t", data_to[i]);
        }
        for(unsigned int i  = 0; i < 6; i++)
        {
          fprintf(currents_file, "%e\t", data_ll[i]);
          fprintf(currents_file, "%e\t", data_rl[i]);
        }
        fprintf(currents_file, "\n");
      }
      
      if(dump_pwm)
      {
        // PWM
        double tempData = 0;
        for (int i = 0; i < 6; i++)
        {
          driver->iampl_ll->getPWM(i,&tempData);
          data_ll[i] = tempData;
          driver->iampl_rl->getPWM(i,&tempData);
          data_rl[i] = tempData;
        }
        for (int i = 0; i < 3; i++)
        {
          driver->iampl_to->getPWM(i,&tempData);
          data_to[i] = tempData;
        }
        
        for(unsigned int i = 0; i < 3; i++)
        {
          fprintf(pwm_file, "%e\t", data_to[i]);
        }
        for(unsigned int i = 0; i < 6; i++)
        {
          fprintf(pwm_file, "%e\t", data_ll[i]);
          fprintf(pwm_file, "%e\t", data_rl[i]);
        }
        fprintf(pwm_file, "\n");
      }
    }
}

void WorkingThread::run()
{
  mutex.wait();
  double current_time = yarp::os::Time::now();
  static double last_time = yarp::os::Time::now();
  if (actions.current_status == ACTION_IDLE)
  {
    last_time = current_time;
  }
  else if (actions.current_status == ACTION_RUNNING)
  {
    actions.current_action++;
    dump_data(actions.current_action);
  }
  else if (actions.current_status == ACTION_START)
  {
    cout << "Starting data dumping." << endl;
    dump_data(0);
    actions.current_status = ACTION_RUNNING;
    actions.current_action++;
  }
  else
  {
    printf("ERROR: unknown current_status\n");
  }
  
  mutex.post();
}
