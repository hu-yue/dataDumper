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
    feetOrtTest = false;
    
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
    
    // Feet ort test
    if(l_foot_ort_file!=NULL)
      fclose(l_foot_ort_file);
    if(r_foot_ort_file!=NULL)
      fclose(r_foot_ort_file);
    if(l_foot_ort_imu_file!=NULL)
      fclose(l_foot_ort_imu_file);
    if(r_foot_ort_imu_file!=NULL)
      fclose(r_foot_ort_imu_file);
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
      imu_left_foot.open("/" + name + "/left_leg/imu/measures:i");
      imu_right_foot.open("/" + name + "/right_leg/imu/measures:i");
      
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
    
    //*** Feet ort test ****
    if(feetOrtTest)
    {
      if(!model_loader.loadModelFromFile(model_name)){
        cout <<"Error while loading the model from " << model_name << endl;
        return false;
      }
      vector<string> legJoints;//"l_hip_pitch", "l_hip_roll", "l_hip_yaw", "l_knee", "l_ankle_pitch", "l_ankle_roll", "r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll"
      legJoints.resize(12);
      legJoints[0] = "l_hip_pitch";
      legJoints[1] = "l_hip_roll";
      legJoints[2] = "l_hip_yaw";
      legJoints[3] = "l_knee";
      legJoints[4] = "l_ankle_pitch";
      legJoints[5] = "l_ankle_roll";
      legJoints[6] = "r_hip_pitch";
      legJoints[7] = "r_hip_roll";
      legJoints[8] = "r_hip_yaw";
      legJoints[9] = "r_knee";
      legJoints[10] = "r_ankle_pitch";
      legJoints[11] = "r_ankle_roll";
      model_loader.loadReducedModelFromFullModel(model_loader.model(),legJoints);
      m_kinDyn.loadRobotModel(model_loader.model());
      m_kinDyn.setFloatingBase("root_link");
      cout << "Degrees of freedom: " << m_kinDyn.getNrOfDegreesOfFreedom() << endl;
      
      // set imuToStrain
      imuToStrain.zero();
      imuToStrain.setVal(0,0,-1);
      imuToStrain.setVal(1,1,1);
      imuToStrain.setVal(2,2,-1);
      
      lEarthToBase = iDynTree::Rotation::Identity();
      rEarthToBase = iDynTree::Rotation::Identity();
    }
    
    return true;
}

void WorkingThread::computeFeetOrt(int j, yarp::sig::Vector& lleg, yarp::sig::Vector& rleg, yarp::os::Bottle* lort, yarp::os::Bottle* rort)
{
  iDynTree::VectorDynSize legsState(12);
  iDynTree::VectorDynSize legsStateVel(12);
  legsStateVel.zero();
  iDynTree::Vector3 gravity;
  gravity.zero();
  gravity(2) = -9.81;
  for(unsigned int i = 0; i < 6; i++)
  {
    legsState(i) = iDynTree::deg2rad(lleg[i]);
    legsState(i+6) = iDynTree::deg2rad(rleg[i]);
  }
  
  m_kinDyn.setRobotState(iDynTree::Transform::Identity(), legsState, iDynTree::Twist::Zero(), legsStateVel, gravity);
  
  iDynTree::Vector3 posZero;
  posZero.zero();
  
  iDynTree::Rotation lStrainToBase = iDynTree::Rotation::Identity();
  iDynTree::Rotation lIMUtoEarth = iDynTree::Rotation::Identity();
  iDynTree::Rotation lStrainToBaseWIMU = iDynTree::Rotation::Identity();
  
  iDynTree::Position lStrainPos;
  iDynTree::Vector3 lStrainRot;
  iDynTree::Position lStrainPosWIMU;
  iDynTree::Vector3 lStrainRotWIMU;
  
  iDynTree::Rotation rStrainToBase = iDynTree::Rotation::Identity();
  iDynTree::Rotation rIMUtoEarth = iDynTree::Rotation::Identity();
  iDynTree::Rotation rStrainToBaseWIMU = iDynTree::Rotation::Identity();
  
  iDynTree::Position rStrainPos;
  iDynTree::Vector3 rStrainRot;
  iDynTree::Position rStrainPosWIMU;
  iDynTree::Vector3 rStrainRotWIMU;
  
  lStrainToBase = m_kinDyn.getRelativeTransform("root_link","l_foot_ft_sensor").getRotation();
  lIMUtoEarth = iDynTree::Rotation::RPY(iDynTree::deg2rad(lort->get(0).asDouble()),iDynTree::deg2rad(lort->get(1).asDouble()),iDynTree::deg2rad(lort->get(2).asDouble()));
  //lIMUtoEarth = lIMUtoEarth.inverse();
  // compute Earth to Base in the first run
  if(j == 0)
  {
    lEarthToBase = lStrainToBase*imuToStrain*lIMUtoEarth.inverse();
  }
  lStrainToBaseWIMU = lEarthToBase*lIMUtoEarth*imuToStrain.inverse();
  
  lStrainPos = m_kinDyn.getRelativeTransform("root_link","l_foot_ft_sensor").getPosition();
  lStrainRot = lStrainToBase.asRPY();
  lStrainRotWIMU = lStrainToBaseWIMU.asRPY();
  
  cout << "IMU left foot: " << lort->toString() << endl;
  cout << "IMU to Earth left: " << lIMUtoEarth.asRPY().toString() << endl;
  cout << "Left Kinematics; fromIMU: " << lStrainToBase.asRPY().toString() << "; " << lStrainToBaseWIMU.asRPY().toString() << endl;
  
  for(unsigned int i = 0; i < 3; i++)
  {
    fprintf(l_foot_ort_file, "%e, ", lStrainRot(i));
  }
  fprintf(l_foot_ort_file, "\n");
  for(unsigned int i = 0; i < 3; i++)
  {
    fprintf(l_foot_ort_imu_file, "%e, ", lStrainRotWIMU(i));
  }
  fprintf(l_foot_ort_imu_file, "\n");
  
  rStrainToBase = m_kinDyn.getRelativeTransform("root_link","r_foot_ft_sensor").getRotation();
  rIMUtoEarth = iDynTree::Rotation::RPY(iDynTree::deg2rad(rort->get(0).asDouble()),iDynTree::deg2rad(rort->get(1).asDouble()),iDynTree::deg2rad(rort->get(2).asDouble()));
  //rIMUtoEarth = rIMUtoEarth.inverse();
  // compute Earth to Base in the first run
  if(j == 0)
  {
    rEarthToBase = rStrainToBase*imuToStrain*rIMUtoEarth.inverse();
  }
  rStrainToBaseWIMU = rEarthToBase*rIMUtoEarth*imuToStrain.inverse();
  
  rStrainPos = m_kinDyn.getRelativeTransform("root_link","r_foot_ft_sensor").getPosition();
  rStrainRot = rStrainToBase.asRPY();
  rStrainRotWIMU = rStrainToBaseWIMU.asRPY();
  
  cout << "IMU right foot: " << rort->toString() << endl;
  cout << "IMU to Earth right: " << rIMUtoEarth.asRPY().toString() << endl;
  cout << "Right Kinematics; fromIMU: " << rStrainToBase.asRPY().toString() << "; " << rStrainToBaseWIMU.asRPY().toString() << endl << endl;
  
  for(unsigned int i = 0; i < 3; i++)
  {
    fprintf(r_foot_ort_file, "%e, ", rStrainRot(i));
  }
  fprintf(r_foot_ort_file, "\n");
  for(unsigned int i = 0; i < 3; i++)
  {
    fprintf(r_foot_ort_imu_file, "%e, ", rStrainRotWIMU(i));
  }
  fprintf(r_foot_ort_imu_file, "\n");
}

void WorkingThread::dump_data(int j)
{
    // IMU data vectors
    yarp::os::Bottle* imu_left_bottle = NULL;
    yarp::os::Bottle* imu_right_bottle = NULL;
    yarp::os::Bottle* lgyro;
    yarp::os::Bottle* lacc;
    yarp::os::Bottle* lmagn;
    yarp::os::Bottle* lort;
    yarp::os::Bottle* rgyro;
    yarp::os::Bottle* racc;
    yarp::os::Bottle* rmagn;
    yarp::os::Bottle* rort;
    
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

    if(dump_imu)
    {
      imu_left_bottle = imu_left_foot.read(false);
      imu_right_bottle = imu_right_foot.read(false);
      
      // we want to read the first 4 bottles
      if(imu_left_bottle->size() < 4)
      {
        cout << "Error in reading the IMU on LEFT leg" << endl;
      }
      if(imu_right_bottle->size() < 4)
      {
        cout << "Error in reading the IMU on RIGHT leg" << endl;
      }
      
      lgyro = imu_left_bottle->get(0).asList()->get(0).asList()->get(0).asList();
      lacc = imu_left_bottle->get(1).asList()->get(0).asList()->get(0).asList();
      lmagn = imu_left_bottle->get(2).asList()->get(0).asList()->get(0).asList();
      lort = imu_left_bottle->get(3).asList()->get(0).asList()->get(0).asList();
      
      for(unsigned int i = 0; i < 3; i++)
      {
        fprintf(imu_data_file, "%e, ", lgyro->get(i).asDouble());
      }
      for(unsigned int i = 0; i < 3; i++)
      {
        fprintf(imu_data_file, "%e, ", lacc->get(i).asDouble());
      }
      for(unsigned int i = 0; i < 3; i++)
      {
        fprintf(imu_data_file, "%e, ", lmagn->get(i).asDouble());
      }
      for(unsigned int i = 0; i < 3; i++)
      {
        fprintf(imu_data_file, "%e, ", lort->get(i).asDouble());
      }
      
      rgyro = imu_right_bottle->get(0).asList()->get(0).asList()->get(0).asList();
      racc = imu_right_bottle->get(1).asList()->get(0).asList()->get(0).asList();
      rmagn = imu_right_bottle->get(2).asList()->get(0).asList()->get(0).asList();
      rort = imu_right_bottle->get(3).asList()->get(0).asList()->get(0).asList();
      
      for(unsigned int i = 0; i < 3; i++)
      {
        fprintf(imu_data_file, "%e, ", rgyro->get(i).asDouble());
      }
      for(unsigned int i = 0; i < 3; i++)
      {
        fprintf(imu_data_file, "%e, ", racc->get(i).asDouble());
      }
      for(unsigned int i = 0; i < 3; i++)
      {
        fprintf(imu_data_file, "%e, ", rmagn->get(i).asDouble());
      }
      for(unsigned int i = 0; i < 2; i++)
      {
        fprintf(imu_data_file, "%e, ", rort->get(i).asDouble());
      }
      fprintf(imu_data_file, "%e\n", rort->get(2).asDouble());
    }
    
    if(dump_enc || feetOrtTest)
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
      
      if(feetOrtTest && dump_imu)
        computeFeetOrt(j,data_ll,data_rl,lort,rort);
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
