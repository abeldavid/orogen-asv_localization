/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace asv_localization;

Task::Task(std::string const& name)
    : TaskBase(name)
{
  
  base::Vector3d vec;
  vec << 0.0, 0.0, 0.0;
  _laser_translation.set(vec);
  _laser_rotation_euler.set(vec);
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
  
  base::Vector3d vec;
  vec << 0.0, 0.0, 0.0;
  _laser_translation.set(vec);
  _laser_rotation_euler.set(vec);  
}

Task::~Task()
{
}

void Task::gps_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_samples_sample)
{
 //std::cout << "GPS callback" << std::endl;
  
  base::Vector3d pos;
  
  //Convert EN-Frame to NW-Frame
  pos[0] = gps_samples_sample.position[1];
  pos[1] = -gps_samples_sample.position[0];
  pos[2] = 0.0; //We asume, that we are only on the surface
  
  pos = pos - (ekf.getRotation().inverse() * relativeGps);
  
  base::samples::RigidBodyState rbs = gps_samples_sample;
  rbs.position = pos;
  
  //If we have a initial position, then observe. Else initialize position
  if(firstPositionRecieved){
     
    if(firstOrientationRecieved){       
      
      pos = pos - firstGpsSample.position;
      
      base::Matrix3d covariance = base::Matrix3d::Identity() * _gps_error.get();
      
      //Observe
      if(ekf.positionObservation( pos, covariance, _gps_reject_threshold.get() )){
	std::cout << "Rejected GPS-Sample" << std::endl;
      }else if(_estimate_velocity.get() ){ //When the observation was valid and we want to estimate the velocity ->
	
	rbs.orientation = ekf.getRotation();
	gpsPositions.push_back(rbs);
		
	if(gpsPositions.full() ){
	  
	  base::samples::RigidBodyState oldSample = *(gpsPositions.begin()+1); //Get the first sample
	  //Get the middle orientation between the actual arientation and the oldest orientation
	  base::Orientation avg_ori = (gpsPositions.begin() + (_velocity_estimation_count/2))->orientation;
	  
			  
	  base::Vector3d relPos = avg_ori.inverse() * (pos - oldSample.position);		  
	  
	  base::Vector3d vel = relPos / (ts.toSeconds() - oldSample.time.toSeconds());	  
	 
	  /*	  
	  std::cout << "Yaw:      " << base::getYaw(ekf.getRotation()) << std::endl;
	  std::cout << "Avg-yaw:  " << base::getYaw(avg_ori) << std::endl;
	  std::cout << "Act. pos: " << pos.transpose() << std::endl;
	  std::cout << "Old. pos: " << oldSample.position.transpose() << std::endl;
	  std::cout << "DeltaPos: " << (pos - oldSample.position).transpose() << std::endl;
	  std::cout << "Velocity: " << vel.transpose() << std::endl;
	  std::cout << "Delta t:  " << (ts.toSeconds() - oldSample.time.toSeconds()) << std::endl;
	  std::cout << "--------------------------" << std::endl; */
	  
	  base::Matrix3d covar = base::Matrix3d::Identity() * _velocity_error.get() ;
	  actualVelocity = vel;
	  
	  //Velocity-Observation
	  if(ekf.velocityObservation(vel, covar, _velocity_reject_threshold.get()))
	    std::cout << "Rejected Velocity" << std::endl;	    

      } 
      }
      
    }    
    
  }else{ //First gps-sample
    
    base::Matrix3d covariance = base::Matrix3d::Identity() * _gps_error.get();
    
    
    lastGpsSample = rbs;
    firstGpsSample = rbs;
    firstPositionRecieved = true;
    
    //Use 0,0,0 as origin
    if(!_initial_gps_origin.get())
      firstGpsSample.position = base::Vector3d::Zero();
    
    ekf.setPosition( pos , covariance);

    std::cout << "Initialize Position";
  }
  
  
  lastGpsTime = ts;
  
}

void Task::imu_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample)
{  
  
  if(!lastImuTime.isNull() && firstOrientationRecieved){
    
    base::samples::IMUSensors sample = imu_samples_sample;
    
    double pitch = base::getPitch(lastOrientation) - 0.1; //Offset for imu calibration TODO calibrate imu!!
    double roll = base::getRoll(lastOrientation) ;//+ 0.034;
    const double gravity = 9.871; 
    
    //The asv-imu is turned to 90 degrees -> change x and y acceleration
    //Use pitch and roll to eliminate gravity
    sample.acc[0] = -imu_samples_sample.acc[1] - gravity * sin(pitch);
    sample.acc[1] = imu_samples_sample.acc[0] + gravity * sin(roll);  
    sample.acc[2] = 0.0; //gravity; //As we only drive on the surface, there is no z-acceleration
    
    //std::cout << "Pitch: " << pitch << " Roll: " << roll << std::endl;
    //std::cout << "New Acceleration: " << sample.acc.transpose() << std::endl;
    
    double dt = ts.toSeconds() - lastImuTime.toSeconds();
    
    //Covariance
    Eigen::Matrix<double, 9, 9 >  process_noise = Eigen::Matrix<double, 9, 9>::Zero();
    //Acceleration noise
    process_noise(0,0) = _acceleration_error.get();
    process_noise(1,1) = _acceleration_error.get();
    process_noise(2,2) = _acceleration_error.get();
    //Velocity noise: sum up covariance 
    process_noise(3,3) = _acceleration_error.get() * dt;
    process_noise(4,4) = _acceleration_error.get() * dt;
    process_noise(5,5) = _acceleration_error.get() * dt;
    //Position noise: sum up velocity covariancee
    process_noise(6,6) = _acceleration_error.get() * dt * dt;
    process_noise(7,7) = _acceleration_error.get() * dt * dt;
    process_noise(8,8) = _acceleration_error.get() * dt * dt;
    
    ekf.predict( sample.acc, dt, process_noise);
      
  }
  
  lastImuTime = ts;
  
}

void Task::orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample)
{    
  //std::cout << "Orientation callback" << std::endl;
  firstOrientationRecieved = true;
  
  base::Orientation ori = imuRotation * orientation_samples_sample.orientation;
  lastOrientation = ori;

  ekf.setRotation(ori);
  
  //Write out actual state  
  if(firstPositionRecieved){
    if(base::Time::now().toSeconds()	- lastGpsTime.toSeconds() < _gps_timeout.get()){    
      
      rbs.position = ekf.getPosition();
      rbs.position[2] = 0.0;
      
      if(_use_gps_velocity.get() == 1)
	rbs.velocity = actualVelocity;
      else if(_use_gps_velocity.get() == 0)
	rbs.velocity = base::Vector3d::Zero();
      else if(_use_gps_velocity.get() == 3){
	rbs.velocity = base::Vector3d::Zero();
	rbs.velocity[0] = actualVelocity[0];
      }else      
	rbs.velocity = ekf.getVelocity();
      
      sum += rbs.velocity[1] * rbs.velocity[1];
      count++;
      //std::cout << "Sum vel: " << sum << " Varianz: " << sum/count << " Std-Abweichung: " << std::sqrt(sum/count) <<  std::endl;
      rbs.orientation = lastOrientation;
      rbs.angular_velocity = orientation_samples_sample.angular_velocity;
      rbs.time = base::Time::now();
      rbs.cov_position = ekf.getPositionCovariance();
      rbs.cov_velocity = ekf.getVelocityCovariance();
      _pose_samples.write(rbs);
    }else{
      std::cerr << "GPS-Timeout" << std::endl;
    }  
    
  }  
  
  
}


void Task::velocity_samplesCallback(const:: base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample){
  //std::cout << "Velocity callback" << std::endl;
  /*if(!lastVelocityTime.isNull()){
    double dt = ts.toSeconds() - lastVelocityTime.toSeconds();
    
    base::Vector3d acc;
    acc[0] = (velocity_samples_sample.velocity[0] - lastVelocitySample.velocity[0]) / dt;
    acc[1] = (velocity_samples_sample.velocity[1] - lastVelocitySample.velocity[1]) / dt;
    acc[2] = (velocity_samples_sample.velocity[2] - lastVelocitySample.velocity[2]) / dt;
    
    
    Eigen::Matrix<double, 9, 9 >  process_noise = Eigen::Matrix<double, 9, 9>::Zero();
    process_noise(0,0) = _acceleration_error.get();
    process_noise(1,1) = _acceleration_error.get();
    process_noise(2,2) = _acceleration_error.get();
    
    process_noise(3,3) = _acceleration_error.get() * dt;
    process_noise(4,4) = _acceleration_error.get() * dt;
    process_noise(5,5) = _acceleration_error.get() * dt;
    
    process_noise(6,6) = _acceleration_error.get() * dt * dt;
    process_noise(7,7) = _acceleration_error.get() * dt * dt;
    process_noise(8,8) = _acceleration_error.get() * dt * dt;
    
    ekf.predict( acc, dt, process_noise);
    
    
  }*/
  
  base::Matrix3d cov = base::Matrix3d::Identity() * _velocity_error.get();
  
  if( !ekf.velocityObservation( velocity_samples_sample.velocity, cov, _velocity_reject_threshold.get()))
    std::cout << "Rejected velocity" << std::endl;
  
  
  lastVelocityTime = ts;
  lastVelocitySample = velocity_samples_sample;
  
}  


void Task::laser_samplesCallback(const base::Time &ts, const base::samples::LaserScan &laser_samples_sample){
  
  if(firstOrientationRecieved && firstPositionRecieved){
  
    double angle = laser_samples_sample.start_angle;
    double res = laser_samples_sample.angular_resolution;
    
    for(std::vector<uint32_t>::const_iterator it = laser_samples_sample.ranges.begin(); it != laser_samples_sample.ranges.end(); it++){
      
      double dist = *it / 1000.0;
      
        if(dist > _laser_min_range && dist < _laser_max_range){
        
        base::Quaterniond rot =   rbs.orientation * laserRotation * Eigen::AngleAxisd( angle, Eigen::Vector3d::UnitZ() );
        
        double abs_yaw = base::getYaw(rot);
        
        double mod = std::fmod(abs_yaw, M_PI * 0.5);

        if( mod < 0.1 || mod > (M_PI * 0.5) - 0.1 ){
          
          boost::tuple<uw_localization::Node*, double, Eigen::Vector3d> distance = 
                        node_map->getNearestDistance("root.wall", rot * base::Vector3d(1.0, 0,0)  , ekf.getPosition());
          
          double sim_dist = distance.get<1>();
        
          if(distance.get<1>() == INFINITY)
            continue;
          
          base::Vector3d meas_pos = distance.get<2>() - (rot * base::Vector3d(dist, 0.0, 0.0) );
          meas_pos(2) = 0.0;
          
          if(ekf.positionObservation( meas_pos, base::Matrix3d::Identity() * _laser_variance.get()  , _gps_reject_threshold.get() )){
            std::cout << "Rejected laserscan" << std::endl;
          }
          
        }
        
     }
      
      
      angle += laser_samples_sample.angular_resolution;
      
    }
    
  }
  
}

void Task::thruster_samplesCallback(const base::Time &ts, const base::samples::Joints &thruster_samples_sample){
  
  if(!lastThrusterTime.isNull()){
    
    base::samples::Joints joint = thruster_samples_sample;
    double size = joint.size();
    
    if(size < 6){
      joint.elements.resize(6);
      
      for(; size < 6; size++){
        joint.elements[size].raw = 0.0;
      }
      
    }
    
    
    double dt = ts.toSeconds() - lastThrusterTime.toSeconds();
    
    if(dt > 0.0 && dt < 2.0){
      
      base::Vector6d Xt;
     
      
      Xt.block<3,1>(0,0) = lastVelocity;;
      Xt.block<3,1>(3,0) = base::Vector3d(0.0, 0.0, 0.0);
      base::Vector6d V = motion_model->transition(Xt, dt, joint);
      lastVelocity = V.block<3,1>(0,0);
      
      if(base::samples::RigidBodyState::isValidValue(lastVelocity) ){
      
        ekf.velocityObservation(lastVelocity, base::Matrix3d::Identity() * _model_variance.get(), _velocity_reject_threshold.get());
      
      }else{
        lastVelocity = base::Vector3d::Zero();
      }
      
    }
  
  
  }
  
  lastThrusterTime = ts;
  
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.




bool Task::configureHook()
{
    
    if (! RTT::TaskContext::configureHook())
        return false;
    
    firstPositionRecieved = false;
    firstOrientationRecieved = false;
    lastImuTime = base::Time();
    lastGpsTime = base::Time();
    lastVelocityTime = base::Time();
    samplesCount = 0;
    
    orientationID = -1;
    imuID = -1;
    gpsID = -1;
    velocityID = -1;
    actualVelocity = base::Vector3d::Zero();
    
    imuRotation = base::Quaterniond(Eigen::AngleAxisd(_imu_rotation.get() , Eigen::Vector3d::UnitZ()) );
    
    gpsPositions = boost::circular_buffer<base::samples::RigidBodyState>(_velocity_estimation_count.get());
    
    laserRotation = base::Quaterniond( Eigen::AngleAxisd( _laser_rotation_euler.get()(0), Eigen::Vector3d::UnitX() )
                                      * Eigen::AngleAxisd( _laser_rotation_euler.get()(1), Eigen::Vector3d::UnitY()  ) 
                                      * Eigen::AngleAxisd( _laser_rotation_euler.get()(2), Eigen::Vector3d::UnitZ()  ) );
                                      
    laserTranslation = _laser_translation.get();    
    
    relativeGps = _relative_gps_position.get();
    count = 0.0;
    sum = 0.0;
    
    strAligner.setTimeout( base::Time::fromSeconds(_max_delay.get()));
    const double buffer_size_factor = 2.0;
    
    //Register data streams at the stream aligner
    if( _gps_samples.connected()){
      
      gpsID = strAligner.registerStream<base::samples::RigidBodyState>(
	    boost::bind( &asv_localization::Task::gps_samplesCallback, this, _1, _2 ),
	    buffer_size_factor * std::ceil( _max_delay.get() / _gps_period.get() ),
	    base::Time::fromSeconds( _gps_period.get() ) );      
    }
    
    
    if( _orientation_samples.connected()){
      
      orientationID = strAligner.registerStream<base::samples::RigidBodyState>(
	    boost::bind( &asv_localization::Task::orientation_samplesCallback, this, _1, _2 ),
	    buffer_size_factor * std::ceil( _max_delay.get() / _ori_period.get() ),
	    base::Time::fromSeconds( _ori_period.get() ) );      
    }
    
    if( _velocity_samples.connected()){
      
      velocityID = strAligner.registerStream<base::samples::RigidBodyState>(
	    boost::bind( &asv_localization::Task::velocity_samplesCallback, this, _1, _2 ),
	    buffer_size_factor * std::ceil( _max_delay.get() / _vel_period.get() ),
	    base::Time::fromSeconds( _vel_period.get() ) );      
    }
    
    if( _imu_samples.connected()){
      
      imuID = strAligner.registerStream<base::samples::IMUSensors>(
	    boost::bind( &asv_localization::Task::imu_samplesCallback, this, _1, _2 ),
	    buffer_size_factor * std::ceil( _max_delay.get() / _imu_period.get() ),
	    base::Time::fromSeconds( _imu_period.get() ) );      
    }
    
    if(_laser_samples.connected()){
      
      laserID = strAligner.registerStream<base::samples::LaserScan>(
            boost::bind( &asv_localization::Task::laser_samplesCallback, this, _1, _2 ),
            buffer_size_factor * std::ceil( _max_delay.get() / _laser_period.get() ),
            base::Time::fromSeconds( _laser_period.get() ) ); 
      
    }
    
    if(_thruster_samples.connected()){
      
      thrusterID = strAligner.registerStream<base::samples::Joints>(
            boost::bind( &asv_localization::Task::thruster_samplesCallback, this, _1, _2 ),
            buffer_size_factor * std::ceil( _max_delay.get() / _thruster_period.get() ),
            base::Time::fromSeconds( _thruster_period.get() ) ); 
      
    }    
    
    
     if(_yaml_file.value().empty()){
       std::cout << "ERROR: No yaml-map given" << std::endl;       
     }else{  
      std::cout << "Setup NodeMap" << std::endl;
      node_map = new uw_localization::NodeMap();
      if(!node_map->fromYaml(_yaml_file.value())){
        std::cerr << "ERROR: No map could be load " << _yaml_file.value().c_str() << std::endl;
      }
      
     }
    
    initMotionModel();
    
    return true;
    
}



bool Task::startHook()
{
    
    if (! RTT::TaskContext::startHook())
        return false;
    

    //ekf = pose_ekf::KFD_PosVelAcc();
    lastGpsTime = base::Time::now();
    
    return true;
    
}



void Task::updateHook()
{

    RTT::TaskContext::updateHook();
    
    //Collect new input data and push them into the stream aligner
    
    if(orientationID != -1){
      base::samples::RigidBodyState orientation;
      while( _orientation_samples.read(orientation) == RTT::NewData )
      {
	  strAligner.push( orientationID, orientation.time, orientation );
	  
      }
    }  

    
    if(gpsID != -1){
      base::samples::RigidBodyState gps_sample;
      while( _gps_samples.read(gps_sample) == RTT::NewData )
      {
	  strAligner.push( gpsID, gps_sample.time, gps_sample );
	  
      }
    }  
    
    if(velocityID != -1){
      base::samples::RigidBodyState velocity_sample;
      while( _velocity_samples.read(velocity_sample) == RTT::NewData )
      {
	  strAligner.push( velocityID, velocity_sample.time, velocity_sample );
	  
      }
    }  
    
    if(imuID != -1){
      base::samples::IMUSensors imu_sample;
      while( _imu_samples.read(imu_sample) == RTT::NewData )
      {
	  strAligner.push( imuID, imu_sample.time, imu_sample );
	  
      }
    }
    
    if(laserID != -1){
      base::samples::LaserScan laser_sample;
      while( _laser_samples.read(laser_sample) == RTT::NewData ){
        
        strAligner.push( laserID, laser_sample.time, laser_sample);
        
      }
      
    }
    
    if(thrusterID != -1){
      base::samples::Joints joints_sample;
      while(_thruster_samples.read(joints_sample) == RTT::NewData){
        strAligner.push( thrusterID, joints_sample.time, joints_sample);
        
      }
    }
    
    //Excecute stream-aligner steps
    while(strAligner.step());  
    
    
    //std::cout << "Update" << std::endl;
    _stream_aligner_status.write(strAligner.getStatus());
        
}



void Task::errorHook()
{
    
    RTT::TaskContext::errorHook();
    

    

    
}



void Task::stopHook()
{
    
    RTT::TaskContext::stopHook();
    

    

    
}



void Task::cleanupHook()
{
    
    RTT::TaskContext::cleanupHook();
    

    

    
}

void Task::initMotionModel()
{
  
  if(_thruster_samples.connected() ){
    motion_model = new uw_localization::UwMotionModel();
    
    uw_localization::UwVehicleParameter param;
    param.Length = 0.0;
    param.Radius = 0.0;
    param.Mass = _vehicle_mass.get();
    
    base::Vector6d thrust;
    
    for(int i = 0; i < 6; i++){
      
      if(i < _thruster_coef.get().size()){
        thrust(i) = _thruster_coef.get()[i];
      }
      else{
        thrust(i) = 0.0;
      }
      
    }
      
      param.ThrusterCoefficient = thrust;
      param.LinearThrusterCoefficient = base::Vector6d::Zero();
      param.SquareThrusterCoefficient = base::Vector6d::Zero();
      param.ThrusterVoltage = _thruster_voltage.get();
      
      param.floating = true;
      
      if(_damping_coefficients.get().size() >= 2){
        
        param.DampingX << 0.0, _damping_coefficients.get()[0];
        param.DampingY << 0.0, _damping_coefficients.get()[1];
      }
      else{
        param.DampingX << 0.0, 10;
        param.DampingY << 0.0, 10;        
        
      }
      param.DampingZ << 9999, 9999;
      
      Eigen::Matrix<double, 6, 3, Eigen::DontAlign> tcm = Eigen::Matrix<double, 6, 3, Eigen::DontAlign>::Zero();
      
      if(_tcm.get().size() == _number_of_thruster.get() * 2){
        
        for(int t = 0; t < 6; t++){
          
          if(t < _number_of_thruster.get()){
            
            tcm(t,0) = _tcm.get()[t];
            tcm(t,1) = _tcm.get()[t + _number_of_thruster.get()];
            
          }
          
        }        
        
      }

      param.TCM = tcm;  
      lastVelocity = base::Vector3d::Zero();
  } 
  
}


