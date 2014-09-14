require 'vizkit'
include Orocos

Orocos.initialize

widget = Vizkit.load "simulator.ui"

Orocos.run "AvalonSimulation", "asv_localization::Task"=> "asv_localization"  , "sonar_feature_estimator::Task"=> "sonar_feature_estimator",
    :wait => 10000, :valgrind => false, :valgrind_options => ['--undef-value-errors=no'] do 
    
    simulation = TaskContext.get 'avalon_simulation'
  
    simulation.apply_conf_file("/home/fabio/avalon/bundles/avalon/config/orogen/simulation::Mars.yml")
    #simulation.initial_scenes = []
    #simulation.initial_scene = "/home/fabio/avalon/simulation/orogen/avalon_simulation/configuration/testhalle.scn"

    simulation.configure
    simulation.start
    
require 'actuators'
values = ActuatorsConfig.new()    
    
    asv_actuators = TaskContext.get 'asv_actuators'
    asv_actuators.node_name = "asv"
    asv_actuators.amount_of_actuators = values.asv_amount_of_actuators
    asv_actuators.maximum_thruster_force = values.asv_maximum_thruster_force    
    asv_actuators.thruster_position = values.asv_thruster_position    
    asv_actuators.thruster_direction = values.asv_thruster_direction      
    asv_actuators.configure
    asv_actuators.start
    asv_writer = asv_actuators.command.writer
    imu = TaskContext.get 'imu'
    imu.name = "asv"
    imu.configure
    imu.start
    
    sonar = TaskContext.get 'sonar'
    sonar.node_name = "asv_laser"
    sonar.left_limit = Math::PI
    sonar.right_limit = -Math::PI
    sonar.resolution = 0.1
    sonar.maximum_distance = 100.0
    sonar.ping_pong_mode = false
    sonar.configure
    sonar.start    
    
    feature = Orocos::TaskContext.get 'sonar_feature_estimator'
    feature.derivative_history_length = 1
    feature.plain_threshold = 0.5
    feature.signal_threshold = 0.7
    feature.enable_debug_output = true 
    
    sonar.sonar_beam.connect_to feature.sonar_input
    
    feature.configure
    feature.start
    
    
    asv_localization = TaskContext.get 'asv_localization'

    imu.pose_samples.connect_to asv_localization.gps_samples do |sample|
      sample.position[0] = sample.position[0] + Random.rand(15) - 7.5
      sample.position[1] = sample.position[1] + Random.rand(15) - 7.5
      sample
    end
    
    imu.calibrated_sensors.connect_to asv_localization.imu_samples      
    imu.pose_samples.connect_to asv_localization.orientation_samples
    feature.new_feature.connect_to asv_localization.laser_samples
    #asv_actuators.pose_samples.connect_to asv_localization.velocity_samples
    
    asv_localization.gps_error = 10
    asv_localization.acceleration_error = 2.5
    asv_localization.velocity_error = 0.2
    asv_localization.laser_variance = 0.2

    asv_localization.gps_reject_threshold = 999999999999999
    asv_localization.velocity_reject_threshold = 9999999999999999

    asv_localization.gps_period = 0.01
    asv_localization.imu_period = 0.01
    asv_localization.ori_period = 0.01
    asv_localization.laser_period = 0.1
    asv_localization.max_delay = 0.5

    asv_localization.initial_gps_origin = false
    asv_localization.estimate_velocity = true
    asv_localization.velocity_estimation_count = 20
    
    asv_localization.laser_min_range = 1.0
    asv_localization.laser_max_range = 50.0 

    asv_localization.use_gps_velocity = 1
    asv_localization.enframe_to_nwframe = false
    asv_localization.yaml_file = "#{ENV['AUTOPROJ_CURRENT_ROOT']}/auv_avalon/orogen/asv_localization/maps/nurc_sim.yml"
    
    asv_localization.configure
    asv_localization.start
    
    widget.joystick1.connect(SIGNAL('axisChanged(double,double)'))do |x,y|
        sample = asv_writer.new_sample
        sample.time = Time.now 
        0.upto(3) do
            sample.mode << :DM_PWM
            sample.target << 0;
        end
        sample.target[0] = x
        sample.target[1] = x
        sample.target[2] = -y
        sample.target[3] = y
        asv_writer.write sample
    end

    #Vizkit.display simulation
    #Vizkit.display sonar
    #Vizkit.display asv_actuators
    Vizkit.display imu
    Vizkit.display asv_localization
    #Vizkit.display feature
    widget.show 
    Vizkit.exec


end

