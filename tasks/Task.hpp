/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ASV_LOCALIZATION_TASK_TASK_HPP
#define ASV_LOCALIZATION_TASK_TASK_HPP

#include "asv_localization/TaskBase.hpp"
#include "pose_ekf/KFD_PosVelAcc.hpp"
#include "aggregator/StreamAligner.hpp"
#include "uw_localization/maps/node_map.hpp"
#include <uw_localization/model/uw_motion_model.hpp>
#include <boost/circular_buffer.hpp>

namespace asv_localization {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','asv_localization::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        void gps_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &gps_samples_sample);
        void imu_samplesCallback(const base::Time &ts, const ::base::samples::IMUSensors &imu_samples_sample);
        void orientation_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_samples_sample);
	 void velocity_samplesCallback(const base::Time &ts, const ::base::samples::RigidBodyState &velocity_samples_sample);
        void laser_samplesCallback(const base::Time &ts, const base::samples::LaserScan &laser_samples_sample);
        void thruster_samplesCallback(const base::Time &ts, const base::samples::Joints &thruster_samples_sample);
        
        void initMotionModel();

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "asv_localization::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
	
      private:
  
	pose_ekf::KFD_PosVelAcc ekf; //The used kalman-filter
	
	bool firstPositionRecieved; //True, when we recieved an gps-sample
	bool firstOrientationRecieved; //True, when we recieved a orientation-sample
	base::Time lastImuTime; //Timestamp, of the last recieved imu-sample. Used for calculating delta-time of acceleration
	base::Time lastGpsTime; //Timestamo, of the last recieved gps_sample. Used for calculation delta-time of velocity
	base::Time lastThrusterTime;
        base::Time lastEnvWrite;
        base::Vector3d lastVelocity;
	
	base::samples::RigidBodyState firstGpsSample; //Use first Gps-sample as origin
	base::samples::RigidBodyState lastGpsSample; //Last gps-sample for velocity-interpolation
	base::samples::RigidBodyState rbs; //Actual vehicle state
	boost::circular_buffer<base::samples::RigidBodyState> gpsPositions; //Ringbuffer of the last gps-samples for velocity estimation
	base::Orientation lastOrientation; 
	int samplesCount; //Counter for gps_samples
	
	base::samples::RigidBodyState lastVelocitySample; //Last velcity-sample. Used for calculating acceleration out of simulation data
	base::Time lastVelocityTime; //Timestamp of the last recieved velocity. USed for acceleration-calculation
	
	aggregator::StreamAligner strAligner; //Stream aligner 
	int orientationID; //ID of the orientation-stream
	int velocityID; //ID of the velocity-stream
	int imuID; //ID ofthe imu-stream
	int gpsID; //ID of the gps-stream
	int laserID; //ID of the lasser-stream
	int thrusterID;
	
	base::Quaterniond imuRotation;
        
        Eigen::AngleAxis<double> laserRotation;
        Eigen::Vector3d laserTranslation;
        
        uw_localization::NodeMap *node_map;
        uw_localization::UwMotionModel *motion_model;
	
	base::Vector3d actualVelocity;
	base::Vector3d relativeGps;	
	
	double sum;
	double count;
	
    };
    
    
}

#endif

