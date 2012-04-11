#include <Aria.h>
#include "geometry_msgs/PointStamped.h"

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <math.h>

#define MaxVel 200
#define PC_Master1_ON 0
#define PC_Master2_ON 0

class PID
{
private:
	double Kp;
	double Kd;
	double Ki;
	double pre_error;
	double integral;
	static const double dt = 0.001;

public:
	PID(const double &Kp, const double &Kd, const double &Ki)
	{
		this->Kp = Kp;
		this->Kd = Kd;
		this->Ki = Ki;
		pre_error = 0.0;
		integral = 0.0;
	}

	double compute(const double &Xr, const double &X)
	{
		double error, output;

		error = Xr  - X;

		integral = integral + Ki*error*dt;

		output  = Kp *error  + Kd*(error - pre_error)/dt  + integral;
		//
		pre_error = error;

		return output;
	}
};
PID PosController(1,0.5,0.01);
/*************************************************************************************************
 * RosAriaNode Class
 *************************************************************************************************/
class RosAriaNode
{
public:
	RosAriaNode(ros::NodeHandle n);
	virtual ~RosAriaNode();

public:
	int Setup();
	void Mas1ToSla_cb( const geometry_msgs::PointStampedConstPtr &msg);
	void Mas2ToSla_cb( const geometry_msgs::PointStampedConstPtr &msg);

	void spin();


protected:
	ros::NodeHandle n;

	ros::Publisher SlaToMas1_Pub;
	ros::Publisher SlaToMas2_Pub;

	ros::Subscriber Mas1ToSla_sub;
	ros::Subscriber Mas2ToSla_sub;

	ros::Time veltime;

	std::string serial_port;

	ArSimpleConnector *conn;
	ArRobot *robot;

	ArSonarDevice sonar;
	ArPose Position;


	// PO-PC algorithm variable
	/*
	 * Master 1
	 */
	double Xm1;
	double Vm1;
	double Fk1;
	double mst1_slv_cmd_P;



	/*
	 * Master 2
	 */
	double Xm2;
	double Vm2;
	double mst2_slv_cmd_P;
	double Fk2;
	/*
	 * Slave
	 */
	double Xsd;
	double Xs;
	double Xsprv;
	double Vs;
	double Fs;
	double Fs1; //For master 1
	double Fs2; //For master 2
	double delta;


	double Scale;

	double alpha;

	double mst1_slv_cmd_N;
	double sla_mst1_cmd_P;


	double mst2_slv_cmd_N;
	double sla_mst2_cmd_P;

	double K_force;

	/*
	 * Ros
	 */
	geometry_msgs::PointStamped SlaToMas1;
	geometry_msgs::PointStamped SlaToMas2;
};

RosAriaNode::RosAriaNode(ros::NodeHandle nh)
{
	//initialize
	Xm1 = 0;
	Vm1 = 0;

	Xm2 = 0;
	Vm2 = 0;

	Xsd = 0;
	Xs  = 0;
	Vs  = 0;
	Fs  = 0;
	delta = 0;
	K_force = 1;

	alpha = 0.5;
	Scale = 20;
	mst1_slv_cmd_N = 0;
	sla_mst1_cmd_P = 0;

	mst2_slv_cmd_N = 0;
	sla_mst2_cmd_P = 0;
	// read in config options
	n = nh;

	// !!! port !!!
	n.param( "port", serial_port, std::string("/dev/ttyUSB0") );
	ROS_INFO( "using serial port: [%s]", serial_port.c_str() );

	// advertise services

	SlaToMas1_Pub = n.advertise<geometry_msgs::PointStamped>("/SlatoMas1",1);
	SlaToMas2_Pub = n.advertise<geometry_msgs::PointStamped>("/SlatoMas2",1);

	// subscribe to services
	Mas1ToSla_sub = n.subscribe( "/Mas1ToSla", 1, (boost::function < void(const geometry_msgs::PointStampedConstPtr&)>) boost::bind( &RosAriaNode::Mas1ToSla_cb, this, _1 ));
	Mas2ToSla_sub = n.subscribe( "/Mas2ToSla", 1, (boost::function < void(const geometry_msgs::PointStampedConstPtr&)>) boost::bind( &RosAriaNode::Mas2ToSla_cb, this, _1 ));

	veltime = ros::Time::now();
}
RosAriaNode::~RosAriaNode()
{
	//disable motors and sonar.
	robot->disableMotors();

	robot->stopRunning();
	robot->waitForRunExit();
	Aria::shutdown();
}

int RosAriaNode::Setup()
{
	ArArgumentBuilder *args;

	args = new ArArgumentBuilder();
	args->add("-rp"); //pass robot's serial port to Aria
	args->add(serial_port.c_str());
	conn = new ArSimpleConnector(args);


	robot = new ArRobot();
	robot->setCycleTime(1);

	ArLog::init(ArLog::File, ArLog::Verbose, "aria.log", true);

	//parse all command-line arguments
	if (!Aria::parseArgs())
	{
		Aria::logOptions();
		return 1;
	}

	// Connect to the robot
	if (!conn->connectRobot(robot)) {
		ArLog::log(ArLog::Terse, "rotate: Could not connect to robot! Exiting.");
		return 1;
	}

	//Sonar sensor
	robot->addRangeDevice(&sonar);
	robot->enableSonar();

	// Enable the motors
	robot->enableMotors();

	robot->runAsync(true);

	return 0;
}

void RosAriaNode::spin()
{
	ros::spin();
}


void RosAriaNode::Mas1ToSla_cb( const geometry_msgs::PointStampedConstPtr &msg)
{
	// Master 1 Position
	Vm1 = msg->point.x;
	Xm1 = Xm1 + Vm1;
	// Master force
	//	Fk1 = msg->point.y;
	// Master 1 Positive Energy
	mst1_slv_cmd_P = msg->point.z;


	Xsd = Scale *(alpha*Xm1 + (1-alpha)*Xm2);// design position

	Xsprv = Xs;
	Position = robot->getPose();
	Xs = Position.getX();
	delta = (Xs - Xsprv)*1000;

	Vs = PosController.compute(Xsd,Xs);

	// Fs - Sum
	Fs = K_force*(Xsd - Xs);
	Fs1 = alpha*Fs;
	Fs2 = (1-alpha)*Fs;
	/*
	 * Master 1 - Slave Channel
	 */

	// Calculate Negative Energy and dissipate Active energy
	if (Vm1*Fs1>0)
	{
		mst1_slv_cmd_N -=Vm1*Fs1;
	}
	else
	{
		//Do nothing
	}

	// PC:
#if (PC_Master1_ON)
	{
		if (mst1_slv_cmd_N+mst1_slv_cmd_P<0)
		{
			mst1_slv_cmd_N +=Vm1*Fs1;    // backward 1 step
			Xm1 = Xm1 - Vm1;			// backward 1 step
			// Modify Vm1
			if (Fs1*Fs1>0)
				Vm1 = (mst1_slv_cmd_N+mst1_slv_cmd_P)/Fs1;
			else
				Vm1 = 0;
			//Update
			Xm1 = Xm1 + Vm1;
			Xsd = Scale *(alpha*Xm1 + (1-alpha)*Xm2);// design position
			Vs = PosController.compute(Xsd,Xs);
			// Modify Fs ????

			mst1_slv_cmd_N -=Vm1*Fs1;
			ROS_INFO("PC");
		}
	}
#endif

	/*
	 * Slave - Master 1 Channel
	 */
	Fk1 = -Fs/2;
	// Calculate Positive Energy


	if (Fk1*Vm1>0)
	{
		//sla_mst1_cmd_P += Fk1*Vs;
		sla_mst1_cmd_P += Fk1*Vm1;
	}
	else
	{
		//Do nothing
	}

	/*
	 * Master 2 - Slave Channel
	 */

	// Calculate Negative Energy and dissipate Active energy
	if (Vm2*Fs2>0)
	{
		mst2_slv_cmd_N -=Vm2*Fs2;
	}
	else
	{
		//Do nothing
	}

	// PC:
#if (PC_Master2_ON)
	{
		if (mst2_slv_cmd_N+mst2_slv_cmd_P<0)
		{
			mst2_slv_cmd_N +=Vm2*Fs2;    // backward 1 step
			Xm2 = Xm2 - Vm2;			// backward 1 step
			// Modify Vm1
			if (Fs2*Fs2>0)
				Vm2 = (mst2_slv_cmd_N+mst2_slv_cmd_P)/Fs2;
			else
				Vm2 = 0;
			//Update
			Xm2 = Xm2 + Vm2;
			Xsd = Scale *(alpha*Xm1 + (1-alpha)*Xm2);// design position
			Vs = PosController.compute(Xsd,Xs);
			// Modify Fs ????

			mst2_slv_cmd_N -=Vm2*Fs2;
			ROS_INFO("PC");
		}
	}
#endif


	/*
	 * Slave - Master 2 Channel
	 */
	// Calculate Positive Energy
	Fk2 = -Fs/2;
	// Calculate Positive Energy


	if (Fk2*Vm2>0)
	{
		//sla_mst1_cmd_P += Fk1*Vs;
		sla_mst2_cmd_P += Fk2*Vm2;
	}
	else
	{
		//Do nothing
	}


	//Saturation
	if (Vs>MaxVel) Vs = MaxVel;
	if (Vs< - MaxVel) Vs = -MaxVel;

	//ROS_INFO("Velocity: %5f",Vs);
	//ROS_INFO("Ref - Real - Vel : %5f  -- %5f  --%5f",Xsd,Xs,Vs);
	//ROS_INFO("Position: %5f - %5f - %5f",Xs,Xsd,Fk2);
	robot->setVel(Vs);


	// Publisher
	SlaToMas1.point.x = Fs1;
	SlaToMas1.point.y = Fk1;
	SlaToMas1.point.z = sla_mst1_cmd_P;

	SlaToMas1_Pub.publish(SlaToMas1);

	SlaToMas2.point.x = Fs2;
	SlaToMas2.point.y = Fk2;
	SlaToMas2.point.z = sla_mst2_cmd_P;

	SlaToMas2_Pub.publish(SlaToMas2);
}

void RosAriaNode::Mas2ToSla_cb( const geometry_msgs::PointStampedConstPtr &msg)
{
	// Master 2 Position
	Vm2 = msg->point.x;
	Xm2 = Xm2 + Vm2;
	// Master force
	Fk2 = msg->point.y;
	// Master 1 Positive Energy
	mst2_slv_cmd_P = msg->point.z;

}
