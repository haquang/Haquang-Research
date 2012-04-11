#include <Aria.h>
#include "geometry_msgs/PointStamped.h"

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <math.h>

#define PC_Xm_On 1

#define maxVel 500
#define Max_Range 1000
#define Min_Range 100
#define Max_Force 10

#define Kmin 0.001
#define Kmax 0.05
#define gama 50
#define range 2000

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
	void MasToSla_cb( const geometry_msgs::PointStampedConstPtr &msg);
	void cmdener_cb( const geometry_msgs::PointStampedConstPtr &msg);

	void spin();


protected:
	ros::NodeHandle n;

	ros::Publisher SlaToMasVs_Pub;
	ros::Publisher SlaToMasFe_Pub;

	ros::Subscriber MasToSla_sub;
	ros::Subscriber MasToSlaEnergy_sub;
	ros::Time veltime;

	std::string serial_port;

	ArSimpleConnector *conn;
	ArRobot *robot;

	ArSonarDevice sonar;
	ArPose Position;

	/*
	 * Master
	 */

	double Xm;
	double Vm;
	double Fv;
	double mst_Xm_P;
	double Scale;
	/*
	 * Slave
	 */
	double Vs;
	double Vsd;
	double Fe;
	double Fs;
	double sla_Vs_P;
	double sla_Fe_P;
	double sla_Xm_N;
	double dObs; // distance from obstacle
	double dObsPrv;// previous distance from obstacle
	double dObsDelta; // Time derivative

	double Ke;		//stiffness
	/*
	 * Ros
	 */
	geometry_msgs::PointStamped SlaToMasVs;
	geometry_msgs::PointStamped SlaToMasFe;
};

RosAriaNode::RosAriaNode(ros::NodeHandle nh)
{
	//initialize
	Vm = 0;
	Xm = 0;
	mst_Xm_P = 0;
	sla_Fe_P = 0;
	sla_Vs_P = 0;
	sla_Xm_N = 0;
	Vs = 0;
	Fe = 0;
	Fv = 0;
	Ke=0;


	Scale = 10;
	// read in config options
	n = nh;

	// !!! port !!!
	n.param( "port", serial_port, std::string("/dev/ttyUSB0") );
	ROS_INFO( "using serial port: [%s]", serial_port.c_str() );

	// advertise services

	SlaToMasVs_Pub = n.advertise<geometry_msgs::PointStamped>("/SlatoMasVs",1);
	SlaToMasFe_Pub = n.advertise<geometry_msgs::PointStamped>("/SlatoMasFe",1);


	// subscribe to services
	MasToSla_sub = n.subscribe( "/MasToSla", 1, (boost::function < void(const geometry_msgs::PointStampedConstPtr&)>) boost::bind( &RosAriaNode::MasToSla_cb, this, _1 ));
	MasToSlaEnergy_sub = n.subscribe( "/MasToSlaEnergy", 1, (boost::function < void(const geometry_msgs::PointStampedConstPtr&)>) boost::bind( &RosAriaNode::cmdener_cb, this, _1 ));

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


void RosAriaNode::cmdener_cb( const geometry_msgs::PointStampedConstPtr &msg)
{
	mst_Xm_P = msg->point.x; // mstE_P_Xm;
}

void RosAriaNode::MasToSla_cb( const geometry_msgs::PointStampedConstPtr &msg)
{
	Vsd = msg->point.x;
	Fv = msg->point.y;
	Vm = msg->point.z;

	Vs = robot->getVel();
	//Virtual force
	Fs = PosController.compute(Vsd,Vs);
	//ROS_INFO("Force - Vsd: %5f - %5f",Fs,Vsd);

	/*
	 * Velocity Command channel
	 */
	// Calculate the negative energy and dissipate active energy

	if (Fs*Vsd>0)
	{
		sla_Xm_N -= Fs*Vsd;
	}
	else
	{
		//Do nothing
	}


	//PC
#if (PC_Xm_On)
	{
		if (sla_Xm_N+mst_Xm_P<0)
		{
			sla_Xm_N += Fs*Vsd; // backward 1 step

			// Modify Vsd
			if (Fs*Fs>0)
			{
				Vsd = (sla_Xm_N+mst_Xm_P)/Fs;
			}
			else
			{
				Vsd = 0;
			}
			//ROS_INFO("Energy: N - P: %5f - %5f",sla_Xm_N,mst_Xm_P);
			//update
			sla_Xm_N -= Fs*Vsd;
		}
	}
#endif
	/*
	 * Velocity Feedback channel
	 */
	// Calculate Positive Energy
	if (Fv*Vs>0)
	{
		sla_Vs_P += Fv*Vs;
	}
	else
	{
		//Do nothing
	}
	//ROS_INFO("Energy - Velocity - Force: %5f - %5f - %5f",sla_Vs_P,Vs,Fv);


	/*
	 * Force Feedback Channel
	 */
	dObsPrv = dObs;
	dObs = sonar.currentReadingPolar(-30,30);
	dObsDelta = dObs - dObsPrv;

	// Variable stiffness
	if (dObsDelta>0)
		Ke = Kmin;
	else if (dObsDelta>-gama) {
		Ke = -(1/gama)*(Kmax - Kmin)*dObsDelta+Kmin;
	}
	else
		Ke = Kmax;

	//Environment force

	if (dObs<range)
		Fe = Ke*(range - dObs);
	else
		Fe = 0;


	if (Fe*Vm>0)
	{
		sla_Fe_P+=Fe*Vm;
	}
	else
	{
		//do nothing
	}

	ROS_INFO("Obstacle Distance Force: %5f",Fe);

	//Saturation
	if (Vsd>maxVel)
		Vsd = maxVel;
	if (Vsd<-maxVel)
		Vsd = -maxVel;

//	ROS_INFO("Velocity - Set Vel: %5f - %5f",Vs,Vsd);

	robot->setVel(Vsd);


	// Publish
	SlaToMasVs.point.x = Vs;
	SlaToMasVs.point.y = sla_Vs_P;
	SlaToMasVs.point.z = Fs;

	SlaToMasVs_Pub.publish(SlaToMasVs);

	SlaToMasFe.point.x = Fe;
	SlaToMasFe.point.y = sla_Fe_P;

	SlaToMasFe_Pub.publish(SlaToMasFe);
}
