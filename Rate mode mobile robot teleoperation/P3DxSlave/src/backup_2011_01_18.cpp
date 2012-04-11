#include <Aria.h>
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <math.h>

#define Max_Range 1000
#define Min_Range 100
#define Max_Force 10

FILE *Vs_save;
FILE *Vsd_save;

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
	void cmdvel_cb( const geometry_msgs::PointStampedConstPtr &msg);
	void cmdener_cb( const geometry_msgs::PointStampedConstPtr &msg);

	void spin();


protected:
	ros::NodeHandle n;

	ros::Publisher chanelParameter_Pub;
	ros::Publisher chanelEnergy_Pub;

	ros::Subscriber cmdvel_sub;
	ros::Subscriber cmdener_sub;

	ros::Time veltime;

	std::string serial_port;

	ArSimpleConnector *conn;
	ArRobot *robot;

	ArSonarDevice sonar;


	// PO-PC algorithm variable
	/*
	 * Slave
	 */
	double distance; // distance from obstacle
	double Vsd;    // Set Speed
	double Vs;		// Real Speed
	double fs;		//virtual force
	double fe;		//environment force
	double Ke;		//stiffness

	double slaE_P_fe;
	double slaE_P_fk;
	double slaE_N_Xm;
	/*
	 * Master
	 */
	double Vm;
	double mstE_P_Xm;
	double fk;

	/*
	 * ros package
	 */
	geometry_msgs::PointStamped chanelParameter;
	geometry_msgs::PointStamped chanelEnergy;

};

RosAriaNode::RosAriaNode(ros::NodeHandle nh)
{
	//initialize
	slaE_P_fe = 0.0;
	slaE_P_fk = 0.0;
	slaE_N_Xm = 0.0;
	distance = 0.0;
	Ke=1000;

	Vs_save = fopen("Vs.txt","w");
	Vsd_save = fopen("Vsd.txt","w");
	// read in config options
	n = nh;

	// !!! port !!!
	n.param( "port", serial_port, std::string("/dev/ttyUSB0") );
	ROS_INFO( "using serial port: [%s]", serial_port.c_str() );

	// advertise services

	chanelParameter_Pub = n.advertise<geometry_msgs::PointStamped>("/chanelParameter",1);
	chanelEnergy_Pub = n.advertise<geometry_msgs::PointStamped>("/chanelEnergy",1);

	// subscribe to services
	cmdvel_sub = n.subscribe( "/master_position", 1, (boost::function < void(const geometry_msgs::PointStampedConstPtr&)>) boost::bind( &RosAriaNode::cmdvel_cb, this, _1 ));
	cmdener_sub = n.subscribe( "/master_energy", 1, (boost::function < void(const geometry_msgs::PointStampedConstPtr&)>) boost::bind( &RosAriaNode::cmdener_cb, this, _1 ));
	veltime = ros::Time::now();
}
RosAriaNode::~RosAriaNode()
{
	fclose(Vs_save);
	fclose(Vsd_save);
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
	sonar.setMaxRange(Max_Range);
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
	mstE_P_Xm = msg->point.x; // mstE_P_Xm;
}
void RosAriaNode::cmdvel_cb( const geometry_msgs::PointStampedConstPtr &msg)
{
	veltime = ros::Time::now();
	//get data from Package
	Vsd =ceil(msg->point.x); //Vsd = Xm/Scale
	Vm=msg->point.y;		// Vm
	fk = msg->point.z;

	//real velocity
	Vs = ceil(robot->getVel());
	//Distance from obstacle
	distance = sonar.currentReadingPolar(-30,30);

	//Environment force
	if (distance<Min_Range)
		fe = -Max_Force;
	else if (distance<Max_Range)
		fe = -Ke*1/distance;
	else
		fe=0;
	//Virtual force
	fs = (Vsd-Vs)/50;

	/*
	 * Xm chanel
	 */

	//PO
	if (fs*Vsd>0)
	{
		//do nothing
	}
	else
	{
		slaE_N_Xm+=fs*Vsd; //output
	}

	//PC
#if 1
	if (slaE_N_Xm+mstE_P_Xm<0) //input + output<0 =>active
	{
		//modify Vsd
		slaE_N_Xm-=fs*Vsd;
		Vsd = (slaE_N_Xm+mstE_P_Xm)/fs;
		slaE_N_Xm+=fs*Vsd;
	}
#endif

	/*
	 * fe chanel
	 */

	if (fe*Vm>0)
	{
		slaE_P_fe+=fe*Vm;
	}
	else
	{
		//do nothing
	}

	/*
	 * fk chanel
	 */

	if (fk*Vs>0)
	{
		slaE_P_fk+=fk*Vs;
	}
	else
	{
		//do nothing
	}

	ROS_INFO("Set velocity: %5f",Vsd);
	ROS_INFO("Real velocity: %5f",Vs);
	ROS_INFO("Force: %5f",fe);

	robot->setVel(Vsd);

	//Package and publish

	chanelParameter.point.x = Vs;
	chanelParameter.point.y = fs;
	chanelParameter.point.z = fe;

	chanelEnergy.point.x = slaE_P_fe;
	chanelEnergy.point.y = slaE_P_fk;

	chanelParameter_Pub.publish(chanelParameter);
	chanelEnergy_Pub.publish(chanelEnergy);

	fprintf(Vs_save,"%.3f \n",Vs);
	fprintf(Vsd_save,"%.3f \n",Vsd);
}
