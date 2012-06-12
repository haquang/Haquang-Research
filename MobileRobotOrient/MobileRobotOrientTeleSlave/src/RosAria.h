#include <Aria.h>
#include "geometry_msgs/PointStamped.h"

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <math.h>

#define PC_On 0

#define maxVel 500
#define Max_Range 1000
#define Min_Range 100
#define Max_Force 10

#define Kmin 0.0005
#define Kmax 0.005

#define rangeX 2000
#define rangeY 500

#define N 100



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
PID RotController(1,0.5,0.01);

FILE *dataSla;
FILE *sonarData;
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
	void MasToSlaVel_cb( const geometry_msgs::PointStampedConstPtr &msg);
	void MasToSlaAng_cb( const geometry_msgs::PointStampedConstPtr &msg);
	void cmdvelener_cb( const geometry_msgs::PointStampedConstPtr &msg);

	void spin();


protected:
	ros::NodeHandle n;

	ros::Publisher SlaToMasVxs_Pub;
	ros::Publisher SlaToMasFxe_Pub;
	ros::Publisher SlaToMasFxv_Pub;

	ros::Subscriber MasToSlaVel_sub;

	ros::Publisher SlaToMasVys_Pub;
	ros::Publisher SlaToMasFye_Pub;
	ros::Publisher SlaToMasFyv_Pub;

	ros::Subscriber MasToSlaAng_sub;


	ros::Subscriber MasToSlaEnergy_sub;
	ros::Time veltime;

	std::string serial_port;

	ArSimpleConnector *conn;
	ArRobot *robot;

	ArSonarDevice sonar;
	ArPose Position;

	bool obstacleFront;

	double velFilter[N];
	int iCount;

	/*
	 * Master
	 */

	double Xm;
	double Vxm;
	double Fxve;
	double mst_Xm_P;
	double velScale;

	double Ym;
	double Vym;
	double Fyve;
	double mst_Ym_P;
	double angScale;

	/*
	 * Slave
	 */
	double Vxs;
	double Vxsd;
	double Fxe;
	double Fxe1;
	double Fxe2;
	double Fxv; //Velocity based
	double FxvGain;
	double Fxs;
	double sla_Vxs_P;
	double sla_Fxe_P;
	double sla_Fxv_P;
	double sla_Xm_N;
	double dxObs; // distance from obstacle
	double dxObs1; // distance from obstacle
	double dxObs2; // distance from obstacle
	double dxObsPrv;// previous distance from obstacle
	double dxObsDelta; // Time derivative

	double Vys;
	double Vysd;
	double Fye;
	double Fye1;
	double Fye2;
	double Fys;
	double sla_Fye_P;
	double sla_Ym_N;

	double dyObs; // distance from obstacle
	double dyObs1; // distance from obstacle
	double dyObs2; // distance from obstacle
	double dyObsPrv;// previous distance from obstacle
	double dyObsDelta; // Time derivative

	double gama;
	double Kex;		//stiffness
	double Key;		//stiffness

	double theta;
	/*
	 * Ros
	 */
	geometry_msgs::PointStamped SlaToMasVxs;
	geometry_msgs::PointStamped SlaToMasFxe;
	geometry_msgs::PointStamped SlaToMasFxv;

	geometry_msgs::PointStamped SlaToMasFye;

};

RosAriaNode::RosAriaNode(ros::NodeHandle nh)
{
	//initialize
	Vxm = 0;
	Xm = 0;
	mst_Xm_P = 0;
	sla_Fxe_P = 0;
	sla_Vxs_P = 0;
	sla_Fxv_P = 0;
	sla_Xm_N = 0;
	Vxs = 0;
	Fxe = 0;
	Fxve = 0;
	Kex=0.001;
	Fxv = 0;
	FxvGain = 0.01;
	gama = 400;

	velScale = 10;

	Vym = 0;
	Ym = 0;
	mst_Ym_P = 0;
	sla_Fye_P = 0;
	sla_Ym_N = 0;
	Vys = 0;
	Fye = 0;
	Fyve = 0;

	Key=0.0025;
	theta = 0;

	for (int i = 0;i<N;i++)
	{
		velFilter[i] = 0;
	}
	iCount = 0;

	dataSla = fopen("slv.txt","w");
	sonarData = fopen("sonar.txt","w");
	// read in config options
	n = nh;

	// !!! port !!!
	n.param( "port", serial_port, std::string("/dev/ttyUSB0") );
	ROS_INFO( "using serial port: [%s]", serial_port.c_str() );

	// advertise services

	SlaToMasVxs_Pub = n.advertise<geometry_msgs::PointStamped>("/SlatoMasVxs",1);
	SlaToMasFxe_Pub = n.advertise<geometry_msgs::PointStamped>("/SlatoMasFxe",1);
	SlaToMasFxv_Pub = n.advertise<geometry_msgs::PointStamped>("/SlatoMasFxv",1);


	SlaToMasFye_Pub = n.advertise<geometry_msgs::PointStamped>("/SlatoMasFye",1);

	// subscribe to services
	MasToSlaVel_sub = n.subscribe( "/MasToSlaVel", 1, (boost::function < void(const geometry_msgs::PointStampedConstPtr&)>) boost::bind( &RosAriaNode::MasToSlaVel_cb, this, _1 ));
	MasToSlaAng_sub = n.subscribe( "/MasToSlaAng", 1, (boost::function < void(const geometry_msgs::PointStampedConstPtr&)>) boost::bind( &RosAriaNode::MasToSlaAng_cb, this, _1 ));
	MasToSlaEnergy_sub = n.subscribe( "/MasToSlaVelEnergy", 1, (boost::function < void(const geometry_msgs::PointStampedConstPtr&)>) boost::bind( &RosAriaNode::cmdvelener_cb, this, _1 ));

	veltime = ros::Time::now();
}
RosAriaNode::~RosAriaNode()
{
	//disable motors and sonar.
	fclose(dataSla);
	fclose(sonarData);
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


void RosAriaNode::cmdvelener_cb( const geometry_msgs::PointStampedConstPtr &msg)
{
	mst_Xm_P = msg->point.x; // mstE_P_Xm;
	mst_Ym_P = msg->point.y;
}

void RosAriaNode::MasToSlaAng_cb( const geometry_msgs::PointStampedConstPtr &msg)
{
	Vysd = msg->point.x;
	Fyve = msg->point.y;
	Vym = msg->point.z;
}
void RosAriaNode::MasToSlaVel_cb( const geometry_msgs::PointStampedConstPtr &msg)
{
	Vxsd = msg->point.x;
	Fxve = msg->point.y;
	Vxm = msg->point.z;

	/*
	 * Velocity
	 */
	Vxs = robot->getVel();

	//Virtual force
	Fxs = PosController.compute(Vxsd,Vxs);
	//ROS_INFO("Force - Vsd: %5f - %5f",Fs,Vsd);

	/*
	 * Velocity Command channel
	 */
	// Calculate the negative energy and dissipate active energy

	if (Fxs*Vxsd>0)
	{
		sla_Xm_N -= Fxs*Vxsd;
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
			sla_Xm_N += Fxs*Vxsd; // backward 1 step

			// Modify Vsd
			if (Fxs*Fxs>0)
			{
				Vxsd = (sla_Xm_N+mst_Xm_P)/Fxs;
			}
			else
			{
				Vxsd = 0;
			}
			//ROS_INFO("Energy: N - P: %5f - %5f",sla_Xm_N,mst_Xm_P);
			//update
			sla_Xm_N -= Fxs*Vxsd;
		}
	}
#endif
	/*
	 * Velocity Feedback channel
	 */
	// Calculate Positive Energy
	if (Fxve*Vxs>0)
	{
		sla_Vxs_P += Fxve*Vxs;
		//	ROS_INFO("Energy - Velocity - Force: %5f - %5f - %5f",sla_Vs_P,Vs,Fve);
	}
	else
	{
		//Do nothing
	}



	/*
	 * Force Feedback Channel
	 */
	int numSonar;
	int i;
	numSonar = robot->getNumSonar();
	ArSensorReading* sonarReading;

	for (i=0; i<numSonar;i++)
	{
		sonarReading = robot->getSonarReading(i);

		if ((sonarReading->getSensorTh()==10))
		{
			dxObs1 = sonarReading->getRange();
		}

		if ((sonarReading->getSensorTh()==170))
		{
			dxObs2 = sonarReading->getRange();
		}
	}
	//ROS_INFO("Distance:  %5f   %5f",dxObs1,dxObs2);

	if (dxObs1<rangeX)
	{
		Fxe1 = -Kex*(rangeX - dxObs1);
	}
	else
	{
		Fxe1 = 0;
	}
	if (dxObs2<rangeX)
	{
		Fxe2 = Kex*(rangeX - dxObs2);
	}
	else
	{
		Fxe2 = 0;
	}

	Fxe = Fxe1 + Fxe2;
	//ROS_INFO("Force: %5f   %5f   %5f",Fxe1,Fxe2,Fxe);

	if (Fxe*Vxm>0)
	{
		sla_Fxe_P+=Fxe*Vxm;
	}
	else
	{
		//do nothing
	}

	// if (Fe>5)
	//ROS_INFO("Obstacle Distance Force: %5f  %5f  %5f",Fe, Ke, dObs);

	/*
	 * Force Feedback Channel
	 */


	//Velocity Based force
	Fxv = -FxvGain*Vxs;

	if (Fxv*Vxm>0)
	{
		sla_Fxv_P+=Fxv*Vxm;
	}
	else
	{
		//do nothing
	}
	//ROS_INFO("Force: %5f",Fxv);
	//Saturation
	if (Vxsd>maxVel)
		Vxsd = maxVel;
	if (Vxsd<-maxVel)
		Vxsd = -maxVel;

	//ROS_INFO("Velocity - Set Vel: %5f - %5f",Vxs,Vxsd);

	robot->setVel(Vxsd);

	//ROS_INFO("Velocity: %5f 	 %5f",Vsd,Vs);

	fprintf(dataSla,"%.3f %.3f %.3f\n",mst_Xm_P,sla_Xm_N,Fxe);
	fprintf(sonarData,"%.3f %.3f\n",dxObs1,dxObs2);

	/*
	 * Heading Angle
	 */

	//Vysd = 0.04;
	velFilter[iCount] = robot->getRotVel();
	iCount++;

	if (iCount>=N)
	{
		iCount = 0;
		Vys = 0;
		for (int i = 0;i<N;i++)
			Vys += velFilter[i]/N;
	}




	//ROS_INFO("Rot Velocity: %5f    %5f ",velFilter[iCount],Vys);

	//Virtual force
	Fys = RotController.compute(Vxsd,Vxs);

	/*
	 * Velocity Command channel
	 */
	// Calculate the negative energy and dissipate active energy

	if (Fys*Vysd>0)
	{
		sla_Ym_N -= Fys*Vysd;
	}
	else
	{
		//Do nothing
	}


	//PC
#if (PC_On)
	{
		if (sla_Ym_N+mst_Ym_P<0)
		{
			sla_Ym_N += Fys*Vysd; // backward 1 step

			// Modify Vsd
			if (Fys*Fys>0)
			{
				Vysd = (sla_Ym_N+mst_Ym_P)/Fys;
			}
			else
			{
				Vysd = 0;
			}
			//ROS_INFO("Energy: N - P: %5f - %5f",sla_Xm_N,mst_Xm_P);
			//update
			sla_Ym_N -= Fys*Vysd;
		}
	}
#endif
		/*
	 * Force Feedback Channel
	 */

	for (i=0; i<numSonar;i++)
	{
		sonarReading = robot->getSonarReading(i);

		if ((sonarReading->getSensorTh()==-90))
		{
			dyObs1 = sonarReading->getRange();
		}

		if ((sonarReading->getSensorTh()==90))
		{
			dyObs2 = sonarReading->getRange();
		}
	}
	//ROS_INFO("Distance:  %5f   %5f",dxObs1,dxObs2);

	if (dyObs1<rangeY)
	{
		Fye1 = -Key*(rangeY - dyObs1);
	}
	else
	{
		Fye1 = 0;
	}
	if (dyObs2<rangeY)
	{
		Fye2 = Key*(rangeY - dyObs2);
	}
	else
	{
		Fye2 = 0;
	}

	Fye = Fye1 + Fye2;
	ROS_INFO("Force: %5f   %5f   %5f",Fye1,Fye2,Fye);

	if (Fye*Vym>0)
	{
		sla_Fye_P+=Fye*Vym;
	}
	else
	{
		//do nothing
	}

	// if (Fe>5)
	//ROS_INFO("Obstacle Distance Force: %5f  %5f  %5f",Fe, Ke, dObs);
	theta += Vysd;
	robot->setHeading(theta);




	/*
	 * Publish
	 */
	SlaToMasVxs.point.x = Vxs;
	SlaToMasVxs.point.y = sla_Vxs_P;
	SlaToMasVxs.point.z = Fxs;

	SlaToMasVxs_Pub.publish(SlaToMasVxs);

	SlaToMasFxe.point.x = Fxe;
	SlaToMasFxe.point.y = sla_Fxe_P;

	SlaToMasFxe_Pub.publish(SlaToMasFxe);

	SlaToMasFxv.point.x = Fxv;
	SlaToMasFxv.point.y = sla_Fxv_P;

	SlaToMasFxv_Pub.publish(SlaToMasFxv);

	//Angle
	SlaToMasFye.point.x = Fye;
	SlaToMasFye.point.y = sla_Fye_P;
	SlaToMasFye.point.z = Fys;
	SlaToMasFye_Pub.publish(SlaToMasFye);

}
