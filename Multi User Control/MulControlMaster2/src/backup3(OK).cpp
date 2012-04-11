#include <iostream>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>


#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

// Haptic Device headers
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
//Ros
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
//Normal
#include <iostream>
#include <fstream>
#include <math.h>

#define PC_1_ON 1

class PID
{
private:
	double Kp;
	double Kd;
	double Ki;
	double pre_error;
	double integral;
	static const double dt = 0.001;
	static const double max =  0.8;
	static const double min = -0.8;
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


/*
 * Device Struct
 */
const int DT_MAX=5000;
int DT=400;
int MAX_FORCE=5;

/*
 * Master 1
 */
double Fm1; //Force
double Vm1; // Velocity
double X1;

// Energy
double mst1_mst2_cmd_P; //Positive Energy for command from master 1 to master 2


/*
 * Master 2
 */

hduVector3Dd position;
hduVector3Dd force;

double Xm2; // Spring Position
double X2;	// Real position
double X2_prv; //previous position
double X0; // init position
double Fm2; // Force (total)
double F1;// Master - Master force
double F2;// Master - Slave force
double Vm2; //Velocity
double KMaster;

// Energy
double mst2_mst1_cmd_P; // Positive Energy for command from master 2 to master 1
double mst1_mst2_cmd_N; //Negative Energy for command from master 1 to master 2

double mst2_slv_cmd_P; // Positive Energy for command from master 2 to slave
double sla_mst2_cmd_N; // Negative Energy for velocity feedback

/*
 * Slave
 */
double Fs; //Force
double Vs; //Velocity
double sla_mst2_cmd_P;
double Fk;


struct  Master1_To_Master2
{
	double Fm1[DT_MAX]; 			//Force
	double Vm1[DT_MAX]; 			// Velocity
	double mst1_mst2_cmd_P[DT_MAX]; //Energy - Positive
} Mas1ToMas2_pack;

struct  Master2_To_Master1
{
	double Fm2[DT_MAX]; 			//Force
	double Vm2[DT_MAX]; 			// Velocity
	double mst2_mst1_cmd_P[DT_MAX]; //Energy - Positive
} Mas2ToMas1_pack;

struct  Master2_To_Slave
{
	double Vm2[DT_MAX]; // Velocity Command
	double Fk[DT_MAX]; // Master 1 Force for velocity change
	double mst2_slv_cmd_P[DT_MAX]; // Positive energy in Veclocity command  channel
} Mas2ToSla_pack;

struct  Slave_to_Master2
{
	double Fs[DT_MAX]; // Slave Force
	double Fk[DT_MAX]; // Slave Velocity
	double sla_mst2_cmd_P[DT_MAX]; //Later use ########################
} SlaToMas2_pack;


// Global Variable
HDErrorInfo error;

HDCallbackCode hOmniCallback;

static HHD Device;
static HDSchedulerHandle hPhantomMain;
bool start = true;
PID PosController(0.2,0.001,0.001);
double alpha = 0.5;

/*
 * ROS Variable
 */
ros::Publisher Mas2ToMas1_pub;
ros::Publisher Mas2ToSla_pub;

geometry_msgs::PointStamped Mas2ToMas1;
geometry_msgs::PointStamped Mas2ToSla;

/*
 * Phantom function
 */
int  phantomOpen();
void phantomCalibrate();
void phantomRun();
void phantomPause();
void phantomClose();
HDCallbackCode HDCALLBACK phantom_callback(void *pUserData);


/*
 * support function
 */


void init_Data()
{
	ROS_INFO("Initilizing Data");
	X0 = 0;
	X1 = 0;
	X2 = 0;
	Xm2 = 0;
	Vm2 = 0;
	Vm1 = 0;
	Vs = 0;

	F1 = 0;
	F2 = 0;
	Fm1 = 0;
	Fm2 = 0;
	Fs = 0;

	KMaster = 0.1;

	mst1_mst2_cmd_N = 0;
	mst1_mst2_cmd_P = 0;
	mst2_mst1_cmd_P = 0;
	sla_mst2_cmd_N =0;
}

void Saturation(double *force){
	if (force[0]<-MAX_FORCE) force[0]=-MAX_FORCE;
	if (force[1]<-MAX_FORCE) force[1]=-MAX_FORCE;
	if (force[2]<-MAX_FORCE) force[2]=-MAX_FORCE;
	if (force[0]>MAX_FORCE) force[0]=MAX_FORCE;
	if (force[1]>MAX_FORCE) force[1]=MAX_FORCE;
	if (force[2]>MAX_FORCE) force[2]=MAX_FORCE;
}

/*
 * Delay function
 */

void Mas2ToMas1_delay()
{
	unsigned int i;

	//Makde delay
	for(i=0; i < DT-1; i++)
	{
		Mas2ToMas1_pack.Vm2[i] = Mas2ToMas1_pack.Vm2[i+1];
		Mas2ToMas1_pack.Fm2[i] = Mas2ToMas1_pack.Fm2[i+1];
		Mas2ToMas1_pack.mst2_mst1_cmd_P[i] = Mas2ToMas1_pack.mst2_mst1_cmd_P[i+1];
	}

	Mas2ToMas1_pack.Vm2[DT-1] = Vm2;
	Mas2ToMas1_pack.Fm2[DT-1] = F1; // Just send F1 which related to master - master channel instead of Fm2
	Mas2ToMas1_pack.mst2_mst1_cmd_P[DT-1] = mst2_mst1_cmd_P;


	// package
	Mas2ToMas1.point.x = Mas2ToMas1_pack.Vm2[0];
	Mas2ToMas1.point.y = Mas2ToMas1_pack.Fm2[0];
	Mas2ToMas1.point.z = Mas2ToMas1_pack.mst2_mst1_cmd_P[0];

	//publish
	Mas2ToMas1_pub.publish(Mas2ToMas1);
}

void Mas2ToSla_delay()
{

	unsigned int i;

	//Makde delay
	for(i=0; i < DT-1; i++)
	{
		Mas2ToSla_pack.Vm2[i] = Mas2ToSla_pack.Vm2[i+1];
		Mas2ToSla_pack.Fk[i] = Mas2ToSla_pack.Fk[i+1];
		Mas2ToSla_pack.mst2_slv_cmd_P[i] = Mas2ToSla_pack.mst2_slv_cmd_P[i+1];
	}

	Mas2ToSla_pack.Vm2[DT-1] = Vm2;
	Mas2ToSla_pack.Fk[DT-1] = Fk;
	Mas2ToSla_pack.mst2_slv_cmd_P[DT-1] = mst2_slv_cmd_P;


	// package
	Mas2ToSla.point.x = Mas2ToSla_pack.Vm2[0];
	Mas2ToSla.point.y = Mas2ToSla_pack.Fk[0];
	Mas2ToSla.point.z = Mas2ToSla_pack.mst2_slv_cmd_P[0];

	Mas2ToSla_pub.publish(Mas2ToMas1);
}


void SlaToMas2_cb(const geometry_msgs::PointStampedConstPtr &msg)
{
	unsigned i;
	for(i=0; i < DT-1; i++)
	{
		SlaToMas2_pack.Fs[i] = SlaToMas2_pack.Fs[i+1];
		SlaToMas2_pack.Fk[i] = SlaToMas2_pack.Fk[i+1];
		SlaToMas2_pack.sla_mst2_cmd_P[i] = SlaToMas2_pack.sla_mst2_cmd_P[i+1];
	}

	SlaToMas2_pack.Fs[DT - 1] = msg->point.x;   // Slave Force for velocity command
	SlaToMas2_pack.Fk[DT - 1] = msg->point.y; 	// Slave Velocity
	SlaToMas2_pack.sla_mst2_cmd_P[DT - 1] = msg->point.z;	// Slave Positve Energy

	Fs =  SlaToMas2_pack.Fs[0];					// Slave Force
	Fk = SlaToMas2_pack.Fk[0]/50;					// Slave Velocity
	sla_mst2_cmd_P = SlaToMas2_pack.sla_mst2_cmd_P[0];// Slave Positve Energy
}

void Mas1ToMas2_cb(const geometry_msgs::PointStampedConstPtr &msg)
{
	Vm1 = msg->point.x; // Get Master 1 Velocity
	X1 = X1 + Vm1;		// Integrate

	Fm1 = msg->point.y; // Master 1 Force
	mst1_mst2_cmd_P = msg->point.z; // Master 1 Positive Energy

	//	ROS_INFO("Ref Position: %5f",X1);
}

int main(int argc, char ** argv)
{
	//Init Phantom
	phantomOpen();
	printf("Phantom Opened \n");

	//Init ROS
	ros::init(argc, argv , "phantom_slave");

	ros::NodeHandle nh;

	// Publisher
	Mas2ToMas1_pub = nh.advertise<geometry_msgs::PointStamped>("Mas2ToMas1",1);
	Mas2ToSla_pub = nh.advertise<geometry_msgs::PointStamped>("Mas2ToSla",1);

	//Subcriber
	ros::Subscriber Mas1ToMas2_sub = nh.subscribe<geometry_msgs::PointStamped>("/Mas1ToMas2", 1,Mas1ToMas2_cb);
	ros::Subscriber SlaToMas2_sub = nh.subscribe<geometry_msgs::PointStamped>("/SlatoMas2", 1,SlaToMas2_cb);

	//Asign callback
	hPhantomMain = hdScheduleAsynchronous(phantom_callback, 0, HD_MAX_SCHEDULER_PRIORITY);
	//	hdSetSchedulerRate(1000);


	init_Data();
	//Start Phantom Scheduler
	phantomRun();


	//waiting
	while(ros::ok() && nh.ok())
	{
		ros::spinOnce();
	}
	phantomClose();

	return 1;
}




int  phantomOpen()
{
	HDErrorInfo error;
	Device = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		//hduPrintError(stderr, &error, "Failed to initialize haptic device");
		ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
		return -1;
	}
	ROS_INFO("Found %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));
	hdEnable(HD_FORCE_OUTPUT);
	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		ROS_ERROR("Failed to start the scheduler");//, &error);
		return -1;
	}

	phantomCalibrate();
}

void phantomCalibrate()
{
	int calibrationStyle;
	int supportedCalibrationStyles;
	HDErrorInfo error;

	hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
	if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
	{
		calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
		printf("HD_CALIBRATION_ENCODER_RESE..\n\n");
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
	{
		calibrationStyle = HD_CALIBRATION_INKWELL;
		printf("HD_CALIBRATION_INKWELL..\n\n");
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
	{
		calibrationStyle = HD_CALIBRATION_AUTO;
		printf("HD_CALIBRATION_AUTO..\n\n");
	}

	do
	{
		hdUpdateCalibration(calibrationStyle);
		printf("Calibrating.. (put stylus in well)\n");
		if (HD_DEVICE_ERROR(error = hdGetError()))
		{
			hduPrintError(stderr, &error, "Reset encoders reset failed.");
			break;
		}
	}   while (hdCheckCalibration() != HD_CALIBRATION_OK);


	printf("\n\nCalibration complete.\n");
}


void phantomPause()
{
	hdStopScheduler();
}

void phantomClose()
{
	hdStopScheduler();
	hdUnschedule(hPhantomMain);
	hdDisableDevice(Device);
}

void phantomRun()
{
	hdStartScheduler();
}

HDCallbackCode HDCALLBACK phantom_callback(void *pUserData)
{
	hdBeginFrame(hdGetCurrentDevice());
	hdGetDoublev(HD_CURRENT_POSITION,position);

	X2_prv = X2;

	if (start)
	{
		X0 = position[0];
		start = false;
		hdEndFrame(hdGetCurrentDevice());
		return HD_CALLBACK_CONTINUE;
	}

	X2 = position[0]-X0;
	Vm2 = X2 - X2_prv;

	F1 = PosController.compute(X1,X2);// X1 - Ref, X2 - Real
	//	F1 = KMaster*(X1 - X2);

	// Master1-Master2 Channel - Calculate Negative energy and dissipate active energy
	// Force: F1
	if (F1*Vm1>0)
	{
		mst1_mst2_cmd_N -=F1*Vm1;
	}
	else
	{
		//Do nothing
	}

	//PC
	if (PC_1_ON)
	{
		if (mst1_mst2_cmd_P + mst1_mst2_cmd_N < 0)
		{
			mst1_mst2_cmd_N +=F1*Vm1; //backward 1 step
			X1 = X1 - Vm1;			// backward 1 step
			// Modify Vm1
			if (F1*F1 > 0.0)
				Vm1 = (mst1_mst2_cmd_P + mst1_mst2_cmd_N)/F1;
			else
				Vm1 = 0;

			// Update F1
			X1 = X1 + Vm1;
			F1 = PosController.compute(X1,X2);// X1 - Ref, X2 - Real
			//	F1 = KMaster*(X1 - X2);
			// Update Energy
			mst1_mst2_cmd_N -=F1*Vm1;
			ROS_INFO("PC");
		}
	}


	// Master2 - Master 1 Channel: Calculate Positive Energy

	if (Fm1*Vm2>0)
	{
		mst2_mst1_cmd_P+=Fm1*Vm2;
	}
	else
	{
		//Do nothing
	}

	/*
	 *	Master2-Slave Channel - Calculate Positive energy
	 */

	// Calculate Positive Energy
	if (Vm2*Fs>0)
	{
		mst2_slv_cmd_P += Vm2*Fs;
	}
	else
	{
		//Do nothing
	}

	/*
	 * Slave - Master 1 Channel
	 */

	// Calculate the Negative energy and dissipate Active Energy
	if (Fk*Vm2>0)
	{
		sla_mst2_cmd_N -=Fk*Vm2;
	}
	else
	{
		//Do nothing
	}

	//PC
	if (sla_mst2_cmd_P+sla_mst2_cmd_N<0)
	{
		sla_mst2_cmd_N +=Fk*Vm2; // backward 1 step
		//Modify Vs
		ROS_INFO("PC");
		if (Vm2*Vm2>0)
			Fk = (sla_mst2_cmd_P+sla_mst2_cmd_N)/Vm2;
		else
			Fk=0;

		//Update
		sla_mst2_cmd_N -=Fk*Vm2;
	}



	// Sum and send to device
	Fm2 = F1 + Fk;
	force[2] =0;
	force[1] =0;
	force[0] =Fm2;

//	ROS_INFO("Force: %5f",Fm2);
//	ROS_INFO("Modify Vel: %5f",Vm1);
	Saturation(force);

	hdSetDoublev(HD_CURRENT_FORCE,force);
	hdEndFrame(hdGetCurrentDevice());

	Mas2ToMas1_delay();
	Mas2ToSla_delay();
	return HD_CALLBACK_CONTINUE;
}
