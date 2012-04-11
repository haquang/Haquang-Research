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
#define PC_1_ON 1
#define Vs_min 50

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
int DT=100;

// Master 1

hduVector3Dd position;
hduVector3Dd force;

double Xm1; // Spring Position
double X1;	// Real position
double X1_prv; //previous position
double X0; // init position
double Fm1; // Force (total)
double F1;// Master - Master force
double F2;// Master - Slave force

double Vm1; //Velocity
double KMaster;


// Energy
double mst1_mst2_cmd_P; //Positive Energy for command from master 1 to master 2
double mst2_mst1_cmd_N; //Negative Energy for command from master 2 to master 1

double mst1_slv_cmd_P; // Positive Energy for command from master 1 to slave
double sla_mst1_cmd_N; // Negative Energy for velocity feedback
/*
 * Master 2
 */
double Fm2; //Force
double Vm2; // Velocity
double X2;	//Position


//Energy
double mst2_mst1_cmd_P; // Positive Energy for command from master 2 to master 1

/*
 * Slave
 */
double Fs; //Force - for calculating energy at master side
double Vs; //Velocity
double sla_mst1_cmd_P; //Later use ########################
double Fk;
double Xs;
double Xsi;



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

struct  Master1_To_Slave
{
	double Vm1[DT_MAX]; // Velocity Command
	double Fk[DT_MAX]; // Master 1 Force for velocity change
	double mst1_slv_cmd_P[DT_MAX]; // Positive energy in Veclocity command  channel
} Mas1ToSla_pack;

struct  Slave_to_Master1
{
	double Fs[DT_MAX]; // Slave Force
	double Vs[DT_MAX]; // Slave Velocity
	double sla_mst1_cmd_P[DT_MAX]; //Later use ########################
} SlaToMas1_pack;



// Global Variable
HDErrorInfo error;
HHD Device;
HDCallbackCode hOmniCallback;
bool start = true;
hduVector3Dd zeroPos (0.0,0.0,0.0);

double alpha = 0.5;
double Scale = 20;

PID PosController(0.2,0.001,0.001);


/*
 * Ros
 */
ros::Publisher Mas1ToMas2_pub;
ros::Publisher Mas1ToSla_pub;

geometry_msgs::PointStamped Mas1ToMas2;
geometry_msgs::PointStamped Mas1ToSla;



using namespace std;
//Function definition
int OmniOpen(void);
int OmniRun(void);
void OmniClose(void);
void OmniCalibrate(void);
void mainLoop();
HDCallbackCode HDCALLBACK omni_callback(void *pUserData);



/*
 * Haptic Callback funtion
 */

int OmniRun()
{
	ROS_INFO("RUN");
	hdStartScheduler();
	return 0;
}

void OmniClose()
{

}
void mainLoop()
{
	char key;
	ros::spin();
	do
	{

		key = getchar();
	}while(key != 27);
}

void init_Data()
{
	ROS_INFO("Initilizing Data");

	X0 = 0;
	X1 = 0;
	X2 = 0;
	Xm1 = 0;
	Vm1 = 0;
	Vm2 = 0;
	Vs = 0;
	Xsi = 0;

	F1 = 0;
	F2 = 0;
	Fm1 = 0;
	Fm2 = 0;
	Fs = 0;

	KMaster = 0.1;

	mst1_mst2_cmd_P = 0;
	mst2_mst1_cmd_N = 0;
	mst2_mst1_cmd_P = 0;
	sla_mst1_cmd_N =0;
}

void Saturation(double *force){
	if (force[0]<-5) force[0]=-5;
	if (force[1]<-5) force[1]=-5;
	if (force[2]<-5) force[2]=-5;
	if (force[0]>5) force[0]=5;
	if (force[1]>5) force[1]=5;
	if (force[2]>5) force[2]=5;
}

/*
 * Delay function
 */

void Mas1ToMas2_delay()
{

	unsigned int i;

	//Makde delay
	for(i=0; i < DT-1; i++)
	{
		Mas1ToMas2_pack.Vm1[i] = Mas1ToMas2_pack.Vm1[i+1];
		Mas1ToMas2_pack.Fm1[i] = Mas1ToMas2_pack.Fm1[i+1];
		Mas1ToMas2_pack.mst1_mst2_cmd_P[i] = Mas1ToMas2_pack.mst1_mst2_cmd_P[i+1];
	}

	Mas1ToMas2_pack.Vm1[DT-1] = Vm1;
	Mas1ToMas2_pack.Fm1[DT-1] = F1;  // Just send F1 instead of Fm1
	Mas1ToMas2_pack.mst1_mst2_cmd_P[DT-1] = mst1_mst2_cmd_P;


	// package
	Mas1ToMas2.point.x = Mas1ToMas2_pack.Vm1[0];
	Mas1ToMas2.point.y = Mas1ToMas2_pack.Fm1[0];
	Mas1ToMas2.point.z = Mas1ToMas2_pack.mst1_mst2_cmd_P[0];

	//	ROS_INFO("Master 1 Vel: %5f",Mas1ToMas2.point.x);
	Mas1ToMas2_pub.publish(Mas1ToMas2);
}

void Mas1ToSla_delay()
{
	unsigned int i;

	//Makde delay
	for(i=0; i < DT-1; i++)
	{
		Mas1ToSla_pack.Vm1[i] = Mas1ToSla_pack.Vm1[i+1];
		Mas1ToSla_pack.Fk[i] = Mas1ToSla_pack.Fk[i+1];
		Mas1ToSla_pack.mst1_slv_cmd_P[i] = Mas1ToSla_pack.mst1_slv_cmd_P[i+1];
	}

	Mas1ToSla_pack.Vm1[DT-1] = Vm1;
	Mas1ToSla_pack.Fk[DT-1] = Fk;
	Mas1ToSla_pack.mst1_slv_cmd_P[DT-1] = mst1_slv_cmd_P;


	// package
	Mas1ToSla.point.x = Mas1ToSla_pack.Vm1[0];
	Mas1ToSla.point.y = Mas1ToSla_pack.Fk[0];
	Mas1ToSla.point.z = Mas1ToSla_pack.mst1_slv_cmd_P[0];

	//	ROS_INFO("Master 1 Vel: %5f",Mas1ToMas2.point.x);
	Mas1ToSla_pub.publish(Mas1ToSla);
}


void SlaToMas1_cb(const geometry_msgs::PointStampedConstPtr &msg)
{
	unsigned i;
	for(i=0; i < DT-1; i++)
	{
		SlaToMas1_pack.Fs[i] = SlaToMas1_pack.Fs[i+1];
		SlaToMas1_pack.Vs[i] = SlaToMas1_pack.Vs[i+1];
		SlaToMas1_pack.sla_mst1_cmd_P[i] = SlaToMas1_pack.sla_mst1_cmd_P[i+1];
	}

	SlaToMas1_pack.Fs[DT - 1] = msg->point.x;   // Slave Force for velocity command
	SlaToMas1_pack.Vs[DT - 1] = msg->point.y; 	// Slave Velocity
	SlaToMas1_pack.sla_mst1_cmd_P[DT - 1] = msg->point.z;	// Slave Positve Energy

	Fs =  SlaToMas1_pack.Fs[0];					// Slave Force
	Vs = SlaToMas1_pack.Vs[0];					// Slave Velocity
	sla_mst1_cmd_P = SlaToMas1_pack.sla_mst1_cmd_P[0];// Slave Positve Energy
}

void Mas2ToMas1_cb(const geometry_msgs::PointStampedConstPtr &msg)
{
	Vm2 = msg->point.x; // Get Master 2 Velocity
	X2 = X2 + Vm2;		// Integrate

	Fm2 = msg->point.y; // Master 2 Force
	mst2_mst1_cmd_P = msg->point.z; // Master 2 Positive Energy

	//	ROS_INFO("Ref Position: %5f",X2);
}

/*
 * Calibrate Function
 */
void OmniCalibrate(void)
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

	ROS_INFO("Device Calibrated");

	//Start Phantom
	hdStartScheduler();
	ROS_INFO("Set init position for master");
	hdStopScheduler();


	init_Data();
	hdScheduleAsynchronous(omni_callback, (void *) 0, HD_DEFAULT_SCHEDULER_PRIORITY);
	hdSetSchedulerRate(1000);

}

/*
 * Main Callback funtion
 */
HDCallbackCode HDCALLBACK omni_callback(void *pUserData)
{
	Xs = Xs +Vs/(Scale*alpha);

	ROS_INFO("Position: %5f - %5f",Xsi,Vs);

	hdBeginFrame(hdGetCurrentDevice());
	hdGetDoublev(HD_CURRENT_POSITION,position);

	X1_prv = X1;



	if (start)
	{
		X0 = position[0];
		start = false;
		hdEndFrame(hdGetCurrentDevice());
		return HD_CALLBACK_CONTINUE;
	}
	X1 = position[0]-X0;
	Vm1 = X1 - X1_prv;

	F1 = PosController.compute(X2,X1);
	//	F1 = KMaster*(X2 - X1);

	/*
	 * Master1-Master2 Channel - Calculate Positive energy
	 */
	// Force: F1
	if (Fm2*Vm1>0)
	{
		mst1_mst2_cmd_P +=Fm2*Vm1;
	}
	else
	{
		//Do nothing
	}

	// Master 2 - Master 1 Channel - Calculate Negative Energy and dissipate active
	if (F1*Vm2>0)
	{
		mst2_mst1_cmd_N -=F1*Vm2;
	}
	else
	{
		//Do nothing
	}
	//PC
	if (PC_1_ON)
	{
		if (mst2_mst1_cmd_N+mst2_mst1_cmd_P<0)
		{
			mst2_mst1_cmd_N +=F1*Vm2; // backward 1 step
			X2 = X2 - Vm2;			  // backward 1 step

			// Modify Vm2
			if (F1*F1 > 0.0)
				Vm2 = (mst2_mst1_cmd_N+mst2_mst1_cmd_P)/F1;
			else
				Vm2 = 0;

			// Update F1
			X2 = X2 + Vm2;
			//		F1 = KMaster*(X2 - X1);
			F1 = PosController.compute(X2,X1);// X1 - Ref, X2 - Real
			// Update Energy
			mst2_mst1_cmd_N -=F1*Vm2;
		}

	}

	/*
	 *	Master1-Slave Channel - Calculate Positive energy
	 */

	// Calculate Positive Energy
	if (Vm1*Fs>0)
	{
		mst1_slv_cmd_P += Vm1*Fs;
	}
	else
	{
		//Do nothing
	}

	/*
	 * Slave - Master 1 Channel
	 */

	Fk = -0.0001*(X1 - Xs);

	// Calculate the Negative energy and dissipate Active Energy
	if (Fk*Vs>0)
	{
		sla_mst1_cmd_N -=Fk*Vs;
	}
	else
	{
		//Do nothing
	}

	//PC
	if (sla_mst1_cmd_P+sla_mst1_cmd_N<0)
	{
		sla_mst1_cmd_N +=Fk*Vs; // backward 1 step
		Xs = Xs - Vs;
		//Modify Vs
		if (Fk*Fk>0)
			Vs = (sla_mst1_cmd_P+sla_mst1_cmd_N)/Fk;
		else
			Vs=0;

		//Update
		Xs = Xs + Vs;
		Fk = -0.0001*(X1 - Xs);
		sla_mst1_cmd_N -=Fk*Vs;
	}




	// Sum up and send to device
	//Fm1 = F1+Fk;
	Fm1 = Fk;
	force[2] =0;
	force[1] =0;
	force[0] =Fm1;

	//ROS_INFO("Xm - Xs - Force: %5f  %5f  %5f",X1,Xs,Fk);
	//ROS_INFO("Modify Vel: %5f",Vm2);

	Saturation(force);

	//hdSetDoublev(HD_CURRENT_FORCE,force);
	hdEndFrame(hdGetCurrentDevice());

	Mas1ToMas2_delay();
	Mas1ToSla_delay();
	return HD_CALLBACK_CONTINUE;
}


/*
 * Main Program
 */

int main(int argc,char* argv[])
{
	//Init ROS
	ros::init(argc, argv , "omini_phantom");
	ros::NodeHandle nh;

	// Publisher
	Mas1ToMas2_pub = nh.advertise<geometry_msgs::PointStamped>("Mas1ToMas2",1);
	Mas1ToSla_pub = nh.advertise<geometry_msgs::PointStamped>("Mas1ToSla",1);

	//Subcriber
	ros::Subscriber Mas2ToMas1_sub = nh.subscribe<geometry_msgs::PointStamped>("/Mas2ToMas1", 1,Mas2ToMas1_cb);
	ros::Subscriber SlaToMas1_sub = nh.subscribe<geometry_msgs::PointStamped>("/SlatoMas1", 1,SlaToMas1_cb);


	//Open Omni Device
	HHD Device = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		ROS_ERROR("Failed to open!");
		return -1;
	}
	ROS_INFO("Device openned!!");

	hdEnable(HD_FORCE_OUTPUT);

	//Calibrate device
	OmniCalibrate();

	init_Data();
	//Run device
	OmniRun();

	// Loop
	mainLoop();


	OmniClose();
	return 0;
}
