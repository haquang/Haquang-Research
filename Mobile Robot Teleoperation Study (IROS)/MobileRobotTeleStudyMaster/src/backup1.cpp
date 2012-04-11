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


#define CASE1 0 // Velocity based force (0 - Velocity Error based)
#define FeOn 1 // Velocity based force (0 - Velocity Error based)

#define PC_Fv_On 1
#define PC_Fe_On 0

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
int DT=300;
int MAX_FORCE=5;



// Global Variable
HDErrorInfo error;

HDCallbackCode hOmniCallback;

static HHD Device;
static HDSchedulerHandle hPhantomMain;
bool start = true;

/*
 * Master
 */
hduVector3Dd init_pos;
hduVector3Dd posistion;
hduVector3Dd force;
PID PosController(0.002,0.0001,0.0001);

double X0; // Initial Position
double Xprv; // Privious Position
double X; // Master Position
double Xm; // Mass spring Position
double V; // Master Velocity
double Vm; // Mass spring Velocity
double Fv; // Velocity Based Force

double Fm; // Master force
double K_Master;

double Scale;
//Energy
double mst_Vs_N; // Negative Energy in Velocity feedback channel
double mst_Fe_N; // Negative Energy in Force feedback channel
double mst_Xm_P; // Positive Energy in Velocity Command channel
double FvGain;
/*
 * Slave
 */
double Vs; // Slave Velocity
double Fe; // Obstacle distance based force
double Fs; // Virtual Force for energy calculating
// Energy
double sla_Vs_P; // Positive Energy in Velocity feedback channel
double sla_Fe_P; // Positive Energy in Force feedback energy


/*
 * Master to Slave
 */
struct  Master_To_Slave
{
	double Xm[DT_MAX];			// Master Position
	double Vm[DT_MAX];			// Master Velocity
	double Fv[DT_MAX];			// Send calculated force to slave side
	double mst_Xm_P[DT_MAX];	// Positive Energy in velocity command channel
} MasToSla_pkg;

/*
 * Slave to Master
 */
struct  Slave_To_Master
{
	double Vs[DT_MAX]; // Slave Velocity
	double Fe[DT_MAX]; // Obstacle distance based force
	double Fs[DT_MAX]; //Virtual Force for energy calculating
	double sla_Vs_P[DT_MAX]; // Positive Energy in Velocity feedback channel
	double sla_Fe_P[DT_MAX]; // Positive Energy in Force feedback energy

} SlaToMas_pkg;

/*
 * ROS Variable
 */
ros::Publisher Mas2Sla_pub;
ros::Publisher Mas2SlaEnergy_pub;
geometry_msgs::PointStamped MasToSla;
geometry_msgs::PointStamped MasToSlaEnergy;

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
	Xm = 0;
	Xprv = 0;

	V = 0;
	Vm = 0;
	Vs = 0;


	mst_Fe_N = 0;
	mst_Vs_N = 0;
	mst_Xm_P = 0;

	Fe = 0;
	Fv = 0;

	FvGain = 0.01;
	Scale = 10;
	K_Master = 1;
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

void MasToSlaDelay()
{
	unsigned int i;

	//Makde delay
	for(i=0; i < DT-1; i++)
	{
		MasToSla_pkg.Xm[i] = MasToSla_pkg.Xm[i+1];
		MasToSla_pkg.Vm[i] = MasToSla_pkg.Vm[i+1];
		MasToSla_pkg.Fv[i] = MasToSla_pkg.Fv[i+1];
		MasToSla_pkg.mst_Xm_P[i] = MasToSla_pkg.mst_Xm_P[i+1];
	}
	MasToSla_pkg.Xm[DT-1] = Xm*Scale;
	MasToSla_pkg.Vm[DT-1] = Vm;
	MasToSla_pkg.Fv[DT-1] = Fv;
	MasToSla_pkg.mst_Xm_P[DT-1] = mst_Xm_P;

	//package
	MasToSla.point.x = MasToSla_pkg.Xm[0];
	MasToSla.point.y = MasToSla_pkg.Fv[0];
	MasToSla.point.z = MasToSla_pkg.Vm[0];
	MasToSlaEnergy.point.x = MasToSla_pkg.mst_Xm_P[0];
	//publish
	Mas2Sla_pub.publish(MasToSla);
	Mas2SlaEnergy_pub.publish(MasToSlaEnergy);
}

void SlaToMasVsDelay(const geometry_msgs::PointStampedConstPtr &msg)
{
	unsigned i;
	for(i=0; i < DT-1; i++)
	{
		SlaToMas_pkg.Vs[i] = SlaToMas_pkg.Vs[i+1];
		SlaToMas_pkg.sla_Vs_P[i] = SlaToMas_pkg.sla_Vs_P[i+1];
		SlaToMas_pkg.Fs[i] = SlaToMas_pkg.Fs[i+1];
	}

	SlaToMas_pkg.Vs[DT-1] = msg->point.x;
	SlaToMas_pkg.sla_Vs_P[DT-1] = msg->point.y;
	SlaToMas_pkg.Fs[DT-1] = msg->point.z;

	Vs = SlaToMas_pkg.Vs[0];
	sla_Vs_P = SlaToMas_pkg.sla_Vs_P[0];
	Fs = SlaToMas_pkg.Fs[0];
}

void SlaToMasFeDelay(const geometry_msgs::PointStampedConstPtr &msg)
{
	unsigned i;
	for(i=0; i < DT-1; i++)
	{
		SlaToMas_pkg.Fe[i] = SlaToMas_pkg.Fe[i+1];
		SlaToMas_pkg.sla_Fe_P[i] = SlaToMas_pkg.sla_Fe_P[i+1];
	}

	SlaToMas_pkg.Fe[DT-1] = msg->point.x;
	SlaToMas_pkg.sla_Fe_P[DT-1] = msg->point.y;

	Fe = SlaToMas_pkg.Fe[0];
	sla_Fe_P = SlaToMas_pkg.sla_Fe_P[DT-1];
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
	Mas2Sla_pub = nh.advertise<geometry_msgs::PointStamped>("/MasToSla",1);
	Mas2SlaEnergy_pub = nh.advertise<geometry_msgs::PointStamped>("/MasToSlaEnergy",1);

	//Subcriber
	ros::Subscriber SlaToMasVs_sub = nh.subscribe<geometry_msgs::PointStamped>("/SlatoMasVs", 1,SlaToMasVsDelay);
	ros::Subscriber SlaToMasFv_sub = nh.subscribe<geometry_msgs::PointStamped>("/SlatoMasFe", 1,SlaToMasFeDelay);

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

	double	Acc=0;
	double  Mss = 0.0001;

	Xprv = X;
	hdBeginFrame(hdGetCurrentDevice());

	hdGetDoublev(HD_CURRENT_POSITION,posistion);
	if (start)
	{
		X0=posistion[0];
		start = false;
		hdEndFrame(Device);
		return HD_CALLBACK_CONTINUE;
	}

	X = posistion[0]-X0;
	V = X - Xprv;

	/*
	 * Velocity Command channel
	 */
	// Calculate Positive Energy
	if (Fs*Xm>0)
	{
		mst_Xm_P += Fs*Xm*Scale;
	}
	else
	{
		// Do nothing
	}
	//ROS_INFO("Xm channel Energy: %5f",mst_Xm_P);
	/*
	 * Velocity feedback channel
	 */
#if CASE1
	Fv = FvGain*Vs;
#else
	Fv = -0.01*(Xm*Scale-Vs);
#endif
	if (Fv*Vs>0)
	{
		mst_Vs_N -=Fv*Vs;
	}
	else
	{
		//Do nothing
	}
	//ROS_INFO("Energy - Velocity - Force: %5f - %5f - %5f",sla_Vs_P,Vs,Fv);
	//PC
#if (PC_Fv_On)
	{
		if (mst_Vs_N+sla_Vs_P<0)
		{
			mst_Vs_N +=Fv*Vs; // backward 1 step

			// Modify Vs
			if (Fv*Fv>0)
			{
				Vs = (mst_Vs_N+sla_Vs_P)/Fv;
			}
			else
			{
				Vs = 0;
			}

			// update
#if CASE1
			Fv = FvGain*Vs;
#else
			Fv = -0.01*(Xm*Scale-Vs);
#endif

			mst_Vs_N -=Fv*Vs;
			ROS_INFO("PC");
		}
	}
#endif

	/*
	 * Obstacle force
	 */
	if (Fe*Vm>0)
		{
			mst_Fe_N-=Fe*Vm; //fe channel output energy
		}
		else
		{

		}

		//	fprintf(mstEnergy,"%.3f \n",mstE_N_fe);
		//	fprintf(slvEnergy,"%.3f \n",slaE_P_fe);
		//	ROS_INFO("Master fe Energy: %5f	",mstE_N_fe);
	#if PC_Fe_On
		if (mst_Fe_N+sla_Fe_P<0) //output+input <0
		{
			mst_Fe_N+=Fe*Vm; //backward 1 step

			// Modify Fe
			if (Vm*Vm > 0.0)
				Fe = (mst_Fe_N+sla_Fe_P)/Vm;
			else
				Fe = 0;

			// Update
			mst_Fe_N-=Fe*Vm;
		}
	#endif

	/*
	 * Combination
	 */
#if FeOn
	Fm = Fv + Fe;
#else
	Fm = Fv;
#endif
	Acc = ( -1*K_Master*( Xm - X ) + Fm )/Mss;

	Vm += Acc*0.001;

	Xm += Vm*0.001;

	Fm = K_Master*(Xm - X);
	ROS_INFO("Velocity Feedback Force: %5f",Fe);
//	ROS_INFO("Vsd: %5f",Xm*Scale);
	force[2] = 0;
	force[1] = 0;
	force[0] = Fm;
	Saturation(force);
	//hdSetDoublev(HD_CURRENT_FORCE, force);
	hdEndFrame(hdGetCurrentDevice());

	MasToSlaDelay();
	return HD_CALLBACK_CONTINUE;
}
