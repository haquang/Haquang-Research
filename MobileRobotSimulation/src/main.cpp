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

#define Xm_PoPc 1
#define Vs_PoPc 1
#define damper 1
#define fake 1

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

		//output  = Kp *error  + Kd*(error - pre_error)/dt  + integral;
		output  = Kp *error  + Kd*(error - pre_error)/dt;
		//
		pre_error = error;

		if(output > max)
		{
			output = max;
		}
		else if(output < min)
		{
			output = min;
		}
		return output;
	}
};

/*
 * Device Struct
 */
const int DT_MAX=5000;
int DT=500;
int MAX_FORCE=5;
double t=0;
double dT = 0.001;
FILE *data;
// Global Variable
HDErrorInfo error;
HDCallbackCode hOmniCallback;

static HHD Device;
static HDSchedulerHandle hPhantomMain;
bool start = true;
const double dt = 0.001;
/*
 * Master
 */
hduVector3Dd init_pos;
hduVector3Dd posistion;
hduVector3Dd force;

double X0; // Initial Position
double X01; // Initial Position
double X02; // Initial Position
double X1;
double X2;

double Xprv; // Previous Position
double Xm; // Master Position
double Vm; // Master Velocity

double Fm; // Master force
double K_Master;
double Scale;
double FvGain;
double Vm_delay;

double mst_Xm_P;
double mst_Xm_P_delay;
double mst_Fv_N;
/*
 * Slave
 */
double M; // Mass of Mobile Robot
double b; // Friction

double Vsd;
double Vs; // Slave Velocity
double Fs; // Slave Force
double Fs_fake; // Slave Force fake
double Fs_cal; // Slave force for energy calculation
double Vsdot; // Slave Acceleration
double Fv;

double Fs_delay;
double Fv_delay;
double slv_Xm_N;
double slv_Fv_P;
double slv_Fv_P_delay;

double K_fake = 5;
double vs_err;
double vs_pr_err;
double Kp = 5;
double Kd = 0.1;
PID velController(5,0.1,0.1);
PID pidY( 0.02 , 0.001, 0.001);
PID pidZ( 0.02 , 0.001, 0.001);

/*
 * Master to Slave
 */
struct  Master_To_Slave
{
	double Xm[DT_MAX];			// Master Position
	double Vm[DT_MAX];			// Master Force
	double mst_Xm_P[DT_MAX];	// Positive Energy in velocity command channel
} MasToSla_pkg;

/*
 * Slave to Master
 */
struct  Slave_To_Master
{
	double Fv[DT_MAX]; // Slave Velocity
	double Fs[DT_MAX]; //Virtual Force for energy calculating
	double slv_Fv_P[DT_MAX]; // Positive Energy in Velocity feedback channel
} SlaToMas_pkg;

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
	Fm = 0;

	Vm = 0;
	Vs = 0;
	Fs_delay = 0;
	Fv_delay = 0;

	mst_Fv_N = 0;
	mst_Xm_P = 0;
	mst_Xm_P_delay = 0;

	slv_Xm_N = 0.0;
	slv_Fv_P = 0.0;
	slv_Fv_P_delay = 0.0;

	Scale = 0.01;
	K_Master = 1;
	FvGain = 2;

	M = 1;
	b = 0.1;

	vs_pr_err = 0;

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
 * Mobile Robot Simulation
 */
double MobileRobotModel(double v,double t,double F)
{
	double vDot;
	vDot = (1/M) *(F-b*v);
	return vDot;
}
double RK4(double v,double t,double dT,double F)
{
	double vCal;
	double K1,K2,K3,K4;

	K1 = dT* MobileRobotModel(v,t,F);
	K2 = dT* MobileRobotModel(v+0.5*dT*K1,t+dT/2,F);
	K3 = dT* MobileRobotModel(v+0.5*dT*K2,t+dT/2,F);
	K4 = dT* MobileRobotModel(v+dT*K3,t+dT,F);

	vCal = v + (K1 + 2 * K2 + 2 * K3 + K4)/6;
	return vCal;
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
		MasToSla_pkg.mst_Xm_P[i] = MasToSla_pkg.mst_Xm_P[i+1];
	}
	MasToSla_pkg.Xm[DT-1] = Xm*Scale;
	MasToSla_pkg.Vm[DT-1] = Vm;
	MasToSla_pkg.mst_Xm_P[DT-1] = mst_Xm_P;

	Vsd = MasToSla_pkg.Xm[0];
 	Vm_delay = MasToSla_pkg.Vm[0];
 	mst_Xm_P_delay = MasToSla_pkg.mst_Xm_P[0];
}

void SlaToMasDelay()
{
	unsigned i;
	for(i=0; i < DT-1; i++)
	{
		SlaToMas_pkg.Fv[i] = SlaToMas_pkg.Fv[i+1];
		SlaToMas_pkg.slv_Fv_P[i] = SlaToMas_pkg.slv_Fv_P[i+1];
		SlaToMas_pkg.Fs[i] = SlaToMas_pkg.Fs[i+1];
	}

	SlaToMas_pkg.Fv[DT-1] = Fv;
	SlaToMas_pkg.slv_Fv_P[DT-1] = slv_Fv_P;
	SlaToMas_pkg.Fs[DT-1] = Fs_cal;

	Fv_delay = SlaToMas_pkg.Fv[0];
	Fs_delay = SlaToMas_pkg.Fs[0];
	slv_Fv_P_delay = SlaToMas_pkg.slv_Fv_P[0];
}

int main(int argc, char ** argv)
{
	//Init Phantom
	phantomOpen();
	printf("Phantom Opened \n");

	//Init ROS
	ros::init(argc, argv , "phantom_slave");

	ros::NodeHandle nh;

	data= fopen("data.txt","w");
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
	fclose(data);
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
	double d = 5;
	Xprv = Xm;
	hdBeginFrame(hdGetCurrentDevice());

	hdGetDoublev(HD_CURRENT_POSITION,posistion);
	if (start)
	{
		X0=posistion[0];
		X01=posistion[1];
		X02=posistion[2];
		start = false;
		hdEndFrame(Device);
		return HD_CALLBACK_CONTINUE;
	}

	Xm = posistion[0]-X0;
	X1 = posistion[1]-X01;
	X2 = posistion[2]-X02;

	Vm = Xm - Xprv;

	/*
	 *  Mobile Robot
	 */
	Fv = -FvGain*Vs;
	vs_err = Vsd - Vs;
	Fs  = Kp *vs_err  + Kd*(vs_err - vs_pr_err)/dt;
	Fs_fake = K_fake*(Vsd - Vs);

	if (Fv*Vm_delay>0)
	{
		slv_Fv_P += Fv*Vm_delay;
	}

#if (fake)
	Fs_cal = Fs_fake;
#else
	Fs_cal = Fs;
#endif
	if (Vsd*Fs_cal>0)
	{
		slv_Xm_N -= Vsd*Fs_cal;
	}
#if (Xm_PoPc)
//	ROS_INFO("Energy : %5f  -  %5f   -   %5f ",mst_Xm_P_delay,slv_Xm_N,mst_Xm_P_delay+slv_Xm_N);
	if (mst_Xm_P_delay==0 ) slv_Xm_N =0;
	if (mst_Xm_P_delay+slv_Xm_N<0)
	{
		ROS_INFO("Xm POPC");
		slv_Xm_N += Vsd*Fs_cal;
		Vsd = (mst_Xm_P_delay+slv_Xm_N)/Fs_cal;
		vs_err = Vsd - Vs;
		Fs  = Kp *vs_err  + Kd*(vs_err - vs_pr_err)/dt;
		#if (fake)
			Fs_cal = Fs_fake;
		#else
			Fs_cal = Fs;
		#endif
		slv_Xm_N -= Vsd*Fs_cal;
	}
#endif
//	ROS_INFO("Slave : %5f  -  %5f   -   %5f ",Vsd,Vs,Fs);
	Vs = RK4(Vs,t,dT,Fs);
	/*
	 *  Master
	 */
	if (Scale*Xm*Fs_delay>0)
	{
		mst_Xm_P += Scale*Xm*Fs_delay;
	}

	if (Fv_delay*Vm>0)
	{
		mst_Fv_N -= Fv_delay*Vm;
	}
#if (Vs_PoPc)
	//ROS_INFO("Energy : %5f  -  %5f   -   %5f ",mst_Fv_N,slv_Fv_P_delay,mst_Fv_N+slv_Fv_P_delay);
	if (mst_Fv_N+slv_Fv_P_delay<0)
	{
		ROS_INFO("Vs POPC");
		mst_Fv_N += Fv_delay*Vm;
		Fv_delay  = (mst_Fv_N+slv_Fv_P_delay)/Vm;
		mst_Fv_N -= Fv_delay*Vm;
	}
#endif

#if damper
	Fm = Fv - d*Vm;
#else
	Fm = Fv;
#endif
//	ROS_INFO("Force - Velocity - Set Velocity: %5f  -  %5f   -   %5f ",Fs,Vs,Vsd);

	force[2] = pidZ.compute(0 , X2);
	force[1] = pidY.compute(0 , X1);
	force[0] = Fm;
	//ROS_INFO("Force : %5f  -  %5f   -   %5f ",force[0],force[1],force[2]);
	Saturation(force);
	hdSetDoublev(HD_CURRENT_FORCE, force);
	hdEndFrame(hdGetCurrentDevice());
	fprintf(data,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",Fs_cal,Vsd,Fs_delay, Scale*Xm,mst_Xm_P_delay,slv_Xm_N,mst_Fv_N,slv_Fv_P_delay);
	vs_pr_err = vs_err;
	MasToSlaDelay();
	SlaToMasDelay();
	return HD_CALLBACK_CONTINUE;
}
