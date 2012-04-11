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

#define PC_ON 0

/*
 * Device Struct
 */
FILE *mstForce;
FILE *slvForce;

FILE *mstEnergy;
FILE *slvEnergy;

FILE *data;

const int DT_MAX=5000;
int DT=500;
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


double K_Master=1;
double Scale = 0.1;

double Xm; //virtual mass spring pos
double X; //real posisiton
double Xprv; //prv posisiton
double X0; //Init position
double fm; //force
double fk;
double delta; //velocity
double mstE_P_Xm; // Xm chanel
double mstE_N_fe; // fe chanel
double mstE_N_fk; // fk chanel
double Vel;
/*
 * Slave
 */

double Vs;
double fe;
double fs;

double slaE_P_fe; // fe chanel
double slaE_P_fk; // fk chanel

/*
 * Package
 */
struct  Master_To_Slave
{
	double Xm[DT_MAX]; 			//x - Xm
	double delta[DT_MAX];		//y - delta
	double fk[DT_MAX];		//y - delta
	double EP[DT_MAX];			//z - mstE_P_Xm
} MasToSla;

struct SlaveToMaster
{
	double fs[DT_MAX]; 			// fs
	double vs[DT_MAX];			// vs
	double fe[DT_MAX];			// fe

	double E_P_fe[DT_MAX];		//Sla E_P_Fe;
	double E_P_fk[DT_MAX];		//Sla E_P_Fk;
} SlaToMas;

/*
 * ROS Variable
 */
ros::Publisher pos_pub;
ros::Publisher energy_pub;
geometry_msgs::PointStamped pos;
geometry_msgs::PointStamped energy;





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
	fm = 0.0;
	Xm = 0.0;
	X = 0.0;
	Xprv = 0.0;
	Vel = 0.0;
	mstE_N_fe=0.0;
	mstE_N_fk = 0.0;
	mstE_P_Xm = 0.0;

	slaE_P_fe = 0.0;
	slaE_P_fk = 0.0;
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

void DelayForward()
{
	unsigned int i;

	//Makde delay
	for(i=0; i < DT-1; i++)
	{
		MasToSla.Xm[i] = MasToSla.Xm[i+1];
		MasToSla.delta[i] = MasToSla.delta[i+1];
		MasToSla.fk[i] = MasToSla.fk[i+1];
		MasToSla.EP[i] = MasToSla.EP[i+1];
	}
	MasToSla.Xm[DT-1] = Xm/Scale;
	MasToSla.delta[DT-1] = Vel;
	MasToSla.fk[DT-1] = fk;
	MasToSla.EP[DT-1] = mstE_P_Xm;

	//package
	pos.point.x = MasToSla.Xm[0];
	pos.point.y = MasToSla.delta[0];
	pos.point.z = MasToSla.fk[0];

	energy.point.x = MasToSla.EP[0];

	//ROS_INFO("Master Position: %5f",pos.point.x);
	pos_pub.publish(pos);
	energy_pub.publish(energy);


}
void DelayBackwardParameter(const geometry_msgs::PointStampedConstPtr &msg)
{
	unsigned int i;

	for(i=0; i < DT-1; i++)
	{
		SlaToMas.vs[i] = SlaToMas.vs[i+1];
		SlaToMas.fs[i] = SlaToMas.fs[i+1];
		SlaToMas.fe[i] = SlaToMas.fe[i+1];
	}

	SlaToMas.vs[DT-1] = msg->point.x;
	SlaToMas.fs[DT-1] = msg->point.y;
	SlaToMas.fe[DT-1] = msg->point.z;

	Vs = SlaToMas.vs[0];
	fe = SlaToMas.fe[0];
	fs = SlaToMas.fs[0];

	//	fprintf(slvForce,"%.3f \n",fs+fe);
}

void DelayBackwardEnergy(const geometry_msgs::PointStampedConstPtr &msg)
{
	unsigned int i;

	for(i=0; i < DT-1; i++)
	{
		SlaToMas.E_P_fe[i] = SlaToMas.E_P_fe[i+1];
		SlaToMas.E_P_fk[i] = SlaToMas.E_P_fk[i+1];
	}

	SlaToMas.E_P_fe[DT-1] = msg->point.x;
	SlaToMas.E_P_fk[DT-1] = msg->point.y;

	slaE_P_fe = SlaToMas.E_P_fe[0];
	slaE_P_fk = SlaToMas.E_P_fk[0];
}


int main(int argc, char ** argv)
{
	//Init Phantom
	phantomOpen();
	printf("Phantom Opened \n");

	//Init ROS
	ros::init(argc, argv , "phantom_slave");

	ros::NodeHandle nh;
	mstForce = fopen("fm.txt","w");
	slvForce = fopen("fs.txt","w");
	mstEnergy= fopen("mstE.txt","w");
	slvEnergy= fopen("slvE.txt","w");
	data= fopen("data.txt","w");

	pos_pub = nh.advertise<geometry_msgs::PointStamped>("master_position",1);
	energy_pub = nh.advertise<geometry_msgs::PointStamped>("master_energy",1);
	ros::Subscriber paraSub = nh.subscribe<geometry_msgs::PointStamped>("/chanelParameter", 1,DelayBackwardParameter);
	ros::Subscriber enerSub = nh.subscribe<geometry_msgs::PointStamped>("/chanelEnergy", 1,DelayBackwardEnergy);

	//Asign callback
	hPhantomMain = hdScheduleAsynchronous(phantom_callback, 0, HD_MAX_SCHEDULER_PRIORITY);




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
	fclose(mstForce);
	fclose(slvForce);
	fclose(mstEnergy);
	fclose(slvEnergy);
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

	//X = ceil(posistion[2])-X0;
	X = posistion[0]-X0;
	delta = X - Xprv;

	// calculate fk
	fk = -0.01*(Xm/Scale-Vs);

	/*
	 * Xm Chanel
	 */

	if (fs*Xm/Scale>0)
	{
		mstE_P_Xm += fs*Xm/Scale;
	}
	else
	{
		//nothing
		//	mstE_P_Xm -= fs*Xm/Scale;
	}

	//ROS_INFO("Master Xm Energy: %5f	",mstE_P_Xm);

	//fprintf(mstEnergy,"%.3f \n",mstE_P_Xm);
	/*
	 * fe
	 */

	if (fe*Vel>0)
	{
		mstE_N_fe-=fe*Vel; //fe channel output energy
	}
	else
	{

	}

	//	fprintf(mstEnergy,"%.3f \n",mstE_N_fe);
	//	fprintf(slvEnergy,"%.3f \n",slaE_P_fe);
	//	ROS_INFO("Master fe Energy: %5f	",mstE_N_fe);
#if PC_ON
	if (mstE_N_fe+slaE_P_fe<0) //output+input <0
	{
		mstE_N_fe+=fe*Vel;
		if (Vel*Vel > 0.0)
			fe = (mstE_N_fe+slaE_P_fe)/Vel;
		else
			fe = 0;
		mstE_N_fe-=fe*Vel;
	}
#endif

	/*
	 * fk
	 */

	if (fk*Vs>0)
	{
		mstE_N_fk -=fk*Vs;
	}
	else
	{
		//	mstE_N_fk -=fk*Vs;
	}
	//	fprintf(mstEnergy,"%.3f \n",mstE_N_fk);
	//	fprintf(slvEnergy,"%.3f \n",slaE_P_fk);

	//	ROS_INFO("Master fk Energy: %5f	",mstE_N_fk);
	//	ROS_INFO("Master Energy: %5f",mstE_N_fk);
	//	ROS_INFO("Slave Energy: %5f",slaE_P_fk);
#if PC_ON
	if (mstE_N_fk+slaE_P_fk<0) //output+input <0
	{
		mstE_N_fk+=fk*Vs;
		if (fk*fk > 0.0)
			Vs = (mstE_N_fk+slaE_P_fk)/fk;
		else
			Vs = 0.0;
		fk = -0.01*(Xm/Scale-Vs);
		//fk = (mstE_N_fk+slaE_P_fk)/Vs;
		mstE_N_fk-=fk*Vs;
	}

#endif

#if PC_ON

	//Combination
	fm = fe+fk;
	//fm = fk;
	//fm = fe;
	//Mass-Spring

	Acc = ( -1*K_Master*( Xm - X ) + fm )/Mss;

	Vel += Acc*0.001;

	Xm += Vel*0.001;

	fm = K_Master*(Xm - X);

	//ROS_INFO("Force: %5f",fm);
	//ROS_INFO("Force: %5f",fe);

#endif

#if !PC_ON
	Xm=X;
	//	fm = fe+fk;
	fm = fk;
#endif
	// to Haptic
	force[2] = 0;
	force[1] = 0;
	force[0] = fm;

	Saturation(force);

	hdSetDoublev(HD_CURRENT_FORCE, force);
	hdEndFrame(hdGetCurrentDevice());

	//	fprintf(mstForce,"%.3f \n",fe);
	fprintf(data,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f \n",fm,fe+fs,fe,Vs,Xm/Scale,mstE_N_fe,slaE_P_fe,mstE_N_fk,slaE_P_fk);
	DelayForward();
	return HD_CALLBACK_CONTINUE;
}
