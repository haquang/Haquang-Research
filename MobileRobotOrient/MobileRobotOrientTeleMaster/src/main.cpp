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


#define FvOn 1 // Velocity based force Enable
#define FveOn 0 // Velocity error based force enable
#define FeOn 0 // Obstacle distance based force enable

#define PC_Fv_On 0
#define PC_Fve_On 0
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
int DT=1;
int MAX_FORCE=5;

FILE *data;

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
PID pidZ( 0.02 , 0.001, 0.001);

double X0; // Initial Position
double Xprv; // Previous Position
double X; // Master Position
double Xm; // Mass spring Position
double Vx; // Master Velocity
double Vxm; // Mass spring Velocity
double Fxve; // Velocity error Based Force

double Fxm; // Master force
double K_Master;
double FxveGain;
double velScale;
//Energy
double mst_Vxs_N; // Negative Energy in Velocity feedback channel
double mst_Fxe_N; // Negative Energy in Force feedback channel
double mst_Fxv_N; // Negative Energy in Force feedback channel
double mst_Xm_P; // Positive Energy in Velocity Command channel

double Y0; // Initial Position
double Yprv; // Previous Position
double Y; // Master Position
double Ym; // Mass spring Position
double Vy; // Master Velocity
double Vym; // Mass spring Velocity
double angScale;
double Fym; // Master force
//Energy
double mst_Vys_N; // Negative Energy in Velocity feedback channel
double mst_Fye_N; // Negative Energy in Force feedback channel
double mst_Fyv_N; // Negative Energy in Force feedback channel
double mst_Ym_P; // Positive Energy in Velocity Command channel


double Z0; // Initial Position
double Zprv; // Previous Position
double Z; // Master Position
/*
 * Slave
 */
double Vxs; // Slave Velocity
double Fxe; // Obstacle distance based force
double Fxs; // Virtual Force for energy calculating
double Fxv; // Velocity Based Force for energy calculating

// Energy
double sla_Vxs_P; // Positive Energy in Velocity feedback channel
double sla_Fxe_P; // Positive Energy in Force feedback energy
double sla_Fxv_P; // Positive Energy in Velocity Based force feedback energy

double Vys; // Slave Velocity
double Fye; // Obstacle distance based force
double Fys; // Obstacle distance based force
// Energy
double sla_Fye_P; // Positive Energy in Force feedback energy


/*
 * Master to Slave
 */
struct  Master_To_Slave
{
	double Xm[DT_MAX];			// Master Position
	double Vxm[DT_MAX];			// Master Velocity
	double Fxve[DT_MAX];			// Send calculated force to slave side
	double mst_Xm_P[DT_MAX];	// Positive Energy in velocity command channel

	double Ym[DT_MAX];			// Master Position
	double Vym[DT_MAX];			// Master Velocity
	double Fyve[DT_MAX];			// Send calculated force to slave side
	double mst_Ym_P[DT_MAX];	// Positive Energy in velocity command channel
} MasToSla_pkg;

/*
 * Slave to Master
 */
struct  Slave_To_Master
{
	double Vxs[DT_MAX]; // Slave Velocity
	double Fxe[DT_MAX]; // Obstacle distance based force
	double Fxv[DT_MAX]; // Velocity based force
	double Fxs[DT_MAX]; //Virtual Force for energy calculating
	double sla_Vxs_P[DT_MAX]; // Positive Energy in Velocity feedback channel
	double sla_Fxe_P[DT_MAX]; // Positive Energy in Force feedback energy
	double sla_Fxv_P[DT_MAX]; // Positive Energy in Velocity Based Force feedback energy

	double Fye[DT_MAX]; // Obstacle distance based force
	double Fys[DT_MAX]; //Virtual Force for energy calculating
	double sla_Fye_P[DT_MAX]; // Positive Energy in Force feedback energy

} SlaToMas_pkg;


/*
 * ROS Variable
 */
ros::Publisher Mas2SlaVel_pub;
ros::Publisher Mas2SlaEnergyVel_pub;
geometry_msgs::PointStamped MasToSlaVel;
geometry_msgs::PointStamped MasToSlaEnergy;

ros::Publisher Mas2SlaAng_pub;
ros::Publisher Mas2SlaEnergyAng_pub;
geometry_msgs::PointStamped MasToSlaAng;
geometry_msgs::PointStamped MasToSlaEnergyAng;

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

	Vx = 0;
	Vxm = 0;
	Vxs = 0;


	mst_Fxe_N = 0;
	mst_Vxs_N = 0;
	mst_Xm_P = 0;

	Fxe = 0;
	Fxv = 0;
	Fxve = 0;

	FxveGain = 0.1;
	velScale = 2;
	K_Master = 2;

	Y0 = 0;
	Ym = 0;
	Yprv = 0;

	Vy = 0;
	Vym = 0;
	Vys = 0;


	mst_Fye_N = 0;
	mst_Vys_N = 0;
	mst_Ym_P = 0;

	Fye = 0;
	angScale = 0.001;

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
		MasToSla_pkg.Vxm[i] = MasToSla_pkg.Vxm[i+1];
		MasToSla_pkg.Fxve[i] = MasToSla_pkg.Fxve[i+1];
		MasToSla_pkg.mst_Xm_P[i] = MasToSla_pkg.mst_Xm_P[i+1];

		MasToSla_pkg.Ym[i] = MasToSla_pkg.Ym[i+1];
		MasToSla_pkg.Vym[i] = MasToSla_pkg.Vym[i+1];
		MasToSla_pkg.Fyve[i] = MasToSla_pkg.Fyve[i+1];
		MasToSla_pkg.mst_Ym_P[i] = MasToSla_pkg.mst_Ym_P[i+1];
	}
	MasToSla_pkg.Xm[DT-1] = Xm*velScale;
	MasToSla_pkg.Vxm[DT-1] = Vxm;
	MasToSla_pkg.Fxve[DT-1] = Fxve;
	MasToSla_pkg.mst_Xm_P[DT-1] = mst_Xm_P;

	MasToSla_pkg.Ym[DT-1] = Ym*angScale;
	MasToSla_pkg.Vym[DT-1] = Vym;
	MasToSla_pkg.mst_Ym_P[DT-1] = mst_Ym_P;

	//package
	MasToSlaVel.point.x = MasToSla_pkg.Xm[0];
	MasToSlaVel.point.y = MasToSla_pkg.Fxve[0];
	MasToSlaVel.point.z = MasToSla_pkg.Vxm[0];
	MasToSlaEnergy.point.x = MasToSla_pkg.mst_Xm_P[0];

	MasToSlaAng.point.x = MasToSla_pkg.Ym[0];
	MasToSlaAng.point.y = MasToSla_pkg.Fyve[0];
	MasToSlaAng.point.z = MasToSla_pkg.Vym[0];
	MasToSlaEnergy.point.y = MasToSla_pkg.mst_Ym_P[0];

	//publish
	Mas2SlaVel_pub.publish(MasToSlaVel);
	Mas2SlaAng_pub.publish(MasToSlaAng);
	Mas2SlaEnergyVel_pub.publish(MasToSlaEnergy);
}

void SlaToMasVxsDelay(const geometry_msgs::PointStampedConstPtr &msg)
{
	unsigned i;
	for(i=0; i < DT-1; i++)
	{
		SlaToMas_pkg.Vxs[i] = SlaToMas_pkg.Vxs[i+1];
		SlaToMas_pkg.sla_Vxs_P[i] = SlaToMas_pkg.sla_Vxs_P[i+1];
		SlaToMas_pkg.Fxs[i] = SlaToMas_pkg.Fxs[i+1];
	}

	SlaToMas_pkg.Vxs[DT-1] = msg->point.x;
	SlaToMas_pkg.sla_Vxs_P[DT-1] = msg->point.y;
	SlaToMas_pkg.Fxs[DT-1] = msg->point.z;

	Vxs = SlaToMas_pkg.Vxs[0];
	sla_Vxs_P = SlaToMas_pkg.sla_Vxs_P[0];
	Fxs = SlaToMas_pkg.Fxs[0];
}

void SlaToMasFxeDelay(const geometry_msgs::PointStampedConstPtr &msg)
{
	unsigned i;
	for(i=0; i < DT-1; i++)
	{
		SlaToMas_pkg.Fxe[i] = SlaToMas_pkg.Fxe[i+1];
		SlaToMas_pkg.sla_Fxe_P[i] = SlaToMas_pkg.sla_Fxe_P[i+1];
	}

	SlaToMas_pkg.Fxe[DT-1] = msg->point.x;
	SlaToMas_pkg.sla_Fxe_P[DT-1] = msg->point.y;

	Fxe = SlaToMas_pkg.Fxe[0];
	sla_Fxe_P = SlaToMas_pkg.sla_Fxe_P[DT-1];
}

void SlaToMasFxvDelay(const geometry_msgs::PointStampedConstPtr &msg)
{
	unsigned i;
	for(i=0; i < DT-1; i++)
	{
		SlaToMas_pkg.Fxv[i] = SlaToMas_pkg.Fxv[i+1];
		SlaToMas_pkg.sla_Fxv_P[i] = SlaToMas_pkg.sla_Fxv_P[i+1];
	}

	SlaToMas_pkg.Fxv[DT-1] = msg->point.x;
	SlaToMas_pkg.sla_Fxv_P[DT-1] = msg->point.y;

	Fxv = SlaToMas_pkg.Fxv[0];
	sla_Fxv_P = SlaToMas_pkg.sla_Fxv_P[DT-1];
}

void SlaToMasFyeDelay(const geometry_msgs::PointStampedConstPtr &msg)
{
	unsigned i;
	for(i=0; i < DT-1; i++)
	{
		SlaToMas_pkg.Fye[i] = SlaToMas_pkg.Fye[i+1];
		SlaToMas_pkg.sla_Fye_P[i] = SlaToMas_pkg.sla_Fye_P[i+1];
		SlaToMas_pkg.Fys[i] = SlaToMas_pkg.Fys[i+1];
	}

	SlaToMas_pkg.Fye[DT-1] = msg->point.x;
	SlaToMas_pkg.sla_Fye_P[DT-1] = msg->point.y;
	SlaToMas_pkg.Fys[DT-1] = msg->point.z;

	Fye = SlaToMas_pkg.Fye[0];
	sla_Fye_P = SlaToMas_pkg.sla_Fye_P[DT-1];
	Fys = SlaToMas_pkg.Fys[0];
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
	// Publisher
	Mas2SlaVel_pub = nh.advertise<geometry_msgs::PointStamped>("/MasToSlaVel",1);
	Mas2SlaAng_pub = nh.advertise<geometry_msgs::PointStamped>("/MasToSlaAng",1);
	Mas2SlaEnergyVel_pub = nh.advertise<geometry_msgs::PointStamped>("/MasToSlaVelEnergy",1);

	//Subcriber
	ros::Subscriber SlaToMasVxs_sub = nh.subscribe<geometry_msgs::PointStamped>("/SlatoMasVxs", 1,SlaToMasVxsDelay);
	ros::Subscriber SlaToMasFxe_sub = nh.subscribe<geometry_msgs::PointStamped>("/SlatoMasFxe", 1,SlaToMasFxeDelay);
	ros::Subscriber SlaToMasFxv_sub = nh.subscribe<geometry_msgs::PointStamped>("/SlatoMasFxv", 1,SlaToMasFxvDelay);

	ros::Subscriber SlaToMasFye_sub = nh.subscribe<geometry_msgs::PointStamped>("/SlatoMasFye", 1,SlaToMasFyeDelay);

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

	double	Acc=0;
	double  Mss = 0.0001;

	Xprv = X;
	Yprv = Y;
	hdBeginFrame(hdGetCurrentDevice());

	hdGetDoublev(HD_CURRENT_POSITION,posistion);
	if (start)
	{
		X0=posistion[2];
		Y0=posistion[0];
		Z0=posistion[1];
		start = false;
		hdEndFrame(Device);
		return HD_CALLBACK_CONTINUE;
	}

	X = X0 - posistion[2];
	Vx = X - Xprv;

	Y = Y0 - posistion[0];
	Vy = Y - Yprv;

	Z = posistion[1]-Z0;

	/*
	 * Velocity Command channel
	 */
	// Calculate Positive Energy
	if (Fxs*Xm>0)
	{
		mst_Xm_P += Fxs*Xm*velScale;
	}
	else
	{
		// Do nothing
	}
	//ROS_INFO("Xm channel Energy: %5f",mst_Xm_P);
	/*
	 * Velocity feedback channel
	 */
	Fxve = -FxveGain*(Xm*velScale-Vxs);
	if (Fxve*Vxs>0)
	{
		mst_Vxs_N -=Fxve*Vxs;
	}
	else
	{
		//Do nothing
	}
	//ROS_INFO("Energy - Velocity - Force: %5f - %5f - %5f",mst_Vs_N,Vs,Fve);
	//PC
#if (PC_Fve_On)
	{
		if (mst_Vxs_N+sla_Vxs_P<0)
		{
			mst_Vxs_N +=Fxve*Vxs; // backward 1 step

			// Modify Vs
			if (Fxve*Fxve>0)
			{
				Vxs = (mst_Vxs_N+sla_Vxs_P)/Fxve;
			}
			else
			{
				Vxs = 0;
			}

			// update

			Fxve = -FxveGain*(Xm*velScale-Vxs);
			mst_Vxs_N -=Fxve*Vxs;
			ROS_INFO("PC");
		}
	}
#endif

	/*
	 * Obstacle force
	 */
	if (Fxe*Vxm>0)
	{
		mst_Fxe_N-=Fxe*Vxm; //fe channel output energy
	}
	else
	{

	}
	//	ROS_INFO("Energy - Velocity - Force: %5f - %5f - %5f",mst_Fe_N,Vm,Fe);
	//	fprintf(mstEnergy,"%.3f \n",mstE_N_fe);
	//	fprintf(slvEnergy,"%.3f \n",slaE_P_fe);
	//	ROS_INFO("Master fe Energy: %5f	",mstE_N_fe);

#if PC_Fe_On
	if (mst_Fxe_N+sla_Fxe_P<0) //output+input <0
	{
		mst_Fxe_N+=Fxe*Vxm; //backward 1 step

		// Modify Fe
		if (Vxm*Vxm > 0.0)
			Fxe = (mst_Fxe_N+sla_Fxe_P)/Vxm;
		else
			Fxe = 0;
		ROS_INFO("PC X");
		// Update
		mst_Fxe_N-=Fxe*Vxm;
	}
#endif

	/*
	 * Velocity Based force
	 */
	if (Fxv*Vxm>0)
	{
		mst_Fxv_N-=Fxv*Vxm; //fe channel output energy
	}
	else
	{

	}
	//	ROS_INFO("Energy - Velocity - Force: %5f - %5f - %5f",mst_Fe_N,Vm,Fe);
	//	fprintf(mstEnergy,"%.3f \n",mstE_N_fe);
	//	fprintf(slvEnergy,"%.3f \n",slaE_P_fe);
	//	ROS_INFO("Master fe Energy: %5f	",mstE_N_fe);

#if PC_Fv_On
	if (mst_Fxv_N+sla_Fxv_P<0) //output+input <0
	{
		mst_Fxv_N+=Fxv*Vxm; //backward 1 step

		// Modify Fe
		if (Vxm*Vxm > 0.0)
			Fxv = (mst_Fxv_N+sla_Fxv_P)/Vxm;
		else
			Fxv = 0;

		// Update
		mst_Fxv_N-=Fxv*Vxm;
		ROS_INFO("PC");
	}
#endif

	/*
	 * Combination
	 */
#if !FeOn
	Fxe = 0;
#endif
#if !FveOn
	Fxve = 0;
#endif
#if !FvOn
	Fxv = 0;
#endif

#if (PC_Fe_On||PC_Fve_On||PC_Fv_On)
	Fxm = Fxe + Fxve + Fxv;

	Acc = ( -1*K_Master*( Xm - X ) + Fxm )/Mss;

	Vxm += Acc*0.001;

	Xm += Vxm*0.001;

	Fxm = K_Master*(Xm - X);
#else
	Xm = X;
	Vxm = Vx;
	Fxm = Fxe +Fxve+Fxv;
#endif


	/*
	 * Heading Angle Command channel
	 */
	//ROS_INFO("Rot Velocity: %5f   %5f",Ym*angScale,Vys);
	// Calculate Positive Energy
	if (Fys*Ym>0)
	{
		mst_Ym_P += Fys*Ym*angScale;
	}
	else
	{
		// Do nothing
	}


	/*
	 * Obstacle force
	 */
	//ROS_INFO("Rot Force: %5f",Fye);
	if (Fye*Vym>0)
	{
		mst_Fye_N-=Fye*Vym; //fe channel output energy
	}
	else
	{

	}
#if PC_Fe_On

	if (mst_Fye_N+sla_Fye_P<0) //output+input <0
	{
		mst_Fye_N+=Fye*Vym; //backward 1 step

		// Modify Fe
		if (Vym*Vym > 0.0)
			Fye = (mst_Fye_N+sla_Fye_P)/Vym;
		else
			Fye = 0;
		ROS_INFO("PC Y");
		// Update
		mst_Fye_N-=Fye*Vym;
	}
#endif

	/*
	 * Combination
	 */
#if !FeOn
	Fye = 0;
#endif

#if (PC_Fe_On||PC_Fve_On||PC_Fv_On)
	Fym = Fye;

	Acc = ( -1*K_Master*( Ym - Y ) + Fym )/Mss;

	Vym += Acc*0.001;

	Ym += Vym*0.001;

	Fym = K_Master*(Ym - Y);
#else
	Ym = Y;
	Vym = Vy;
	Fym = Fye;
#endif


	//	ROS_INFO("Obstacle Feedback Force: %5f",Fe);
	//	ROS_INFO("Vsd: %5f",Xm*velScale);
	//  ROS_INFO("Force: %5f",Fym);
	force[2] = -Fxm;
	force[1] = pidZ.compute(0 , Z);
	force[0] = -Fym;
	Saturation(force);
	hdSetDoublev(HD_CURRENT_FORCE, force);
	hdEndFrame(hdGetCurrentDevice());

	MasToSlaDelay();
	return HD_CALLBACK_CONTINUE;
}
