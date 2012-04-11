// Haptic Device headers
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

using namespace std;

template <class T>
class PoPc{
private:
	T Fi; // Input Force
	T Fo; // Modified Force

	T Vi; // Input Velocity
	T Vo; // Modified Velocity

	T Ep; // Positive Energy
	T En; // Negative Energy

	bool type; // Admittance and Impedance
	bool active;
	int Dof;
public:

	PoPc() {
		// TODO Auto-generated constructor stub
		for (size_t j=0;j<Dof;j++)
		{
			Fi[j] = 0;
			Fo[j] = 0;

			Vi[j] = 0;
			Vo[j] = 0;

			Ep[j] = 0;
			En[j] = 0;

			Dof =1;
		}
	}

	void Config(bool typeSet,bool activePoPc,int Dof_set) {
		// TODO Auto-generated constructor stub
		type = typeSet;
		active = activePoPc;
		Dof = Dof_set;
	}
	void updatePoPc(T FiSet,T ViSet)
	{
		Fi = FiSet;
		Vi = ViSet;

		for (size_t j=0;j<Dof;j++)
		{
			if (Fi[j]*Vi[j]>0)
			{
				if (active) // calculate negative energy
					En[j] -= Fi[j]*Vi[j];
				else		// calculate positive energy
					Ep[j] += Fi[j]*Vi[j];
			}
		}
	}
	bool PcControl(T Ein)
	{
		if (!active)
		{
			return false;
		}
		else
		{
			if (type) //Modify Velocity
			{
				for (size_t j=0;j<Dof;j++)
				{
					if (Ein[j] + En[j] <0)
					{
						En[j] += Fi[j]*Vi[j];  //backward 1 step
						Vo[j] = (Ein[j] + En[j])/Fi[j];
						En[j] += Fi[j]*Vo[j];  //update 1 step
					}
					else
					{
						Vo[j] = Vi[j]; // Without any modification
					}
				}
			}

			else   // Modify Force
			{
				for (size_t j=0;j<Dof;j++)
				{
					if (Ein[j] + En[j] <0)
					{
						En[j] += Fi[j]*Vi[j];  //backward 1 step
						Fo[j] = (Ein[j] + En[j])/Vi[j];
						En[j] += Fo[j]*Vi[j];  //update 1 step
					}
					else
					{
						Fo[j] = Fi[j]; // Without any modification
					}
				}
			}
		}
		return true;
	}
	T getForce()
	{
		return Fo;
	}
	T getVel()
	{
		return Vo;
	}
	T getPositiveEnergy()
	{
		return Ep;
	}

	~PoPc() {
		// TODO Auto-generated destructor stub
	}
};



//Function definition
int OmniOpen(void);
int OmniRun(void);
void OmniClose(void);
void OmniCalibrate(void);
void mainLoop();
HDCallbackCode HDCALLBACK omni_callback(void *pUserData);
void sendMasterMarkerPos(const float &x, const float &y, const float &z);

// Global Variable
HDErrorInfo error;
HHD hHD;
HDCallbackCode hOmniCallback;

hduVector3Dd Position;
hduVector3Dd Force;
hduVector3Dd Position_init;
hduVector3Dd slaveEnergyForce;
hduVector3Dd VelSet = hduVector3Dd(0.0, 0.0, 0.0);
hduVector3Dd Position_prv = hduVector3Dd(0.0, 0.0, 0.0);

const int DT_MAX=5000;
int DT=500;
int i;
bool start = true;


ros::Publisher masterPosePub;
ros::Publisher masterEnergyPub;

geometry_msgs::Pose masterPose[DT_MAX];
geometry_msgs::Pose masterEnergyPosition[DT_MAX];

float Scale=-1;

PoPc<hduVector3Dd> PoPcPosChannel;
PoPc<hduVector3Dd> PoPcForceChannel;


void Saturation(double *force){
	if (force[0]<-5) force[0]=-5;
	if (force[1]<-5) force[1]=-5;
	if (force[2]<-5) force[2]=-5;
	if (force[0]>5) force[0]=5;
	if (force[1]>5) force[1]=5;
	if (force[2]>5) force[2]=5;
}

void MasToSlave_delay()
{
	for (i=0;i<DT-1;i++)
	{
		masterPose[i] = masterPose[i+1];
		masterEnergyPosition[i] = masterEnergyPosition[i+1];
	}
	masterPose[DT-1].position.x = VelSet[0];
	masterPose[DT-1].position.y = VelSet[1];
	masterPose[DT-1].position.z = VelSet[2];


	hduVector3Dd masterEnergyTemp = PoPcPosChannel.getPositiveEnergy();
	masterEnergyPosition[DT-1].position.x = masterEnergyTemp[0];
	masterEnergyPosition[DT-1].position.y = masterEnergyTemp[1];
	masterEnergyPosition[DT-1].position.z = masterEnergyTemp[2];

	masterEnergyPub.publish(masterEnergyPosition[0]);
	masterPosePub.publish(masterPose[0]);
}
void forceFeedbackCallback (const geometry_msgs::PosePtr msg)
{
	Force[0] = Scale*msg->position.x;
	Force[1] = Scale*msg->position.y;
	Force[2] = Scale*msg->position.z;
}
void slaveEnergyCallBack(const geometry_msgs::PosePtr msg)
{
	slaveEnergyForce[0] = msg->position.x;
	slaveEnergyForce[1] = msg->position.y;
	slaveEnergyForce[2] = msg->position.z;

}
void initData()
{
	PoPcPosChannel.Config(true,false,3); // PoPc in Position command channel - nonactive
	PoPcForceChannel.Config(false,true,3); // PoPc in Force feedback command channel - active, modify force
	for (i=0;i<DT-1;i++)
	{
		masterPose[i].position.x = 0;
		masterPose[i].position.y = 0;
		masterPose[i].position.z = 0;
	}
}
int main(int argc,char* argv[]) {

	initData();
	//Init Omini
	OmniOpen();
	//Init ROS
	ros::init(argc, argv , "omini_phantom");
	ros::NodeHandle nh;
	masterPosePub = nh.advertise<geometry_msgs::Pose>("PositionCommand",1);
	masterEnergyPub = nh.advertise<geometry_msgs::Pose>("MasterEnergy",1);
	ros::Subscriber slaveForce = nh.subscribe<geometry_msgs::PosePtr>("/ForceFeedback",1,forceFeedbackCallback);
	ros::Subscriber slaveEnergy = nh.subscribe<geometry_msgs::PosePtr>("/SlaveEnergy",1,slaveEnergyCallBack);

	//Asign callback
	hOmniCallback = hdScheduleAsynchronous(omni_callback, 0, HD_MAX_SCHEDULER_PRIORITY);
	//hdSetSchedulerRate(1000);

	//Start Omini Scheduler
	OmniRun();

	while (ros::ok() && nh.ok())
	{
		ros::spinOnce();
	}
	OmniClose();
	return 0;
}



int OmniOpen()
{
	// Initialize the default haptic device.
	HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		ROS_ERROR("Failed to open!");
		return -1;
	}

	// Start the servo scheduler and enable forces.
	hdEnable(HD_FORCE_OUTPUT);
	printf("Position x = %lf, y = %lf, z = %lf", Position[0], Position[1], Position[2] );
	OmniCalibrate();
	return 0; //if successful
}
int OmniRun()
{
	// Start Scheduler
	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		ROS_ERROR("Failed to run!");
		return -1;
	}
	return 0;
}

void OmniClose()
{
	hdStopScheduler();
	hdUnschedule(hOmniCallback);
	hdDisableDevice(hHD);
}
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

	printf("\n\nCalibration complete.\n");
}
HDCallbackCode HDCALLBACK omni_callback(void *pUserData)
{
	hdBeginFrame(hdGetCurrentDevice());

	hdGetDoublev(HD_CURRENT_POSITION, Position);
	if (start)
	{
		Position_init = Position;
		start = false;
		hdEndFrame(hdGetCurrentDevice());
		return HD_CALLBACK_CONTINUE;
	}

	Position = Position - Position_init;
	VelSet = Position - Position_prv;

	// Position Command Channel
	PoPcPosChannel.updatePoPc(Force,VelSet);

	// Force feedback Command Channel
	PoPcForceChannel.updatePoPc(Force,VelSet);
	if (PoPcForceChannel.PcControl(slaveEnergyForce))
	{
		Force = PoPcPosChannel.getForce();
	}

	Saturation(Force);
	ROS_INFO("Position: %5f --- %5f --- %5f", Position[0],Position[1],Position[2]);

	//ROS_INFO("Force: %5f --- %5f --- %5f", Force[0],Force[1],Force[2]);
	hdSetDoublev(HD_CURRENT_FORCE,Force);
	hdEndFrame(hdGetCurrentDevice());

	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Error during main scheduler callback\n");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}
	Position_prv = Position;
	MasToSlave_delay();
	return HD_CALLBACK_CONTINUE;

}
