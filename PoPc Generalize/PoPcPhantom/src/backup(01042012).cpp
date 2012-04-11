#include <iostream>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

class PD
{
private:
	double Kp;
	double Kd;
	double pre_error;
	static const double dt = 0.001;
	static const double max =  0.8;
	static const double min = -0.8;
public:
	PD(const double &Kp, const double &Kd, const double &Ki)
	{
		this->Kp = Kp;
		this->Kd = Kd;
		pre_error = 0.0;
	}

	double compute(const double &Xr, const double &X)
	{
		double error, output;

		error = Xr  - X;

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

class PoPc{
private:
	hduVector3Dd Fi; // Input Force
	hduVector3Dd Fo; // Modified Force

	hduVector3Dd Vi; // Input Velocity
	hduVector3Dd Vo; // Modified Velocity

	hduVector3Dd Ep; // Positive Energy
	hduVector3Dd En; // Negative Energy

	int type; // Admittance and Impedance
	bool active;
public:
	PoPc() {
		// TODO Auto-generated constructor stub
		for (int j=0;j<3;j++)
		{
			Fi[j] = 0;
			Fo[j] = 0;

			Vi[j] = 0;
			Vo[j] = 0;

			Ep[j] = 0;
			En[j] = 0;
		}
	}

	void Config(int typeSet,bool activePoPc) {
		// TODO Auto-generated constructor stub
		type = typeSet;
		active = activePoPc;
	}
	void updatePoPc(hduVector3Dd FiSet,hduVector3Dd ViSet)
	{
		Fi = FiSet;
		Vi = ViSet;

		for (int j=0;j<3;j++)
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
	int PcControl(hduVector3Dd Ein)
	{
		if (!active)
		{
			return -1;
		}
		else
		{
			if (type == 0) //Modify Velocity
			{
				for (int j=0;j<3;j++)
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
				for (int j=0;j<3;j++)
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
		return 0;
	}
	hduVector3Dd getForce()
	{
		return Fo;
	}
	hduVector3Dd getVel()
	{
		return Vo;
	}
	hduVector3Dd getPositiveEnergy()
	{
		return Ep;
	}

	~PoPc() {
		// TODO Auto-generated destructor stub
	}
};

int  phantomOpen();
void phantomCalibrate();
void phantomRun();
void phantomPause();
void phantomClose();
HDCallbackCode HDCALLBACK phantom_callback(void *pUserData);


const int DT_MAX=5000;
int DT=500;
int i,j;
static HHD Device;
static HDSchedulerHandle hPhantomMain;

PD pdX( 0.02 , 0.001, 0.001);
PD pdY( 0.02 , 0.001, 0.001);
PD pdZ( 0.02 , 0.001, 0.001);

hduVector3Dd VelSet = hduVector3Dd(0.0, 0.0, 0.0);
hduVector3Dd PositionSet = hduVector3Dd(0.0, 0.0, 0.0);


PoPc PoPcPosChannel;
PoPc PoPcForceChannel;


hduVector3Dd Position;
hduVector3Dd Force;

hduVector3Dd masterEnergyPos;


ros::Publisher slaveForcePub;
ros::Publisher slaveEnergyPub;
geometry_msgs::Pose slaveForce[DT_MAX];
geometry_msgs::Pose slaveEnergyForce[DT_MAX];

void SlaToMas_delay()
{
	//Makde delay
	for(i=0; i < DT-1; i++)
	{
		slaveForce[i] = slaveForce[i+1];
		slaveEnergyForce[i] = slaveEnergyForce[i+1];
	}

	slaveForce[DT-1].position.x = Force[0];
	slaveForce[DT-1].position.y = Force[1];
	slaveForce[DT-1].position.z = Force[2];

	hduVector3Dd slaveEnergyTemp = PoPcForceChannel.getPositiveEnergy();
	slaveEnergyForce[DT-1].position.x = slaveEnergyTemp[0];
	slaveEnergyForce[DT-1].position.y = slaveEnergyTemp[1];
	slaveEnergyForce[DT-1].position.z = slaveEnergyTemp[2];

	slaveForcePub.publish(slaveForce[0]);
	slaveEnergyPub.publish(slaveEnergyForce[0]);
}



void setPositionCallBack(const geometry_msgs::PosePtr msg)
{
	VelSet[0] = msg->position.x;
	VelSet[1] = msg->position.y;
	VelSet[2] = msg->position.z;
}
void masterEnergyCallBack(const geometry_msgs::PosePtr msg)
{
	masterEnergyPos[0] = msg->position.x;
	masterEnergyPos[1] = msg->position.y;
	masterEnergyPos[2] = msg->position.z;

}

void initData()
{
	PoPcPosChannel.Config(0,true); // PoPc in Position command channel - active, modify velocity
	PoPcForceChannel.Config(0,false); // PoPc in Force feedback command channel - nonactive
	for(i=0; i < DT-1; i++)
	{
		slaveForce[i].position.x = 0;
		slaveForce[i].position.y = 0;
		slaveForce[i].position.z = 0;
	}
}
int main(int argc, char ** argv)
{
	initData();
	//Init Phantom
	phantomOpen();
	printf("Phantom Opened \n");

	//Init ROS
	ros::init(argc, argv , "phantom_slave");

	ros::NodeHandle nh;

	slaveForcePub =  nh.advertise<geometry_msgs::Pose>("ForceFeedback",1);
	slaveEnergyPub =  nh.advertise<geometry_msgs::Pose>("SlaveEnergy",1);
	ros::Subscriber masterPose = nh.subscribe<geometry_msgs::PosePtr>("PositionCommand", 1, setPositionCallBack);
	ros::Subscriber masterEnergy = nh.subscribe<geometry_msgs::PosePtr>("MasterEnergy", 1, masterEnergyCallBack);

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
	hdBeginFrame(Device);
	hdGetDoublev(HD_CURRENT_POSITION,Position);

	PositionSet +=VelSet;

	Force[0] = pdX.compute(PositionSet[0] , Position[0]);
	Force[1] = pdY.compute(PositionSet[1] , Position[1]);
	Force[2] = pdZ.compute(PositionSet[2] , Position[2]);

	// Position Command Channel
	PoPcPosChannel.updatePoPc(Force,VelSet);
	PositionSet -= VelSet;
	if (PoPcPosChannel.PcControl(masterEnergyPos)==0)
	{
		//VelSet = PoPcPosChannel.getVel();
	}

	PositionSet += VelSet;

	Force[0] = pdX.compute(PositionSet[0] , Position[0]);
	Force[1] = pdY.compute(PositionSet[1] , Position[1]);
	Force[2] = pdZ.compute(PositionSet[2] , Position[2]);

	// Force feedback Command Channel
	PoPcForceChannel.updatePoPc(Force,VelSet);

	hdSetDoublev(HD_CURRENT_FORCE,Force);

	hdEndFrame(Device);
	SlaToMas_delay();
	return HD_CALLBACK_CONTINUE;
}
