/*
 * libPoPc.cpp
 *
 *  Created on: May 9, 2012
 *      Author: tam
 */

#include "PoPc.h"

PoPc::PoPc(bool typeSet,bool activePoPc,int Dof_set) {
	// TODO Auto-generated constructor stub
	type = typeSet;
	active = activePoPc;
	Dof =Dof_set;
	Fi = new double(Dof);
	Fo = new double(Dof);

	Vi = new double(Dof);
	Vo = new double(Dof);

	Ep = new double(Dof);
	En = new double(Dof);

	for (int j=0;j<Dof;j++)
	{
		Fi[j] = 0.0;
		Fo[j] = 0.0;

		Vi[j] = 0.0;
		Vo[j] = 0.0;

		Ep[j] = 0.0;
		En[j] = 0.0;
	}
}

PoPc::~PoPc() {
	// TODO Auto-generated destructor stub
	delete Fi;
	delete Fo;

	delete Vi;
	delete Vo;

	delete Ep;
	delete En;
}
void PoPc::updatePoPc(double* FiSet,double* ViSet)
{
	for (int j=0;j<Dof;j++)
	{
		Fi[j] = FiSet[j];
		Vi[j] = ViSet[j];

		if (Fi[j]*Vi[j]>0)
		{
			if (active) // calculate negative energy
				En[j] -= Fi[j]*Vi[j];
			else		// calculate positive energy
				Ep[j] += Fi[j]*Vi[j];
		}
	}

}

bool PoPc::PcControl(double* Ein)
{
	if (!active)
	{
		return false;
	}
	else
	{
		if (type) //Modify Velocity
		{
			for (int j=0;j<Dof;j++)
			{
				if (Ein[j] + En[j] <0)
				{
					En[j] += Fi[j]*Vi[j];  //backward 1 step
					if (Fi[j]!=0)
						Vo[j] = (Ein[j] + En[j])/Fi[j];
					else
						Vo[j] = 0;
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

			for (int j=0;j<Dof;j++)
			{
				if (Ein[j] + En[j] <0)
				{
					En[j] += Fi[j]*Vi[j];  //backward 1 step
					if (Vi[j]!=0)
						Fo[j] = (Ein[j] + En[j])/Vi[j];
					else
						Fo[j] = 0;
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
void PoPc::getForce(double* F)
{
	for (int j=0;j<Dof;j++)
		F[j] = Fo[j];
}
void PoPc::getVel(double* V)
{
	for (int j=0;j<Dof;j++)
		V[j] = Vo[j];
}
void PoPc::getPositiveEnergy(double* E)
{
	for (int j=0;j<Dof;j++)
		E[j] = Ep[j];
}

void PoPc::getNegativeEnergy(double* E)
{
	for (int j=0;j<Dof;j++)
		E[j] = En[j];
}
