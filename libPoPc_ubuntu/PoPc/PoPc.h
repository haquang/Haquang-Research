/*
 * libPoPc.h
 *
 *  Created on: May 9, 2012
 *      Author: tam
 */

#ifndef POPC_H_
#define POPC_H_

class PoPc {
private:
	double* Fi;  	// Input Force
	double* Fo;		// Modified Force

	double* Vi;		// Input Velocity
	double* Vo;		// Modified Velocity

	double* Ep;		// Positive Energy
	double* En;		// Negative Energy

	bool type;		// Admittance or Impedance
	bool active;
	int Dof;
public:
	PoPc(bool typeSet,bool activePoPc,int Dof_set);

	void updatePoPc(double* FiSet,double* ViSet);
	bool PcControl(double* Ein);
	void getForce(double* F);
	void getVel(double* V);
	void getPositiveEnergy(double* E);
	void getNegativeEnergy(double* E);
	virtual ~PoPc();
};

#endif /* LIBPOPC_H_ */
