//  ---------------------- Doxygen info ----------------------
//! \file FastResearchInterfaceTest.h
//!
//! \brief
//! Header file for some functions of FastResearchInterfaceTest application
//!
//! \date December 2014
//!
//! \version 1.2
//!
//!	\author Torsten Kroeger, tkr@stanford.edu\n
//! \n
//! Stanford University\n
//! Department of Computer Science\n
//! Artificial Intelligence Laboratory\n
//! Gates Computer Science Building 1A\n
//! 353 Serra Mall\n
//! Stanford, CA 94305-9010\n
//! USA\n
//! \n
//! http://cs.stanford.edu/groups/manips\n
//! \n
//! \n
//! \copyright Copyright 2014 Stanford University\n
//! \n
//! Licensed under the Apache License, Version 2.0 (the "License");\n
//! you may not use this file except in compliance with the License.\n
//! You may obtain a copy of the License at\n
//! \n
//! http://www.apache.org/licenses/LICENSE-2.0\n
//! \n
//! Unless required by applicable law or agreed to in writing, software\n
//! distributed under the License is distributed on an "AS IS" BASIS,\n
//! WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n
//! See the License for the specific language governing permissions and\n
//! limitations under the License.\n
//!
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#ifndef __FastResearchInterfaceTest__
#define __FastResearchInterfaceTest__

#include <armadillo>


void RunTrajectory						(FastResearchInterface	*FRI);
void HapticsDemo							(FastResearchInterface	*FRI);
void ImpedanceDemo						(FastResearchInterface	*FRI);
void MoveToCandle							(FastResearchInterface	*FRI);
void Cart1DDemo							(FastResearchInterface	*FRI);
void LineCompliance						(FastResearchInterface	*FRI);
void RunTrajectorySimple					(FastResearchInterface	*FRI);
void StateSpaceSwitching					(FastResearchInterface	*FRI);
void SensorGuidedSwitching				(FastResearchInterface	*FRI);
void TransOTG								(FastResearchInterface	*FRI);
void ICRAOTG								(FastResearchInterface	*FRI);
void Anya									(FastResearchInterface	*FRI);
void KinectJointSpace						(FastResearchInterface	*FRI);
void KinectCartSpace						(FastResearchInterface	*FRI);
void StateSpaceSwitchingWithCompliance	(FastResearchInterface	*FRI);
void StanfordTrackingCartSpace			(FastResearchInterface	*FRI);
void StanfordTrackingJointSpace			(FastResearchInterface	*FRI);
void JediBot								(FastResearchInterface	*FRI);
void RobertsDemo							(FastResearchInterface	*FRI);
void SweepTest								(FastResearchInterface	*FRI);
void SweepTestPosCtrl						(FastResearchInterface	*FRI);

void MoveToCandleWithImpendance			(FastResearchInterface	*FRI);
void DoSquareWithCartImpendance			(	FastResearchInterface *FRI,
												float side,					// square side (m)
												double maxVelocity,			// m/sec
												double maxAcceleration,		// m/sec^2
												float* stiffness,			// N/m
												float* damping,				// Nsec/m
												char* loggingName=NULL
											);
void DoCircleWithCartImpendance			(	FastResearchInterface *FRI,
												double radius,				// radius (m)
												double velocity,			// linear velocity (m/sec)
												float* stiffness,			// N/m
												float* damping,				// Nsec/m
												char* loggingName=NULL
											);
void GotoJointReferencePosition			(	FastResearchInterface *FRI,
												float* refpose				// joint space
											);

void PPRC									(	FastResearchInterface *FRI,
												float* EndPosition,
												float Rho0,
												float RhoInf,
												float T,
												char* loggingName);

void PPRC_1Joint							(	FastResearchInterface *FRI,
												float EndPosition,
												float RhoInf,
												float T,
												char* loggingName,
												float K,
												float Kappa);

void PPC_1joint_torqu					(	FastResearchInterface *FRI,
												float EndPosition,
												float RhoInf,
												float T,
												float Rho_s0,
												float Rho_sInf,
												float T_s,
												float K,
												float Ks);

void PPC_1joint_velEst(	FastResearchInterface *FRI,
					float k1,
					float k2,
					float setpoint,
					int contSelect,
					int estSelect);

void PPC_impact							(	FastResearchInterface *FRI,
												float* EndPosition,
												float T,
												float l,
												float Rho_Inf,
												float ki,
												float ks,
												float Kv,
												float Ki);

void PPRC_vopen							(	FastResearchInterface *FRI,
												float* EndPosition,
												float RhoInf,
												char* loggingName,
												float Gain);

void PPRC_vclosed							(	FastResearchInterface *FRI,
												float* EndPosition,
												float RhoInf,
												char* loggingName,
												float Gain);

//Handshake version 1: Adaptation of amplitude only.
void handshake_v1							(	FastResearchInterface *FRI,
												float* EndPosition,
												float RhoInf,
												char* loggingName,
												float Gain);

//Handshake version 2: First try for ellipse fitting.
void handshake_v2							(	FastResearchInterface *FRI,
												float* EndPosition,
												float RhoInf,
												char* loggingName,
												float Gain);

void handshake_v3	(	FastResearchInterface *FRI,
					float* EndPosition,
					float RhoInf,
					char* loggingName,
					float Gain);

void flacco_jLimits	(	FastResearchInterface *FRI,
							float* EndPosition,
							float RhoInf,
							char* loggingName,
							float Gain);

void PPRC_vopen_jLimits	(	FastResearchInterface *FRI,
							float* EndPosition,
							float RhoInf,
							char* loggingName,
							float Gain);

void simple_1Joint	(	FastResearchInterface *FRI,
							float EndPosition,
							float Rho0,
							float RhoInf,
							float T,
							char* loggingName,
							float K);
void simple_1Joint_2	(	FastResearchInterface *FRI,
					float EndPosition,
					char* loggingName,
					float K);
void getMeas	(	FastResearchInterface *FRI );

void getMeas2	(	FastResearchInterface *FRI );

void CartesianVel	(	FastResearchInterface *FRI,
					float uCart,
					char* loggingName);

void CDC2015_exp(	FastResearchInterface *FRI,
					float k1,
					float k2,
					float setpoint,
					int mode);


void PPC_1joint_torqu_withEst(	FastResearchInterface *FRI,
					float EndPosition,
					float RhoInf,
					float T,
					float Rho_s0,
					float Rho_sInf,
					float T_s,
					float K,
					float Ks,
					int mode);

void Leonidas_Roll_Kinematic(	FastResearchInterface *FRI);
void Leonidas_Roll_Kinematic_2(	FastResearchInterface *FRI);
void Leonidas_Roll_Kinematic_3(	FastResearchInterface *FRI);
void Leonidas_Roll_Kinematic_4(	FastResearchInterface *FRI);
void Leonidas_Roll_dynamic(	FastResearchInterface *FRI);

void est_HoganFunction(FastResearchInterface *FRI,  float step);

void PPC_PnV(	FastResearchInterface *FRI, float KqAmpl, float KyAmpl);
void torqueLevel_JLA	(	FastResearchInterface *FRI, float* EndPosition , int nullEnable);
void flacco_jLimits	(	FastResearchInterface *FRI, float* EndPosition);
void PPRC_vopen_jLimits	(	FastResearchInterface *FRI, float* EndPosition, float RhoInf, float Gain);
void RunTrajectorySimple_cos(FastResearchInterface *FRI);
void est_HoganFunction_With_Lm(FastResearchInterface *FRI,  float step);

void handshake_v3	(	FastResearchInterface *FRI, float* EndPosition, float RhoInf, char* loggingName, float Gain);
int main_dora(FastResearchInterface *FRI);
int leadThrough(FastResearchInterface *FRI);
void giannis_impedance	(	FastResearchInterface *FRI);
void grav_comp_z(	FastResearchInterface *FRI);
int variableImpedance(FastResearchInterface *FRI);
int codeVariableAdmittance(FastResearchInterface *FRI);
int mainVariableAdmittance(FastResearchInterface *FRI);
void fullarm_rob_exp(	FastResearchInterface *FRI, int mode);

void Leonidas_Roll_Kinematic_5(	FastResearchInterface *FRI);
double main_minimaze(double *pp, double sigma);
int aiding(	FastResearchInterface *FRI);

int  dissipative(	FastResearchInterface *FRI);
int aiding_spacial(	FastResearchInterface *FRI);
int dissipative_adm (	FastResearchInterface *FRI);
int dissipativeR(	FastResearchInterface *FRI);
int dissipativeRT(	FastResearchInterface *FRI);
int dissipativeRTpaper3(	FastResearchInterface *FRI);
int bettiniAiding(	FastResearchInterface *FRI);

int dissipativeRTwithDQ(	FastResearchInterface *FRI);
int aiding_spacial_OT(	FastResearchInterface *FRI);
int aiding_spacial_SERAFun(	FastResearchInterface *FRI);
int aiding_spacial_SERAFun2(	FastResearchInterface *FRI);


void MoveToPointWithCartImpendance(	FastResearchInterface *FRI,
										double totalTime,
										arma::vec pd,
										arma::mat Rd);

void gravity_comp	(	FastResearchInterface *FRI);


enum RPYSolution
{
	RPYSolution1	=	1,
	RPYSolution2	=	2
};


void CalculateRPYAnglesFromFrame(const double Frame[12], double RPYAnglesInDegrees[3], const RPYSolution Solution);
void CalculateRPYAnglesFromFrame(const float Frame[12], float RPYAnglesInDegrees[3], const RPYSolution Solution);

void CalculateFrameFromRPYAngles(const double RPYAnglesInDegrees[3], double Frame[12]);
void CalculateFrameFromRPYAngles(const float RPYAnglesInDegrees[3], float Frame[12]);

double atan2_New(const double b, const double a);


double getDistanceFromPlane(arma::vec p);


inline void PrintFrame(float *Frame)
{
	unsigned int		i		=	0
					,	j		=	0;

	for ( i = 0; i < 3; i++)
	{
		printf("( ");
		for ( j = 0; j < 4; j++)
		{
			printf("   %8.3f   ", Frame[4 * i + j]);
		}
		printf(" )\n");
	}
	printf("(       0.000         0.000         0.000         1.000    )\n\n");
}


inline void PrintFrame(double *Frame)
{
	unsigned int		i		=	0
					,	j		=	0;

	for ( i = 0; i < 3; i++)
	{
		printf("( ");
		for ( j = 0; j < 4; j++)
		{
			printf("   %8.3f   ", Frame[4 * i + j]);
		}
		printf(" )\n");
	}
	printf("(       0.000         0.000         0.000         1.000    )\n\n");
}

inline void PrintFrame(double Frame[3][3])
{
	unsigned int		i		=	0
					,	j		=	0;

	float				Values[12];

	for ( i = 0; i < 3; i++)
	{
		for ( j = 0; j < 3; j++)
		{
			Values[4 * i + j]	=	Frame[i][j];
		}
	}

	PrintFrame(Values);
}




#endif
