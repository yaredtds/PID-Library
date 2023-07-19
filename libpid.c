/*
 * @version: 3.0
 * @author : Yared Tadesse
 *
 * @date : 2:21 PM Saturday, June-29,2021 GC
 *
 * @date : 6:08 PM Saturday, Aug-24,2019 GC
 *	   Kekenu 12:08, Kidame, Nehase 18,2011 EC
 *         Addis Ababa ,Ethiopia
 * @reference:
 * 1. PID controller-Wikipedia, the free encyclopedia 
 *       https://en.wikipedia.org/wiki/PID_controller ..07/26/2016 02:06PM
 * 2. Karl Johan Astrom and Richard M. Murray ,
 *      "Feedback systems : an introduction for scientists and engineers 2009,
 *       Princeton University Press,Version v2.10
 * 3. I.Kar,"Digital Control,Module 1:Introduction to Digital Control Lecture Note
 * 4. Xin-lan Li,Jong-Gyu Park,& Hwi-Beom Shin,"Comparison and Evaluation of
 * 	    Anti-Windup PI Controllers",Journal of Power Electronics, Vol.11, No.1,
 * 		January 2011
 */

#include <math.h>
#include <float.h>
#include <stdint.h>
#include "libpid.h" 

//#include "Arduino.h" // Uncomment this line if Arduino IDE is used for the project

/* change this with time function of mcu ide */
#include <time.h>
#include <stdio.h>
/* required to access the process time in micro-second */
clock_t clock(void);
/* local function prototype */

double proportional(pidc_t *pidp, double P, double b);
double derivative(pidc_t *pidp, double K, double Td, double Ts, double N, double c, char method);
double integral(pidc_t *pidp, double K, double Ti, double Tt, double Ts, char method);
double input_filter(pidc_t *pidp, double set_point);
double actuator_limit(pidc_t *pidp,double ckp);

void initialize(pidc_t *pidp,double _ckm1, double _ekm1)
{
	pidp->ckm1=_ckm1;
	pidp->eskm1=_ekm1;
	pidp->ekm1=_ekm1;
	pidp->esk = actuator_limit(pidp, 0.0);
}

void set_feedback(pidc_t *pidp, double value)
{
	pidp->feedback = value;
}

double get_feedback(pidc_t *pidp)
{
	return pidp->feedback;
}

/*PID Control Algorithm Source Code*/
void set_setpoint(pidc_t *pidp,double set_point)
{
	pidp->r = input_filter(pidp, set_point);
}

double get_setpoint(pidc_t *pidp)
{
	return pidp->r;
}

void set_input_range(pidc_t *pidp,double minima,double maxima)
{
	pidp->min_input = minima;
	pidp->max_input = maxima;
}

void set_actuator_limit(pidc_t *pidp,double minima,double maxima)
{
	pidp->min_output = minima;
	pidp->max_output = maxima;
}

double input_filter(pidc_t *pidp, double set_point)
{
	if (set_point > pidp->max_input){
		return pidp->max_input; 
	}else if(set_point < pidp->min_input){
		return pidp->min_input;
	}else{
		return  set_point;
	}
}

double actuator_limit(pidc_t *pidp,double ckp)
{
	if( ckp > pidp->max_output){
		/* return Negative error */
		return  (pidp->max_output - ckp);
	}else if (ckp < pidp->min_output){
		/* return Positive error */
		return (pidp->min_output - ckp);
	}else{
		return 0;
	}
}

double proportional(pidc_t *pidp, double P, double b)
{
	double epk = pidp->ek + (b - 1)*pidp->r;
	return	P * epk;
}

double derivative(pidc_t *pidp, double K, double Td, double Ts, double N, double 
c, char method)
{
	double cdk = 0.0;
	double edk = pidp->ek + (c - 1)*pidp->r;
	double derr = edk - pidp->ekm1;
	double dn = K * Td * N;
	double nts = N*Ts;

	switch (method)
	{
	case(Forward):
		cdk = (1- nts)* pidp->ckm1 + dn*derr;
		break;			
	case(Backward):
		cdk = (Td*pidp->ckm1 + dn*derr)/(Td + nts);
		break;
	case(Trapezoidal):
		cdk = ( (Td - (nts*0.5))*pidp->ckm1 + dn*derr )/(Td + nts*0.5);
		break;
	};
	
	return cdk;			 
}

double integral(pidc_t *pidp, double K, double Ti, double Tt, double Ts, char 
method)
{
	double cik = 0;
	double eik = pidp->ek;		

	switch(method)
	{
	case(Forward):
		cik = pidp->ckm1 + Ts*((pidp->ekm1*K)/Ti  +  pidp->eskm1/Tt);
		break;
	case(Backward):
		cik = pidp->ckm1 + Ts*((eik*K)/Ti  +  pidp->esk/Tt);
		break;
	case(Trapezoidal):
		cik = pidp->ckm1 + 0.5*Ts*(K*(eik + pidp->ekm1)/Ti - (pidp->esk + pidp->eskm1)/Tt);
		break;
	};

	return cik;
}


/* Main controller algorithm */
double pid(pidc_t *pidp, double Kp, double Ki, double Kd, double N, double 
b,double c, double Ts, char integral_type, char filter_type)
{
	double Ti = Kp/Ki;
	double Td = Kd/Kp;
	double Tt = sqrt(fabs(Ti*Td));;
	/* Update Tt if and only if PI controller*/
	if(Kd == 0){
		Tt = Ti;
	}

	pidp->ek = pidp->r - get_feedback(pidp);

	double p_term = proportional(pidp, Kp, b);  
	double d_term = derivative(pidp, Kp, Td, Ts, N, c, filter_type);
	double i_term = integral(pidp, Kp, Ti, Tt, Ts, integral_type); 
	double ck = p_term + i_term + d_term;//

	/* updating controller parameters*/
	pidp->eskm1 = pidp->esk;	
	pidp->esk = actuator_limit(pidp, ck);
	pidp->ekm1 = pidp->ek;
	pidp->ckm1 = pidp->ck;
	pidp->ck = ck + pidp->esk;

	return pidp->ck;		
}

