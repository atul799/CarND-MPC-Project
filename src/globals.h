/*
 * globals.h
 *
 *
 *      Author: atpandey
 */

#ifndef SRC_GLOBALS_H_
#define SRC_GLOBALS_H_


/**************
 * GLOBALS
 *************/
const size_t N=10; //number of states to generate
const double dt=0.1; //time between state change evaluation

const double dt_latency=0.1;//simulator latency
//N/dt -->100 evaluations of state will be done by MPC per sim call
//N=10,dt=0.1 works fine

/* This value assumes the model presented in the classroom is used.
 *
 * It was obtained by measuring the radius formed by running the vehicle in the
 * simulator around in a circle with a constant steering angle and velocity on a
 * flat terrain.
 *
 * Lf was tuned until the the radius formed by the simulating the model
 * presented in the classroom matched the previous radius.
 *
 * This is the length from front to CoG that has a similar radius. */

const double Lf=2.67; //turning radius
const int nb_state_vars=6; //number of state variables
//px,py,psi,v,cte,epsi considered (add latency??)
const int nb_actuators=2; //steering angle and throttle
//throttle represents acceleration or braking (1,-1)

//target velocity, used as constraint
//const double ref_v = 40;
//const double ref_v = 100;
const double ref_v = 200;

//target cte, used as constraint
const double ref_cte=0;
// epsi, used as constraint
const double ref_epsi=0;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should establish
// when one variable starts and another ends to make our lifes easier.
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

//weights for cost
const double cte_w = 2000.0;
//const double cte_w = 1500.0;
//const double cte_w = 400.0;
//const double cte_w = 600.0;
//const double cte_w = 800.0;

//const double epsi_w = 2000.0;
//const double epsi_w = 1500.0;
//const double epsi_w = 100.0;
//const double epsi_w = 400;
//const double epsi_w = 400;
//const double epsi_w = 600;
const double epsi_w = 1000;


const double v_w = 0.2;
//const double v_w = 0.5;
//const double v_w = 0.1;

//const double delta_w = 5.0;
//const double delta_w = 10.0;
const double delta_w = 1.0;

//const double a_w = 5.0;
//const double a_w = 10.0;
const double a_w = 1.0;
//const double a_w = 0.1;

//const double delta_s_w = 200.0; // smooth angle change between subsequent steps
//const double delta_s_w = 150.0; // smooth angle change between subsequent steps
//const double delta_s_w = 10.0;
//const double delta_s_w = 5.0;
const double delta_s_w = 40.0;

//const double a_s_w = 10.0; // smooth acceleration change between subsequent steps
//const double a_s_w = 15.0; // smooth acceleration change between subsequent steps
//const double a_s_w = 0.01;
const double a_s_w = 2.0;

//const double cte_v_w=2.0; //penalize velocity for high cte error
//const double cte_v_w=1.0;
//const double cte_v_w=0.5;
const double cte_v_w=0.2;

const double delta_v_w=10.0; //penalize accceleration if vel is high

const int degree=3; //degree of polynomial to consider
//const int degree=4; //vehicle slower at turns
///////////////////////////////////////



#endif /* SRC_GLOBALS_H_ */
