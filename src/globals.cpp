/*
 * globals.cpp
 *
 *      Author: atpandey
 */
#include <iostream>
#include "globals.h"

/**************
 * GLOBALS
 *************/
N=10; //number of states to generate
dt=0.1; //time between state change evaluation

dt_latency=0.1;//simulator latency
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

Lf=2.67; //turning radius
nb_state_vars=6; //number of state variables
//px,py,psi,v,cte,epsi considered (add latency??)
actuators=2; //steering angle and throttle
//throttle represents acceleration or braking (1,-1)

//target velocity, used as constraint
//double ref_v = 40;
ref_v = 100;
//ref_v = 200;

//target cte, used as constraint
ref_cte=0;
// epsi, used as constraint
ref_epsi=0;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should establish
// when one variable starts and another ends to make our lifes easier.
x_start = 0;
y_start = x_start + N;
psi_start = y_start + N;
v_start = psi_start + N;
cte_start = v_start + N;
epsi_start = cte_start + N;
delta_start = epsi_start + N;
a_start = delta_start + N - 1;

//weights for cost
cte_w = 2000.0;
epsi_w = 2000.0;
//cte_w = 1500.0;
//epsi_w = 1500.0;

v_w = 1.0;

delta_w = 5.0;
a_w = 5.0;
//delta_w = 10.0;
//a_w = 10.0;


delta_s_w = 200.0; // smooth angle change between subsequent steps
a_s_w = 10.0; // smooth acceleration change between subsequent steps
//delta_s_w = 150.0; // smooth angle change between subsequent steps
//a_s_w = 15.0; // smooth acceleration change between subsequent steps

degree=3; //degree of polynomial to consider
///////////////////////////////////////
