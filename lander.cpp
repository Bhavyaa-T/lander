// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <iostream>
#include <fstream>
#include <vector>

void autopilot(void)
// Autopilot to adjust the engine throttle, parachute and attitude control
{
    // Tunable parameters
    const double K_h = 0.015;    
    const double K_p = 1;       
    const double delta = 0.05;

    // Get altitude and vertical speed
    double altitude = position.abs() - MARS_RADIUS;
    vector3d e_r = position.norm(); // Radial unit vector
    double vertical_speed = velocity * e_r; // Radial component of velocity

    // Error signal (target is soft landing, so negative vertical speed)
    double error = -(0.5 + K_h * altitude + vertical_speed);

    // Proportional control
    double P = K_p * error;

    // Throttle control logic
    if (P < -delta) {
        throttle = 0.0;
    }
    else if (P > 1.0 - delta) {
        throttle = 1.0;
    }
    else {
        throttle = delta + P;
    }


    if (throttle < 0.0) throttle = 0.0;
    if (throttle > 1.0) throttle = 1.0;


    static vector<double> h_list, v_list, t_list;
    static double t = 0.0;

    t_list.push_back(t);
    h_list.push_back(altitude);
    v_list.push_back(vertical_speed);

    t += delta_t;


    ofstream fout("trajectories.txt");
    if (fout) {
        for (size_t i = 0; i < t_list.size(); ++i) {
            fout << t_list[i] << ' ' << h_list[i] << ' ' << v_list[i] << std::endl;
        }
    }
}
void numerical_dynamics(void)
{
    static bool first_step = true;


    vector3d thr = thrust_wrt_world();
    double d = atmospheric_density(position);

    vector3d F_d;
    if (parachute_status == DEPLOYED) {
        F_d = -0.5 * d * velocity.abs() * velocity * (1 * 3.14159 * LANDER_SIZE * LANDER_SIZE + 2 * 5 * 2 * 2 * LANDER_SIZE * LANDER_SIZE);
    }
    else {
        F_d = -0.5 * d * velocity.abs() * velocity * (3.14159 * LANDER_SIZE * LANDER_SIZE);
    }

    double LANDER_MASS = UNLOADED_LANDER_MASS + fuel * FUEL_DENSITY * FUEL_CAPACITY;
    double r = position.abs();

    // acceleration at time n
    vector3d a = -((GRAVITY * MARS_MASS) / (r * r * r)) * position
        + F_d / LANDER_MASS + thr / LANDER_MASS;

    // Verlet integrator 

    
    vector3d new_position = position + velocity * delta_t + 0.5 * a * delta_t * delta_t;

    
    thr = thrust_wrt_world();
    d = atmospheric_density(new_position);

    if (parachute_status == DEPLOYED) {
        F_d = -0.5 * d * velocity.abs() * velocity * (1 * 3.14159 * LANDER_SIZE * LANDER_SIZE + 2 * 5 * 2 * 2 * LANDER_SIZE * LANDER_SIZE);
    }
    else {
        F_d = -0.5 * d * velocity.abs() * velocity * (3.14159 * LANDER_SIZE * LANDER_SIZE);
    }

    r = new_position.abs();
    vector3d a_new = -((GRAVITY * MARS_MASS) / (r * r * r)) * new_position
        + F_d / LANDER_MASS + thr / LANDER_MASS;


    vector3d new_velocity = velocity + 0.5 * (a + a_new) * delta_t;

    // commit updates
    position = new_position;
    velocity = new_velocity;

    
    if (autopilot_enabled) autopilot();
    if (stabilized_attitude) attitude_stabilization();
}


void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
