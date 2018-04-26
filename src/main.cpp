#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "globals.h"
#include <fstream>


// for convenience
using json = nlohmann::json;



//double dt_latency=0.1;//simulator latency
//const double Lf=2.67; //turning radius
//const int nb_state_vars=6; //number of state variables
//int degree=3; //degree of polynomial to consider

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.rfind("}]");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
		int order) {
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (int i = 0; i < xvals.size(); i++) {
		A(i, 0) = 1.0;
	}

	for (int j = 0; j < xvals.size(); j++) {
		for (int i = 0; i < order; i++) {
			A(j, i + 1) = A(j, i) * xvals(j);
		}
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	return result;
}

int main() {
	uWS::Hub h;

	  //////////////////////////////////
	  // open a file handle to write x and y,heading,speed,steering and throttle
	  string out_file_name="../outputs/trackdata_fromMPC.csv";
	  ofstream out_file (out_file_name, ofstream::out);
	  if (!out_file.is_open())  {
		  cerr << "Cannot open output file: " << out_file_name << endl;
		  exit(EXIT_FAILURE);
	  }

	  out_file << "px" <<","<< "py" <<","<< "psi" <<","<<"v" <<","<< "steer_value" <<"," <<"throttle_value"<<endl;


	// MPC is initialized here!
	MPC mpc;

	h.onMessage([&mpc,&out_file](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
			uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		string sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
			string s = hasData(sdata);
			if (s != "") {
				auto j = json::parse(s);
				string event = j[0].get<string>();
				if (event == "telemetry") {

					/*****************************
					 *
					 * Current values from simulator
					 ******************************/
					// j[1] is the data JSON object
					vector<double> ptsx = j[1]["ptsx"];
					vector<double> ptsy = j[1]["ptsy"];
					double px = j[1]["x"];
					double py = j[1]["y"];
					double psi = j[1]["psi"];
					double v = j[1]["speed"];
					double steer_value = j[1]["steering_angle"];
					double throttle_value = j[1]["throttle"];


					out_file << px <<","<< py <<","<< psi <<","<<v <<","<< steer_value <<"," <<throttle_value<<endl;


					/********************************************
					 *
					 *
					 * Transform Waypoints to Vehicle Coords
					 ********************************************/
					/*
					 * : Calculate steering angle and throttle using MPC.
					 *
					 * Both are in between [-1, 1].
					 *
					 */

					//shift ptsx by px and then homogenous transform/rotate
					//this changes waypoints from map coords to cars coord

					//vector<double> waypoints_x;
					//vector<double> waypoints_y;
					Eigen::VectorXd waypoints_x(ptsx.size());
					Eigen::VectorXd waypoints_y(ptsy.size());
					for(unsigned int i=0;i<ptsx.size();i++){
						double shift_x=ptsx[i]-px;
						double shift_y=ptsy[i]-py;

						//ptsx[i]=shift_x*cos(0-psi)-shift_y*sin(0-psi);
						//ptsy[i]=shift_x*sin(0-psi)+shift_y*cos(0-psi);
						//waypoints_x.push_back(shift_x * cos(0-psi) - shift_y * sin(-psi));
						//waypoints_y.push_back(shift_x * sin(-psi) + shift_y * cos(-psi));
						waypoints_x[i]=shift_x * cos(0-psi) - shift_y * sin(0-psi);
						waypoints_y[i]=shift_x * sin(0-psi) + shift_y * cos(0-psi);
					}


					//convert from vector<doublke> to Eigen::VectorXd
					//double * ptrx=&waypoints_x[0];
					//Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx,6);
					//Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx,waypoints_x.size());


					//double * ptry=&waypoints_y[0];
					//Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry,6);
					//Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry,waypoints_y.size());

					//Fit Polynomial to get coeffs using waypoints
					//auto coeffs=polyfit(ptsx_transform,ptsy_transform,3);
					auto coeffs=polyfit(waypoints_x,waypoints_y,degree);

					/*************************************
					 *
					 * Calculate current cte and heading Error
					 ************************************/
					//calc cte and epsi, x is in car cords,so, px,py=0
					// current CTE is fitted polynomial (road curve) evaluated at px = 0.0
					//double cte =polyeval(coeffs, px) - py;
					double cte=polyeval(coeffs,0)-0;

					//heading error is arctan of f(polynomila function)
					//double epsi = psi - atan(coeffs[1]);
					//double epsi=psi-atan(coeffs[1]+2*px*coeffs[2]+3*coeffs[3]*pow(px,2));
					//px,py,psi=0
					double epsi=0-atan(coeffs[1]);

					//double Lf=2.67;

					/*************************************
					 * Add Latency
					 *************************************/
					//simulator has a delay of 100ms (0.1s) between actuator command
					//to response, add an estimate position based on
					//current velocity and heading to state.
					// current px,py and psi in vehicle coordinates:
					// 0.0, 0.0, 0.0
					// and vehicle coordinates have undergone homogenous transform so aliged to
					//waypoints (x-axis is vehicle direction, so x direction doesn't change)
					// The steering angle is negative the given value as we have
					// as recall that during transformation we rotated all waypoints by -psi

					//this won't work
					//double current_px = 0.0 + v * cos(psi) * dt_latency;
					//double current_py = 0.0 + v * sin(psi) * dt_latency;
					double current_px = 0.0 + v * dt_latency;
					double current_py = 0.0 ;
					double current_psi = 0.0 + v * (-steer_value) / Lf * dt_latency;
					double current_v = v + throttle_value * dt_latency;
					double current_cte = cte + v * sin(epsi) * dt_latency;
					double current_epsi = epsi + v * (-steer_value) / Lf * dt_latency;



					//declare state vector
					//Eigen::VectorXd state(6);
					Eigen::VectorXd state(nb_state_vars);
					//state << 0,0,0,v,cte,epsi;
					state << current_px, current_py, current_psi, current_v, current_cte, current_epsi;

					/******************************
					 * Run optimizer from mpc class
					 *****************************/
					auto vars=mpc.Solve(state,coeffs);


					json msgJson;
					// NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
					// Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
					//msgJson["steering_angle"] = vars[0]/(deg2rad(25)*Lf);
					msgJson["steering_angle"] = -vars[0]/(deg2rad(25)*Lf);
					msgJson["throttle"] = vars[1];

					/**************************************
					 *Prepare data for display in  simulator
					 **************************************/
					//Display the waypoints/reference line
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					//choose to represent 25 waypoints
					double poly_inc=2.5;
					int num_pts=25;
					//generate vector of y vals based on polyfit and poly_inc*i
					for(int i=1;i<num_pts;i++){
						next_x_vals.push_back(poly_inc*i);
						next_y_vals.push_back(polyeval(coeffs,poly_inc*i));
					}


					//Display the MPC predicted trajectory
					vector<double> mpc_x_vals;
					vector<double> mpc_y_vals;


					//first 2 of vars are steer and throttle, next are mpc predicted points
					for(unsigned int i=2;i<vars.size();i++){
						if(i%2==0) {
							mpc_x_vals.push_back(vars[i]);
						} else {
							mpc_y_vals.push_back(vars[i]);
						}
					}





					//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
					// the points in the simulator are connected by a Green line

					msgJson["mpc_x"] = mpc_x_vals;
					msgJson["mpc_y"] = mpc_y_vals;


					//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
					// the points in the simulator are connected by a Yellow line

					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;


					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					//debug
					//std::cout << msg << std::endl;

					// Latency
					// The purpose is to mimic real driving conditions where
					// the car does actuate the commands instantly.
					//
					// Feel free to play around with this value but should be to drive
					// around the track with 100ms latency.
					//
					// NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
					// SUBMITTING.
					//this_thread::sleep_for(chrono::milliseconds(100));

					//cout <<"Lat: "<<latency_t<<endl;
					this_thread::sleep_for(chrono::milliseconds(latency_t));

					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			} else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
			size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
			char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();

	//close the data capture file
	out_file.close();
}
