#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
	auto foundNull = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("]");
	if (foundNull != std::string::npos) {
		return "";
	} else if (b1 != std::string::npos && b2 != std::string::npos) {
		return s.substr(b1, b2 - b1 + 1);
	}
	return "";
}

int main(int argc, char* argv[]) {

	const auto usage =
			"Usage: FusionEKF <sensor>\n   where <sensor> is \"radar\" or \"lidar\" to only use measurements from that sensor,\n   or left out to use measurements from both sensors.";

	auto useLIDAR { true };  // Set to false to ignore LIDAR measurements
	auto useRADAR { true };  // Set to false to ignore RADAR measurements

	if (argc > 1) {
		string param {argv[1]};
		if (argc == 2 and param == "radar") {
			useLIDAR = false;
			cout << "Using measurements from RADAR only" << endl;
		}
		else if (argc == 2 and param == "lidar") {
			useRADAR = false;
			cout << "Using measurements from LIDAR only" << endl;
		}

		else
			cout << usage << endl << "Arguments ignored" << endl;
	}

	uWS::Hub h;

	// Create a Kalman Filter instance
	FusionEKF fusionEKF;

	// used to compute the RMSE later
	vector<VectorXd> estimations;
	vector<VectorXd> groundTruth;

	auto step { 0 }; // Progressive counter of computation steps, increased after processing of every measurement, useful for debugging

	h.onMessage(
			[&step, useRADAR, useLIDAR, &fusionEKF,&estimations,&groundTruth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
				// "42" at the start of the message means there's a websocket message event.
				// The 4 signifies a websocket message
				// The 2 signifies a websocket event

				if (length && length > 2 && data[0] == '4' && data[1] == '2')
				{

					auto s = hasData(std::string(data));
					if (s != "") {

						auto j = json::parse(s);

						std::string event = j[0].get<std::string>();

						if (event == "telemetry") {
							string sensorMeasurement = j[1]["sensor_measurement"];

							MeasurementPackage measPackage;
							istringstream iss(sensorMeasurement);
							long long timestamp;

							// reads first element from the current line
							string sensorType;
							iss >> sensorType;

							if (sensorType.compare("L") == 0) {
								measPackage.sensorType = MeasurementPackage::LASER;
								measPackage.rawMeasurements = VectorXd(2);
								float px;
								float py;
								iss >> px;
								iss >> py;
								measPackage.rawMeasurements << px, py;
								iss >> timestamp;
								measPackage.timestamp = timestamp;
							} else if (sensorType.compare("R") == 0) {
								measPackage.sensorType = MeasurementPackage::RADAR;
								measPackage.rawMeasurements = VectorXd(3);
								float rho;
								float theta;
								float rhoDot;
								iss >> rho;
								iss >> theta;
								iss >> rhoDot;
								measPackage.rawMeasurements << rho,theta, rhoDot;
								iss >> timestamp;
								measPackage.timestamp = timestamp;
							}
							float x_gt;
							float y_gt;
							float vx_gt;
							float vy_gt;
							iss >> x_gt;
							iss >> y_gt;
							iss >> vx_gt;
							iss >> vy_gt;
							VectorXd gt_values(4);
							gt_values(0) = x_gt;
							gt_values(1) = y_gt;
							gt_values(2) = vx_gt;
							gt_values(3) = vy_gt;

							// Feed measurements to an Extended Kalman Filter (EKF) for processing
							bool useMeasure = (sensorType.compare("R") == 0 && useRADAR)||(sensorType.compare("L") == 0 && useLIDAR);
							if (useMeasure) {
								groundTruth.push_back(gt_values); // This will be used below to calculate the RMSE, i.e. goodness of the estimates
								fusionEKF.processMeasurement(measPackage);
							}

							// Append to `estimations` the current estimated state, as per the EKF.
							auto x = fusionEKF.getState();
							VectorXd RMSE;
							if (useMeasure)
								estimations.push_back(x);

							json msgJson;
							if (estimations.size() >0) {
								RMSE= calculateRMSE(estimations, groundTruth);
								msgJson["estimate_x"] = x(0);
								msgJson["estimate_y"] = x(1);
								msgJson["rmse_x"] = RMSE(0);
								msgJson["rmse_y"] = RMSE(1);
								msgJson["rmse_vx"] = RMSE(2);
								msgJson["rmse_vy"] = RMSE(3);
							}
							/* If no estimate is available, then all measurements processed so far are of the kind
							 * (LIDAR or RADAR) that should be skipped; the simulator is still waiting for a message
							 * with estimates and RMSE, so send a message with dummy values (otherwise the simulator
							 * will wait indefinitely, and not send the next measurements)
							 */
							else {
								msgJson["estimate_x"] = 0;
								msgJson["estimate_y"] = 0;
								msgJson["rmse_x"] = 0;
								msgJson["rmse_y"] = 0;
								msgJson["rmse_vx"] = 0;
								msgJson["rmse_vy"] = 0;
							}
							auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
							++step;
							ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

						}
					} else {
						std::string msg = "42[\"manual\",{}]";
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}
				}

			});

	// We don't need this since we're not using HTTP but if it's removed the program
	// doesn't compile :-(
	h.onHttpRequest(
			[](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
				const std::string s = "<h1>Hello world!</h1>";
				if (req.getUrl().valueLength == 1)
				{
					res->end(s.data(), s.length());
				}
				else
				{
					// i guess this should be done more gracefully?
					res->end(nullptr, 0);
				}
			});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection(
			[&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
}

