#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int limit(int current_wp, const vector<double> map_waypoints_x)
{
	//double current_wp = map_waypoints_x[0];
	int start = current_wp;
	for (int i = start; i < map_waypoints_x.size(); i++)
	{
		if (current_wp > map_waypoints_x[i])
		{
			return current_wp;
		}
		current_wp = map_waypoints_x[i];
	}
	return -1;
}

double getDfromLane(int lane)
{
	return 2+(4*lane);
}

int getLanefromD(double d)
{
	if (d < 4) { return 0; }
	if (d < 8) { return 1; }
	if (d < 12) { return 2; }
	return -1;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

	// Test out map loaded correctly and spline works correctly
	cout << "Waypoint counts, x, y, dx, dy, s" << endl;
	cout << map_waypoints_x.size() << " " << map_waypoints_y.size() << " " << map_waypoints_dx.size() 
	<< " " << map_waypoints_dy.size() << " " << map_waypoints_s.size() << endl;
	cout << "First 5 waypoints:" << endl;
	for (int i = 0; i < 5; i++)
	{
		cout << "x: " << map_waypoints_x[i] << ", y: " << map_waypoints_y[i] << ", s: " << map_waypoints_s[i] << ", dx: " << map_waypoints_dx[i] << ", dy: " << map_waypoints_dy[i] << endl;
	}
	tk::spline s;
	vector<double> wp_x(map_waypoints_x.begin(),map_waypoints_x.begin()+10);
	vector<double> wp_y(map_waypoints_y.begin(),map_waypoints_y.begin()+10);
	s.set_points(wp_x, wp_y);    // requires that X is already sorted
	double x=800;
	printf("Spline test at %f is %f\n", x, s(x));

	h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
							(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) 
	{
	// "42" at the start of the message means there's a websocket message event.
	// The 4 signifies a websocket message
	// The 2 signifies a websocket event
	//auto sdata = string(data).substr(0, length);
	//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2') 
		{
			auto s = hasData(data);

			if (s != "") 
			{
				auto j = json::parse(s);
				
				string event = j[0].get<string>();
				
				if (event == "telemetry") 
				{
					// j[1] is the data JSON object
					
					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];
					// why can't we just have SI units?
					car_speed = 0.44704 * car_speed;
					// cout << " car speed " << car_speed << endl;

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values 
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];
					// 0 = car's unique ID,
					// 1 = car's x position in map coordinates,
					// 2 = car's y position in map coordinates,
					// 3 = car's x velocity in m/s,
					// 4 = car's y velocity in m/s,
					// 5 = car's s position in frenet coordinates, 
					// 6 = car's d position in frenet coordinates

					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;

					// TODO starts here

					double accel_max = 10; // m/s^2+
					double jerk_max = 50; // m/s^3
					double speed_max = 21; // m/s
					int previous_path_size = previous_path_x.size();
					double target_speed = speed_max; // m/s
					double wp_time_gap = 0.02;
					int waypoints_max = 100;
					double horizon = 20;
					double predict_time = wp_time_gap*previous_path_size;
					double wp_spacing_max = speed_max*wp_time_gap;
					double wp_spacing = wp_spacing_max;
					double delta_wp = 0;
					double wp_spacing_final = wp_spacing_max;

					// reference
					double ref_x = car_x;
					double ref_y = car_y;
					double ref_yaw = deg2rad(car_yaw);
					vector<double> ref_sd = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
					double ref_s = ref_sd[0];
					double ref_d = ref_sd[1];
					double current_lane = getLanefromD(car_d);
					double target_lane = current_lane;

					// ordered by lane, then by close/far
					vector<vector<int>> remotes_lanes;
					for (int i=0; i<3; i++)
					{
						vector<int> lanes = {0, 0};
						remotes_lanes.push_back(lanes);
					}
					bool change_lanes = false;
					for (int i = 0; i< sensor_fusion.size(); i++)
					{
						int ID = sensor_fusion[i][0]; // 0 = car's unique ID,
						double x = sensor_fusion[i][1]; // 1 = car's x position in map coordinates,
						double y = sensor_fusion[i][2]; // 2 = car's y position in map coordinates,
						double vx = sensor_fusion[i][3]; // 3 = car's x velocity in m/s,
						double vy = sensor_fusion[i][4]; // 4 = car's y velocity in m/s,
						double rs = sensor_fusion[i][5]; // 5 = car's s position in frenet coordinates,
						double rd = sensor_fusion[i][6]; // 6 = car's d position in frenet coordinates
						int r_lane = getLanefromD(rd);
						double r_vel = sqrt(vx*vx+vy*vy);
						rs = rs+predict_time*r_vel;
						double my_s = car_s + car_speed*predict_time;
						double s_diff = rs - my_s;
						// If the car will be close in the future, we need to take action
						// if close, ahead, same lane
						if ( (s_diff < 20) && (s_diff > 0) && (r_lane == current_lane) )
						{
							change_lanes = true;
							target_speed = r_vel;
						}
						// count the cars that would prevent lane change
						if (0 == r_lane && (s_diff < 20) && (s_diff > -10))
						{
							remotes_lanes[0][0]++;
						}
						if (0 == r_lane && (s_diff < 50) && (s_diff > 20))
						{
							remotes_lanes[0][1]++;
						}
						if (1 == r_lane && (s_diff < 20) && (s_diff > -10))
						{
							remotes_lanes[1][0]++;
						}
						if (1 == r_lane && (s_diff < 50) && (s_diff > 20))
						{
							remotes_lanes[1][1]++;
						}
						if (2 == r_lane && (s_diff < 20) && (s_diff > -10))
						{
							remotes_lanes[2][0]++;
						}
						if (2 == r_lane && (s_diff < 50) && (s_diff > 20))
						{
							remotes_lanes[2][1]++;
						}
					}
					// cout << "Intersting Cars in lanes:" << endl;
					// cout << remotes_lane0.size() << " "
					// 		 << remotes_lane1.size() << " "
					// 		 << remotes_lane2.size() << " " << endl;

					// This is basically a state machine...
					if (change_lanes)
					{
						if (0 == current_lane && remotes_lanes[1][0] == 0)
						{
							target_lane = 1;
						}
						if (2 == current_lane && remotes_lanes[1][0] == 0)
						{
							target_lane = 1;
						}
						if (1 == current_lane)
						{
							vector<int> costs = {(10 * remotes_lanes[0][0] + remotes_lanes[0][1]),
																			9,
																			10 * remotes_lanes[2][0] + remotes_lanes[2][1]};
							cout << "costs: " << costs[0] << " " << costs[1] << " " << costs[2] << endl;
							double cost = 999;
							target_lane = 1;
							for (int i = 0; i<costs.size(); i++)
							{
								if (costs[i]<cost)
								{
									cost = costs[i];
									target_lane = i;
								}
							}
							cout << car_s << " chose target_lane " << target_lane << endl;
						}						
					}
					
					// points to interpolate with spline
					vector<double> ptsx;
					vector<double> ptsy;

					// if there is no previous path use current pos
					if (previous_path_size < 2)
					{
						double previous_car_x = car_x - cos(car_yaw);
						double previous_car_y = car_y - sin(car_yaw);
						ptsx.push_back(previous_car_x);
						ptsy.push_back(previous_car_y);
						ptsx.push_back(car_x);
						ptsy.push_back(car_y);
					}
					else // start at end of previous path
					{
						ref_x = previous_path_x[previous_path_size-1];
						ref_y = previous_path_y[previous_path_size-1];
						double ref_x_prev = previous_path_x[previous_path_size-2];
						double ref_y_prev = previous_path_y[previous_path_size-2];
						ref_yaw = atan2(ref_y-ref_y_prev, ref_x - ref_x_prev);
						ref_sd = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
						ref_s = ref_sd[0];
						ref_d = ref_sd[1];
						ptsx.push_back(ref_x_prev);
						ptsy.push_back(ref_y_prev);
						ptsx.push_back(ref_x);
						ptsy.push_back(ref_y);
					}

					//make sure first added spine point is ahead of previous path
					vector<double> next_sparse_wp = getXY(ref_s + 40, getDfromLane(target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
					ptsx.push_back(next_sparse_wp[0]);
					ptsy.push_back(next_sparse_wp[1]);
					for (int i = 1; i < 3; i++)
					{
						vector<double> next_sparse_wp = getXY(ref_s + 40 + 10 * i, getDfromLane(target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
						ptsx.push_back(next_sparse_wp[0]);
						ptsy.push_back(next_sparse_wp[1]);
					}

					// transorm to local coords
					// cout << "Points for splining:" << endl;
					for (int i = 0; i< ptsx.size(); i++)
					{
						// coord vector relative to reference point
						double shift_x = ptsx[i] - ref_x;
						double shift_y = ptsy[i] - ref_y;
						// rotate coord vector so x axis is direction of motion
						ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
						ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
						// cout << ptsx[i] << " " << ptsy[i] << endl;
					}

					vector<double> ppathx;
					vector<double> ppathy;
					for (int i = 0; i< previous_path_size; i++)
					{
						// coord vector relative to reference point
						double shift_x = (double)previous_path_x[i] - ref_x;
						double shift_y = (double)previous_path_y[i] - ref_y;
						// rotate coord vector so x axis is direction of motion
						ppathx.push_back(shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
						ppathy.push_back(shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
						// cout << ptsx[i] << " " << ptsy[i] << endl;
					}

					// make the spline in local coords
					// doing this avoids x[i]>x[i+1], as long as spline isn't too long
					tk::spline s;
					s.set_points(ptsx, ptsy);

					// current path starts with remainder of previous path
					for (int i = 0; i< previous_path_size; i++)
					{
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					double target_x = 5.0;
					double target_y = s(target_x);
					// distance from current pos to final point
					double target_dist = sqrt((pow(target_x,2)+pow(target_y,2)));
					while (target_dist<horizon)
					{
						target_x++;
						target_y = s(target_x);
						target_dist = sqrt((pow(target_x,2)+pow(target_y,2)));
					}
					while (target_dist>horizon)
					{
						target_x--;
						target_y = s(target_x);
						target_dist = sqrt((pow(target_x,2)+pow(target_y,2)));
					}

					double ppath_x_dist = 0;

					if (previous_path_size > 1)
					{
						double ppath_x_end = ppathx[previous_path_size-1];
						double ppath_x_start = ppathx[0];
						ppath_x_dist = ppath_x_end - ppath_x_start;
						//cout << "p x dist " << ppath_x_dist << endl;
					}

					// s = d/t
					double last_speed = car_speed;
					double change;
					double delta_speed;
					if (previous_path_size > 1)
					{
						double last_speed_x = ppathx[previous_path_size-1] - ppathx[previous_path_size-2];
						double last_speed_y = ppathy[previous_path_size-1] - ppathy[previous_path_size-2];
						last_speed = sqrt(last_speed_x*last_speed_x+last_speed_y*last_speed_y)/wp_time_gap;
					}
					if (abs(target_speed - last_speed)/wp_time_gap < accel_max*0.75)
					{
						wp_spacing = target_speed * wp_time_gap;
						//cout << " got to match speed " << endl;
					}
					else
					{
						change = (target_speed - last_speed)/abs(target_speed - last_speed);
						delta_speed = change*accel_max*0.75*wp_time_gap;
						delta_wp = delta_speed * wp_time_gap;
						wp_spacing_final = target_speed * wp_time_gap;
						wp_spacing = last_speed * wp_time_gap;
					}
					// cout << "target_speed wp_spacing change delta_speed" << endl;
					// cout << target_speed << " " << wp_spacing << " " << change << " " << delta_speed << endl;

					double x_add_on = 0;
					int new_wps = 0;

					// add on points to finish off the current path
					while (x_add_on + ppath_x_dist < target_x &&
								 previous_path_size + new_wps < waypoints_max)
					//for (int i = 1; i <= num_waypoints-previous_path_size; i++)
					{
						if (wp_spacing + delta_wp < wp_spacing_final)
						{
							wp_spacing = wp_spacing + delta_wp;
						}
						if (wp_spacing + delta_wp > wp_spacing_final)
						{
							wp_spacing = wp_spacing + delta_wp;
						}
						double N = (target_dist/(wp_spacing));
						double x_point = x_add_on+(target_x/N);
						double y_point = s(x_point);

						x_add_on = x_point;

						double x_ref = x_point;
						double y_ref = y_point;

						x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
						y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

						x_point += ref_x;
						y_point += ref_y;

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);
						new_wps++;

					}

					// Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

					// int previous_wp = ClosestWaypoint(car_x, car_y, map_waypoints_x, map_waypoints_y);
					// int next_wp = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
					// if (previous_wp == next_wp)
					// {
					// 	previous_wp = previous_wp - 1;
					// }
					// double dist_inc = 0.4;
					// for (int i = 0; i < 50; i++)
					// {
					// 	double next_s = car_s + (i+1) * dist_inc;
					// 	double next_d = car_d;
					// 	vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
					// 	next_x_vals.push_back(xy[0]);
					// 	next_y_vals.push_back(xy[1]);
					// }

					// END
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
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
}
