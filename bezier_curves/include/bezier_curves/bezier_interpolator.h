/// bezier_interpolator.h header file 
/// Zhiang Chen; Dec,2015
/// This class interpolates points with cubic Bezier curves.

#ifndef BEZIER_INTERPOLATOR_H_
#define BEZIER_INTERPOLATOR_H_

#include <math.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <baxter_traj_streamer/baxter_traj_streamer.h>

class Bezier_interpolator
{
public:
	Bezier_interpolator(int seg, double inter_t, double delta);
	Bezier_interpolator(double inter_t);
	bool Bezier_interpolator_solver(const trajectory_msgs::JointTrajectory new_trajectory, vector<Vectorq7x1> & qvecs, int & n_b);
	void set_calibration(){calibration=true;};
private:
	bool bezier_curves(const trajectory_msgs::JointTrajectory new_trajectory, vector<Vectorq7x1> &Ct, vector<Vectorq7x1> &Cp);
	void two_points_interpolate(vector<double> t,vector<double> positions,int n_control_points,vector<Vectorq7x1> &C_t, vector<Vectorq7x1> &C_p);
	void bezier_interpolation(const vector<Vectorq7x1> C_t, const vector<Vectorq7x1> C_p, vector<Vectorq7x1> & qvecs, const double inter_t);
	void find_control_points1(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double slope, int j);
	void find_control_points2(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double slope, int j);
	void find_control_points3(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, double slope1, double slope2, int j);
	void find_control_points4(vector<double> traj_p, vector<double> traj_t, vector<double> &C_p, vector<double> &C_t, int j);
	int n_seg;
	double inter_t;
	double delta;
	bool calibration;
};

#endif