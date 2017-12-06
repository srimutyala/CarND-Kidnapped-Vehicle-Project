/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

#define EPS 0.00001

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	if (!is_initialized)
	{
		num_particles = 100;
		double std_x = std[0];
		double std_y = std[1];
		double std_theta = std[2];

		normal_distribution<double> nd_x(x, std_x);
		normal_distribution<double> nd_y(y, std_y);
		normal_distribution<double> nd_theta(theta, std_theta);
		default_random_engine generator;

		Particle p;
		for (int i = 0; i < num_particles; i++)
		{
			p.id = i;
			p.x = nd_x(generator);
			p.y = nd_y(generator);
			p.theta = nd_theta(generator);
			p.weight = 1.0;
			particles.push_back(p);
		}
		is_initialized = true;
	}

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	double std_x = std_pos[0];
	double std_y = std_pos[1];
	double std_theta = std_pos[2];

	normal_distribution<double> nd_x(0, std_x);
	normal_distribution<double> nd_y(0, std_y);
	normal_distribution<double> nd_theta(0, std_theta);
	default_random_engine generator;

	for (int i = 0; i < num_particles; i++)
	{
		double theta = particles[i].theta;	
		//Different when  yaw rate is zero 
		if (fabs(yaw_rate) < EPS) { 
			particles[i].x += velocity * delta_t * cos(theta);
			particles[i].y += velocity * delta_t * sin(theta);
		}
		else {
			particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
			particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}
		particles[i].x += nd_x(generator);
		particles[i].y += nd_y(generator);
		particles[i].theta += nd_theta(generator);

	}

}

void ParticleFilter::data_Association(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	//*
	unsigned int num_Observations = observations.size();
	unsigned int num_Predictions = predicted.size();

	for (unsigned int i = 0; i < num_Observations; i++)
	{
		double min_Distance = numeric_limits<double>::max();
		int mapId = -1;
		for (unsigned j = 0; j < num_Predictions; j++)
		{
			double x_Distance = observations[i].x - predicted[j].x;
			double y_Distance = observations[i].y - predicted[j].y;
			double distance = x_Distance * x_Distance + y_Distance * y_Distance;

			if (distance < min_Distance)
			{
				min_Distance = distance;
				mapId = predicted[j].id;
			}
		}
		observations[i].id = mapId;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html


	double x;
	double y;
	double theta;
	double stdLandmarkRange = std_landmark[0];
	double stdLandmarkBearing = std_landmark[1];
	for (int i = 0; i < num_particles; i++)
	{
		x = particles[i].x;
		y = particles[i].y;
		theta = particles[i].theta;
		double sen_range_square = sensor_range * sensor_range;
		vector<LandmarkObs> obs_Landmarks;
		int lm_id;
		float lm_x;
		float lm_y;		
		for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
		{
			lm_id = map_landmarks.landmark_list[j].id_i;
			lm_x = map_landmarks.landmark_list[j].x_f;
			lm_y = map_landmarks.landmark_list[j].y_f;
			
			double dist_x = x - lm_x;
			double dist_y = y - lm_y;
			if (dist_x*dist_x + dist_y*dist_y <= sen_range_square) {
				obs_Landmarks.push_back(LandmarkObs{ lm_id, lm_x, lm_y });
			}
		}
		vector<LandmarkObs> map_Observations;
		for (int j = 0; j < observations.size(); j++)
		{
			double xm = x + cos(theta)*observations[j].x - sin(theta)*observations[j].y;
			double ym = y + sin(theta)*observations[j].x + cos(theta)*observations[j].y;
			map_Observations.push_back(LandmarkObs{ observations[j].id, xm, ym });
		}

		data_Association(obs_Landmarks, map_Observations);

		particles[i].weight = 1.0;
		for (int j = 0; j < map_Observations.size(); j++)
		{
			double x_Observation = map_Observations[j].x;
			double y_Observation = map_Observations[j].y;
			int landmarkId = map_Observations[j].id;

			double x_Landmark, y_Landmark;
			int k = 0; //*
			int n_Landmarks = obs_Landmarks.size();
			bool found = false;
			while (!found && k < n_Landmarks) {
				if (obs_Landmarks[k].id == landmarkId) {
					found = true;
					x_Landmark = obs_Landmarks[k].x;
					y_Landmark = obs_Landmarks[k].y;
				}
				k++;
			}

			double dist_X = x_Observation - x_Landmark;
			double dist_y = y_Observation - y_Landmark;

			double weight = (1 / (2 * M_PI*stdLandmarkRange*stdLandmarkBearing)) * exp(-(dist_X*dist_X / (2 * stdLandmarkRange*stdLandmarkRange) + (dist_y*dist_y / (2 * stdLandmarkBearing*stdLandmarkBearing))));
			if (weight == 0) {
				particles[i].weight *= EPS;
			}
			else {
				particles[i].weight *= weight;
			}
		}

	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<double> weights;
	double maxWeight = numeric_limits<double>::min();
	for (int i = 0; i < num_particles; i++) {
		weights.push_back(particles[i].weight);
		if (particles[i].weight > maxWeight) {
			maxWeight = particles[i].weight;
		}
	}

	uniform_real_distribution<double> distDouble(0.0, maxWeight);
	uniform_int_distribution<int> distInt(0, num_particles - 1);
	default_random_engine generator;

	int index = distInt(generator);
	double beta = 0.0;

	vector<Particle> resampledParticles;
	for (int i = 0; i < num_particles; i++) {
		beta += distDouble(generator) * 2.0;
		while (beta > weights[index]) {
			beta -= weights[index];
			index = (index + 1) % num_particles;
		}
		resampledParticles.push_back(particles[index]);
	}
	particles = resampledParticles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
