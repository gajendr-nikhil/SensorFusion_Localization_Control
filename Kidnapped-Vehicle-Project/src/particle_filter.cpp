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

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 100;
	default_random_engine gen;
	normal_distribution<double> dist_x(gps_x, std[0]);
	normal_distribution<double> dist_y(gps_y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	for (int i = 0; i < num_particles; i++) {
		Particle p = {i, dist_x(gen), dist_y(gen), dist_theta(gen), 1.0};
		particles.push_back(p);
	}
	weights.resize(num_particles, 1.0f);
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	normal_distribution<double> dist_x(gps_x, std_pos[0]);
	normal_distribution<double> dist_y(gps_y, std_pos[1]);
	normal_distribution<double> dist_theta(theta, std_pos[2]);
	for (int i = 0; i < num_particles; i++) {
		if (fabs(yaw_rate) > 0.001) {
			particles[i].x += (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y += (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t
		}
		else {
			particles[i].x += velocity*delta_t*cos(particles[i].theta);
			particles[i].y += velocity*delta_t*sin(particles[i].theta);
		}
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(theta);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (int i = 0; i < observations.size(); i++) {
		int idx; // index of the closest predicted landmark
		double min_err = 99999999999.0;
		for (int j = 0; j < predicted.size(); j++) {
			double err = (((predicted[j].x - observations[i].x) * (predicted[j].x - observations[i].x)) +
						  ((predicted[j].y - observations[i].y) * (predicted[j].y - observations[i].y)));
			if (err < min_err) {
				idx = j;
				min_err = err;
			}
		}
		observations[i].id = idx;
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
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
	double covx = 2 * std_landmark[0] * std_landmark[0];
	double covy = 2 * std_landmark[1] * std_landmark[1];
	double denom = sqrt(2 * M_PI * std_landmark[0] * std_landmark[1]);

	// update weight of each particle
	for (int i = 0; i < num_particles; i++) {
		double px = particles[i].x;
		double py = particles[i].y;
		double ptheta = particles[i].theta;
		double prob = 1.0;

		// transform vehicle coordinates to map coordinates
		vector<LandmarkObs> map_obs;
		for (int j = 0; j < observations.size(); j++) {
			double x = observations[j].x;
			double y = observations[j].y;
			int id = observations[j].id;
			double tx = px + (x * cos(ptheta) - y * sin(ptheta));
			double ty = py + (x * sin(ptheta) + y * cos(ptheta));

			LandmarkObs obs = {id, tx, ty};
			map_obs.push_back(obs);
		}

		// landmarks within range
		vector<LandmarkObs> predicted_landmarks;
		for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
			double x = map_landmarks.landmark_list[j].x_f;
			double y = map_landmarks.landmark_list[j].y_f;
			int id = map_landmarks.landmark_list[j].id_i;
			double err = ((x - px) * (x - px)) + ((y - py) * (y - py));
			if (err < sensor_range) {
				LandmarkObs pred = {id, x, y};
				predicted_landmarks.push_back(pred);
			}
		}

		dataAssociation(predicted_landmarks, map_obs);

		for (int j = 0; j < map_obs.size(); i++) {
			double id = map_obs[j].id;
			double x = map_obs[j].x;
			double y = map_obs[j].y;
			double px = predicted_landmarks[id].x;
			double py = predicted_landmarks[id].y;
			double b = (((x - px) * (x - px)) / covx) + (((y - py) * (y - py)) / covy);
			prob *= (exp(-b) / denom);
		}
		particles[i].weight = prob;
		weights[i] = prob;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
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
