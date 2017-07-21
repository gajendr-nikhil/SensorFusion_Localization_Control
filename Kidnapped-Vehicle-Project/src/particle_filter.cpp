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
	num_particles = 100;
	default_random_engine gen;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	for (int i = 0; i < num_particles; i++) {
		Particle p = {i, dist_x(gen), dist_y(gen), dist_theta(gen), 1.0};
		particles.push_back(p);
	}
	weights.resize(num_particles, 1.0);
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	default_random_engine gen;
	normal_distribution<double> dist_x(0.0, std_pos[0]);
	normal_distribution<double> dist_y(0.0, std_pos[1]);
	normal_distribution<double> dist_theta(0.0, std_pos[2]);
	for (int i = 0; i < num_particles; i++) {
		if (fabs(yaw_rate) > 0.001) {
			particles[i].x += (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			particles[i].y += (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			particles[i].theta += yaw_rate * delta_t;
		}
		else {
			particles[i].x += velocity*delta_t*cos(particles[i].theta);
			particles[i].y += velocity*delta_t*sin(particles[i].theta);
		}
		particles[i].x += dist_x(gen);
		particles[i].y += dist_y(gen);
		particles[i].theta += dist_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
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

	double covx = 2 * std_landmark[0] * std_landmark[0];
	double covy = 2 * std_landmark[1] * std_landmark[1];
	double denom = sqrt(2 * M_PI * std_landmark[0] * std_landmark[1]);

	// update weight of each particle
	for (int i = 0; i < num_particles; i++) {
		double px = particles[i].x;
		double py = particles[i].y;
		double ptheta = particles[i].theta;

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
			double err = sqrt(((x - px) * (x - px)) + ((y - py) * (y - py)));
			if (err < sensor_range) {
				LandmarkObs pred = {id, x, y};
				predicted_landmarks.push_back(pred);
			}
		}

		dataAssociation(predicted_landmarks, map_obs);

		double prob = 1.0;
		for (int j = 0; j < map_obs.size(); j++){
			int id = map_obs[j].id;
			double x = map_obs[j].x;
			double y = map_obs[j].y;
			double predx = predicted_landmarks[id].x;
			double predy = predicted_landmarks[id].y;
			double dx = x - predx;
			double dy = y - predy;
			double b = ((dx * dx) / covx) + ((dy * dy) / covy);
			prob *= (exp(-b) / denom);
		}
		particles[i].weight = prob;
		weights[i] = prob;
	}
}

void ParticleFilter::resample() {
	vector<Particle> newp (num_particles);
	default_random_engine gen;
	discrete_distribution<int> idx(weights.begin(), weights.end());
	for (int i = 0; i < num_particles; i++)
		newp[i] = particles[idx(gen)];
	particles = newp;
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
