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

#define EPS 0.000001

using namespace std;

/*************************************************************************************************/
/*************************************************************************************************/
void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from GPS) and all weights to 1.
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    // define number of particles
    num_particles = 75;
    // set size for particles vector & weights vector
    particles.resize(num_particles);
    // generate random particles representing position and orientation
    std::default_random_engine gen;
    // create normalized Gaussian distribution for position and orientation
    std::normal_distribution<double> dist_x(x, std[0]);
    std::normal_distribution<double> dist_y(y, std[1]);
    std::normal_distribution<double> dist_theta(theta, std[2]);
    // initialize using random numbers generated earlier
    for (auto& p:particles) {
      p.x = dist_x(gen);
      p.y = dist_y(gen);
      p.theta = dist_theta(gen);
      p.weight = 1.0;
    }

    // set initialization flag
    is_initialized = true;
}

/*************************************************************************************************/
/*************************************************************************************************/
void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    // generate random noise using random engine generator
    default_random_engine gen;
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);

    // estimate new position using bicycle model
    for (auto& p: particles) {
        if (fabs(yaw_rate)<=EPS) {
            // when yaw-rate is zero
            p.x += velocity * delta_t * cos(p.theta);
            p.y += velocity * delta_t * sin(p.theta);
            //particles[i].theta does not change
        }
        else {
            // when yaw-rate is non-zero
            p.x += (velocity / yaw_rate) * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
            p.y += (velocity / yaw_rate) * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
            p.theta += yaw_rate * delta_t;
        }

        // add noise to estimate
        p.x += dist_x(gen);
        p.y += dist_y(gen);
        p.theta += dist_theta(gen);
    }
}

/*************************************************************************************************/
/*************************************************************************************************/
void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

    // loop over number of observations
    for (auto& obs: observations) {
      // initialize minimum distance parameter
      double min_dist = numeric_limits<float>::max();

      // loop over predicted states
      for (auto& pr: predicted) {
        // compute distance
        double dist_val = dist(obs.x, obs.y, pr.x, pr.y);
        // check condition
        if (dist_val < min_dist) {
          min_dist = dist_val;
          // update observation ID to nearest neighbor's ID in predicted set
          obs.id = pr.id;
        }
      }
    }
}

/*************************************************************************************************/
/*************************************************************************************************/
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

  // clear weights
  weights.clear();

  // define parameters one-time for later use
  double x_norm = 2 * std_landmark[0] * std_landmark[0];
  double y_norm = 2 * std_landmark[1] * std_landmark[1];
  double xy_norm = 2 * M_PI * std_landmark[0] * std_landmark[1];

  // loop over each particle
  for (auto& p: particles) {
    // save map landmark points as vector
    // discard locations outside sensor range of the particle
    vector<LandmarkObs> ref_landmark;
    // compare distance between particle and landmark
    for (auto& mark: map_landmarks.landmark_list) {
      double dist_val = dist(p.x, p.y, mark.x_f, mark.y_f);
      if (dist_val < sensor_range) {
        ref_landmark.push_back(LandmarkObs{mark.id_i, mark.x_f, mark.y_f});
      }
    }

    // transform all observation points from vehicle to map coordinate system
    double cT = cos(p.theta), sT = sin(p.theta);
    vector<LandmarkObs> obs_map;
    LandmarkObs obs_new;
    for (auto& obs: observations) {
        // apply transformation
        obs_new.id = obs.id;
        obs_new.x = obs.x*cT - obs.y*sT + p.x;
        obs_new.y = obs.x*sT + obs.y*cT + p.y;
        obs_map.push_back(obs_new);
    }

    // data association - assign landmark index for each observation (now in map-coordinates)
    dataAssociation(ref_landmark, obs_map);

    // assign and update weights using multi-variate Gaussian distribution
    p.weight = 1.0;
    for (unsigned int j=0; j<obs_map.size(); j++) {
        Map::single_landmark_s landmark = map_landmarks.landmark_list.at(obs_map[j].id-1);

        double xW = pow((obs_map[j].x - landmark.x_f), 2) / x_norm;
        double yW = pow((obs_map[j].y - landmark.y_f), 2) / y_norm;
        double W = exp(-(xW + yW)) / xy_norm;

        p.weight *= W;
    }
    weights.push_back(p.weight);
  }
}

/*************************************************************************************************/
/*************************************************************************************************/
void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    // new particle list vector
    vector<Particle> resampledP (num_particles);
    // generate discrete distribution to get particles by weight
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> dist(weights.begin(), weights.end());
    // resample particles according to weight
    for (int i = 0; i < num_particles; i++) {
        resampledP[i] = particles[dist(gen)];
    }

    // assign new weights
    particles = resampledP;
    // clearing weights, per recommendation on forum
    //weights.clear();
}

/*************************************************************************************************/
/*************************************************************************************************/
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

/*************************************************************************************************/
/*************************************************************************************************/
string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
/*************************************************************************************************/
/*************************************************************************************************/
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
/*************************************************************************************************/
/*************************************************************************************************/
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
