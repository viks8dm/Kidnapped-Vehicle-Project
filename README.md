# Kidnapped Vehicle Project

In this project-3 of term-2 of the self driving car nanodegree program by Udacity, a 2D Particle Filter is used for localization of a car placed in an unknown location, using reference landmarks around the car. For satisfactory project completion, the requirement is that the (i) particle filter localize the vehicle to within the required accuracy, (ii) complete one run within the specified time of 100 seconds. (further details are present in the [project ruberic](https://review.udacity.com/#!/rubrics/747/view).

The project has been created using Udacity's [starter Code](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project)

## Implementation outline

The implementation consists of following steps:

* `initialization`: The location of car is initialized using noisy GPS data.
* `prediction`: new location of vehicle after every tiem step is predicted using a simple bicycle model, with known previous vehicle location/orientation information along with velocity and yaw-rate.
* `weight-update`: in this step the measured data from sensors is used to update the assigned weights of the particles in the filter, under the assumption that those measurements were made by the particle itself. In this process, (a) the sensor measurement is transformed form vehicle to map coordinate system, (b) measurements areassciated with actual landmarks on the map, (c) calculated probability that the measurements were made by the particle itself.
* `resampling`: here, particles are re-sampled to retain more of those with higher probability, in order to increase the accuracy of the overall system.

All these steps are implemented in the script in `src/particle_filter.cpp`.

## To run the code

Clone this repository and enter following commands in a terminal

`./clean.sh`

`./build.sh`

`./run.sh`

After execution of `./run.sh`, [simulator](https://github.com/udacity/self-driving-car-sim/releases/) should be opened and `Project 3: kidnapped-vehicle` option should be scrolled to. On pressing the start button one will see a car running between the landmarks and a blue annular region trying to track the car.


## Results & Discussion

[image1]: ./results/intermediateScreen.jpg "screenshot at random time step"
[image2]: ./results/finalScreen.jpg "final result"
[image3]: ./results/sampleWeights.jpg "sample weights summary"

The particle-filter implementation for this project performes as expected. It meets the requirements, as is illustrated in the final screenshot below, which displays "Success! Your particle filter passed!".

--

![alt text][image2]
--

For my implementation I am using 75 particles. I experimented with 100 and 50 particles as well and successfully met the requirements. Due to use of random numbers, the runtime is different with each simulation, but I did not encounter any failure. A video for one of the sample runs can be found in the results folder [here](./results/KidnappedVehicleVideo_75_particles.mov).

The tracking is a little jittery, which I assume is because of use of linear model, size of time step and the noise introduced per requirements. There was no significant change in this jittery behavior with increase/decrease in number of particles (I did not try extremely high or low number though).

During each run, the main.cpp script also displays the highest and average weights on the screen. A sample output with these numbers is shown here for illustration purposes only

--

![alt text][image3]
--


