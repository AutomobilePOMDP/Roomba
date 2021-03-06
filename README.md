# AA228FinalProject
An example video showing the robot first localizing itself using its bump sensors, then navigating safely to the goal. The Roomba's belief about where it may be located is represented by the blue regions, and is updated using a particle filter.
<img src="demo.gif" width="450">

## Installation
```
add https://github.com/LAMDA-POMDP/Roomba
```

## Getting Started
Run the ```lidar_roomba.ipynb``` jupyter notebook to become familiar with the Roomba environment. This will walk you through a step-by-step example of how to set up the environment, define a baseline policy, and evaluate the performance of the policy.

Next, familiarize yourself with the source code by examining the files in the ```src``` directory. A brief description of the files is given below:
* ```Roomba.jl``` - defines the package module for this project and includes the necessary import and export statements
* ```roomba_env.jl``` - defines the environment as a POMDPs.jl MDP and POMDP
* ```env_room.jl``` - defines the environment room and rectangles used to define it
* ```line_segment_utils.jl``` - functions for determining whether the Roomba's path interects with a line segment and struct defining line segments
* ```filtering.jl``` - specification of particle filters for the bumper and lidar Roomba environments

## Usage
```julia
# Action Space
max_speed = 2.0
speed_interval = 2.0
max_turn_rate = 1.0
turn_rate_interval = 1.0
action_space = vec([RoombaAct(v, om) for v in 0:speed_interval:max_speed, om in -max_turn_rate:turn_rate_interval:max_turn_rate])

# Bumper Roomba
sensor = Bumper()
pomdp = RoombaPOMDP(sensor=sensor, mdp=RoombaMDP(aspace=action_space));

# Lidar Roomba
sensor = Lidar()
pomdp = RoombaPOMDP(sensor=sensor, mdp=RoombaMDP(config=2, aspace=action_space))

# Discrete Lidar Roomba
cut_points =  exp10.(range(-.5, stop=1.3, length=10))
sensor = DiscreteLidar(cut_points)
pomdp = RoombaPOMDP(sensor=sensor, mdp=RoombaMDP(config=2, aspace=action_space))

# Belief updater
num_particles = 5000 # number of particles in belief
pos_noise_coeff = 0.3 # noise of position in the resampler
ori_noise_coeff = 0.1 # noise of orientation in the resampler

# LidarResampler for lidar sensor
resampler = LidarResampler(num_particles, pomdp, pos_noise_coeff, ori_noise_coeff)
# BumperResampler for bumper sensor
# resampler = BumperResampler(num_particles, pomdp, pos_noise_coeff, ori_noise_coeff)

belief_updater = BasicParticleFilter(pomdp, resampler, num_particles)
```

## Adding additional packages or running files from terminal
If you would like to add additional packages or run files from terminal, please follow the procedure below.
In julia, first enter the package manager by hitting the ```]``` key. Then activate the AA228FinalProject environment by following the instructions in the Installation section. Packages you now add will be added to the AA228FinalProject environment, and if you exit the package manager (by pressing backspace), the code you run while the environment is activated will have access to all packages specified in the ```Project.toml``` file.

## Parameters
The parameters of Roomba are listed as follows.
```julia
maximum velocity of Roomba [m/s]
v_max::Float64  = 10.0  # m/s

maximum turn-rate of Roombda [rad/s]
om_max::Float64 = 1.0   # rad/s

simulation time-step [s]
dt::Float64     = 0.5   # s

penalty for wall-contact
contact_pen::Float64 = -1.0 

penalty per time-step
time_pen::Float64 = -0.1

reward for reaching goal
goal_reward::Float64 = 10

penalty for reaching stairs
stairs_penalty::Float64 = -10

specifies room configuration (location of stairs/goal) {1,2,3}
config::Int = 1

environment room struct
room::Room  = Room(sspace,configuration=config)

environment state-space (ContinuousRoombaStateSpace or DiscreteRoombaStateSpace)
sspace::SS = ContinuousRoombaStateSpace()

environment action-space struct
aspace::AS = RoombaActions()
```
