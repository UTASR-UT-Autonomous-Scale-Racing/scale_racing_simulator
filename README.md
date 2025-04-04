# Will's Scale Racing Simulator
This project was made for the scale racing team at the University of Toronto. The club aims to build a small autonomous car, similar to the F10th competitions. A part of developing the software/autonomy side of the project is selecting and testing reinforcement learning algorithms for controlling the vehicle. Training RL policies in real time on physical hardware is difficult and time consuming. The aim of this simulator is to give a platform for experimenting with policies and learning more about RL strategies that might be useful for the physical car. 

## Tools:
- PyBullet: acts as the physics simulator and rendering environment for this simulated race track. 
- Gymnasium (similar to OpenAI Gym) provides a uniform inferface for RL tools.

## Setup:
I recommend using a python package manager such as Micromamba or conda to create an isolated environment. 

### Gymnasium
```
pip install gymnasium
```

to install the custom package containing the gym environment, you need to run:  
```bash
pip install -e gym_envs
```
This -e uses editable mode, which allows you to modify the package without having to call this every time you make a change. 

### Pybullet
you must [download pybullet](https://github.com/bulletphysics/bullet3) 

### Math tools

#### numpy

### Geometry tools

#### alphashape
Used for finding contours of shapes
#### scipy
This is used for interpolation along paths. 

### Plotting tools

#### matplotlib

# Structure

- gym-envs contains the gym environment definition  
    - construct_environment.py provides the code for setting up the world and spawning in the track.
    - racing_env.py contains the majority of the environment definition, such as the step function etc.
- the urdf files contain robot definitions for the cars. see [the source](https://gist.github.com/GerardMaggiolino/723302b55498cc7c4e508e1abd15daea).  
- manual_control.py allows you to drive the car manually  
- train.py allows you to run a policy. Currently uses proportional controller logic. 
- tracks contains the object files for the tracks. Note that there must be an inner and outer track provided. 



