# CIS_4900_Ind_Study_Comp_Sci - Autonomous Goal-Kicking Robot

Objective: To design a Reinforcment Learning Framework which would enable a robot to accurately kick a ball towards a goal within a webots appliction


## Setting up Webots (The Environment)
Make sure to download webots using this link [https://cyberbotics.com/]

### Setting up Conda Environment 
Download the latest annaconda application using this link [https://www.anaconda.com/download]. Create and activate the conda enviroment named "deepbots" which runs python 3.8. Deepbots is not able to run 3.9 or higher 
 - conda create - n deepbots python=3.8
 - conda activate deepbots

We need to install an older version of pip, so pip can be compatible to install deepbots. Once you activate the conda environment, input this:
 - python -m pip install pip==21.3.1 setuptools==59.5.0 wheel
 - pip install deepbots

### Connecting the Conda Environment with Webots
Open a webots world file will use system python and not the deepbots conda environment. We need to set Webots, so it runs python under conda environment. Activate the conda environment and type **"where python"**. This will show the python file which contains the Deepbot download.  
 - conda activate deepbots --> where python --> insert files to webots preferences python path. 

Finally type "pip install torch" to download the pytorch as the main RL framework of the program. 

### Opening the Webots simulator
There will be projects under the "Learning Webots", "ReinforcmentLearning_Soccer", and "CartPole" directories. In either of the 3 directories, open the "worlds" folder. 
Open any of the .wbt files or any icon with a ladybug


## Navigation and Project Duration
The Final report will be under this [link](https://docs.google.com/document/d/10s23bLTjrOAsplGaDYIm2D-zqwJvWRKvnmxXFjn8EJI/edit?usp=sharing). I have outlined the scope of each project along with the specific milestones achieved during my tenure
#### CartPole

#### Learning Webots

#### Reinforcement Learning Soccer



## Credits


## What's next
After 2 years I will be returning to this project under a new repository "Autonomous-Goal-Kicking-Robot". I will be discarding using Deepbots as the primary Reinforcmenet Learning Framework. I will be focusing my research to apply reinforcment learning on inverse kinematics. 
["New Repository"] (https://github.com/TLe1289/Autonomous-Goal-Kicking-Robot)

