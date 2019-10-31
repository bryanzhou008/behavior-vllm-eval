#  Interactive Gibson Environment
Large Scale Virtualized Interactive Environment for Learning Robot Manipulation and Navigation

Interactive Gibson is a fast simulator and a dataset for indoor navigation and manipulation. It was first released in June 2019. It allows for complicated interactions between the agent and the environment, such as picking up and placing objects, or opening doors and cabinets. This environment opens up new venues for jointly training base and arm policies, allowing researchers to explore the synergy between manipulation and navigation.


#### Paper
If you use Interactive Gibson Simulator or Interactive Gibson assets, please consider citing the following paper:

```
@techreport{xiagibson2019,
           title = {Gibson Env V2: Embodied Simulation Environments for Interactive Navigation},
           author = {Xia, Fei and Li, Chengshu and Chen, Kevin and Shen, William B and Mart{\'i}n-Mart{\'i}n, Roberto and Hirose, Noriaki and Zamir, Amir R and Fei-Fei, Li and Savarese, Silvio},
           group = {Stanford Vision and Learning Group},
           year = {2019},
           institution = {Stanford University},
           month = {6},
}
```


Release
=================
**This is the gibson2 0.0.1 release. Bug reports, suggestions for improvement, as well as community developments are encouraged and appreciated.** [change log file](misc/CHANGELOG.md).  

**Support for [Gibson v1](http://github.com/StanfordVL/GibsonEnv/) will be moved to this repo**.

Database
=================
The full database includes 572 spaces and 1440 floors and can be downloaded [here](gibson/data/README.md). A diverse set of visualizations of all spaces in Gibson can be seen [here](http://gibsonenv.stanford.edu/database/). To make the core assets download package lighter for the users, we  include a small subset (39) of the spaces. Users can download the rest of the spaces and add them to the assets folder. We also integrated [Stanford 2D3DS](http://3dsemantics.stanford.edu/) and [Matterport 3D](https://niessner.github.io/Matterport/) as separate datasets if one wishes to use Gibson's simulator with those datasets (access [here](gibson/data/README.md)).

Table of contents
=================

   * [Installation](#installation)
   * [Quick Start](#quick-start)
        * [Tests](#tests)
        * [Gibson FPS](#gibson-framerate)
        * [Rendering Semantics](#rendering-semantics)
        * [Robotic Agents](#robotic-agents)
        * [ROS Configuration](#ros-configuration)
   * [Coding your RL agent](#coding-your-rl-agent)
   * [Environment Configuration](#environment-configuration)
   * [Goggles: transferring the agent to real-world](#goggles-transferring-the-agent-to-real-world)
   * [Citation](#citation)

Installation
=================

#### Installation Method

Gibson v2 can be installed as a python package:

```bash
git clone https://github.com/fxia22/gibsonv2 --recursive
cd gibsonv2

conda create -n py3-gibson python=3.6 anaconda
source activate py3-gibson
pip install -e .
```

#### System requirements

The minimum system requirements are the following:

- Ubuntu 16.04
- Nvidia GPU with VRAM > 6.0GB
- Nvidia driver >= 384
- CUDA >= 9.0, CuDNN >= v7

#### Download data

First, our environment core assets data are available [here](https://storage.googleapis.com/gibsonassets/assets_gibson_v2.tar.gz).  You can store the data where you want and put the path in `global_config.yaml`.  The `assets` folder stores necessary data (agent models, environments, etc) to run gibson environment. 

Users can add more environments files into `dataset` folder and put the path in `global_config.yaml` to run gibson on more environments. Visit the [database readme](gibson/data/README.md) for downloading more spaces. Please sign the [license agreement](gibson/data/README.md#download) before using Gibson's database. The default path is:

```yaml
assets_path: assets #put either absolute path or relative to current directory
dataset_path: assets/dataset
```

#### Download sample scenes

```
wget https://storage.googleapis.com/gibsonassets/gibson_mesh/Ohopee.tar.gz
```
Put the downloaded `Ohopee` scene into `dataset_path` and you should be able to run all the tests and some examples. Full dataset will be realeased soon, check back soon!


Uninstalling
----

Uninstall gibson is easy with `pip uninstall gibson2`


Quick Start
=================

Tests
----

```bash
cd test
pytest # the tests should pass, it will take a few minutes
```

Gibson v2 Framerate
----

Gibson v2 framerate compared with gibson v1 is shown in the table below:

 <table>
               <thead>
                 <tr>
                   <th scope="col"></th>
                   <th scope="col">Gibson V2</th>
                   <th scope="col">Gibson V1</th>
                 </tr>
               </thead>
               <tbody>
                 <tr>
                   <th scope="row">RGBD, pre network<code>f</code></th>
                   <td>264.1</td>
                   <td>58.5</td>
                 </tr>
                 <tr>
                   <th scope="row">RGBD, post network<code>f</code></th>
                   <td>61.7</td>
                   <td>30.6</td>
                 </tr>
                 <tr>
                   <th scope="row">Surface Normal only</th>
                   <td>271.1</td>
                   <td>129.7</td>
                 </tr>
                 <tr>
                   <th scope="row">Semantic only</th>
                   <td>279.1</td>
                   <td>144.2</td>
                 </tr>
                 <tr>
                   <th scope="row">Non-Visual Sensory</th>
                   <td>1017.4</td>
                   <td>396.1</td>
                 </tr>
               </tbody>
             </table>


Rendering Semantics
----
TBA

Robotic Agents
----

Gibson provides a base set of agents. See videos of these agents and their corresponding perceptual observation [here](http://gibsonenv.stanford.edu/agents/). 
<img src=misc/agents.gif>

To enable (optionally) abstracting away low-level control and robot dynamics for high-level tasks, we also provide a set of practical and ideal controllers for each agent.

| Agent Name     | DOF | Information      | Controller |
|:-------------: | :-------------: |:-------------: |:-------------|
| Mujoco Ant      | 8   | [OpenAI Link](https://blog.openai.com/roboschool/) | Torque |
| Mujoco Humanoid | 17  | [OpenAI Link](https://blog.openai.com/roboschool/) | Torque |
| Husky Robot     | 4   | [ROS](http://wiki.ros.org/Robots/Husky), [Manufacturer](https://www.clearpathrobotics.com/) | Torque, Velocity, Position |
| Minitaur Robot  | 8   | [Robot Page](https://www.ghostrobotics.io/copy-of-robots), [Manufacturer](https://www.ghostrobotics.io/) | Sine Controller |
| JackRabbot      | 2   | [Stanford Project Link](http://cvgl.stanford.edu/projects/jackrabbot/) | Torque, Velocity, Position |
| TurtleBot       | 2   | [ROS](http://wiki.ros.org/Robots/TurtleBot), [Manufacturer](https://www.turtlebot.com/) | Torque, Velocity, Position |
| Quadrotor         | 6   | [Paper](https://repository.upenn.edu/cgi/viewcontent.cgi?referer=https://www.google.com/&httpsredir=1&article=1705&context=edissertations) | Position |


### Starter Code 

Demonstration examples can be found in `examples/demo` folder. `demo.py` shows the procedure of starting an environment with a random agent.

ROS Configuration
---------

We provide examples of configuring Gibson with ROS [here](examples/ros/gibson-ros). We use turtlebot as an example, after a policy is trained in Gibson, it requires minimal changes to deploy onto a turtlebot. See [README](examples/ros/gibson-ros) for more details.


Coding Your RL Agent
====
You can code your RL agent following our convention. The interface with our environment is very simple (see some examples in the end of this section).

First, you can create an environment by creating an instance of classes in `gibson/core/envs` folder. 


```python
env = AntNavigateEnv(is_discrete=False, config = config_file)
```

Then do one step of the simulation with `env.step`. And reset with `env.reset()`
```python
obs, rew, env_done, info = env.step(action)
```
`obs` gives the observation of the robot. It is a dictionary with each component as a key value pair. Its keys are specified by user inside config file. E.g. `obs['nonviz_sensor']` is proprioceptive sensor data, `obs['rgb_filled']` is rgb camera data.

`rew` is the defined reward. `env_done` marks the end of one episode, for example, when the robot dies. 
`info` gives some additional information of this step; sometimes we use this to pass additional non-visual sensor values.

We mostly followed [OpenAI gym](https://github.com/openai/gym) convention when designing the interface of RL algorithms and the environment. In order to help users start with the environment quicker, we
provide some examples at [examples/train](examples/train). The RL algorithms that we use are from [openAI baselines](https://github.com/openai/baselines) with some adaptation to work with hybrid visual and non-visual sensory data.
In particular, we used [PPO](https://github.com/openai/baselines/tree/master/baselines/ppo1) and a speed optimized version of [PPO](https://github.com/openai/baselines/tree/master/baselines/ppo2).


Environment Configuration
=================
Each environment is configured with a `yaml` file. Examples of `yaml` files can be found in `examples/configs` folder. Parameters for the file is explained below. For more informat specific to Bullet Physics engine, you can see the documentation [here](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit).

| Argument name        | Example value           | Explanation  |
|:-------------:|:-------------:| :-----|
| envname      | AntClimbEnv | Environment name, make sure it is the same as the class name of the environment |
| model_id      | space1-space8      |   Scene id, in beta release, choose from space1-space8 |
| target_orn | [0, 0, 3.14]      |   Eulerian angle (in radian) target orientation for navigating, the reference frame is world frame. For non-navigation tasks, this parameter is ignored. |
|target_pos | [-7, 2.6, -1.5] | target position (in meter) for navigating, the reference frame is world frame. For non-navigation tasks, this parameter is ignored. |
|initial_orn | [0, 0, 3.14] | initial orientation (in radian) for navigating, the reference frame is world frame |
|initial_pos | [-7, 2.6, 0.5] | initial position (in meter) for navigating, the reference frame is world frame|
|fov | 1.57  | field of view for the camera, in radian |
| use_filler | true/false  | use neural network filler or not. It is recommended to leave this argument true. See [Gibson Environment website](http://gibson.vision/) for more information. |
|display_ui | true/false  | Gibson has two ways of showing visual output, either in multiple windows, or aggregate them into a single pygame window. This argument determines whether to show pygame ui or not, if in a production environment (training), you need to turn this off |
|show_diagnostics | true/false  | show dignostics(including fps, robot position and orientation, accumulated rewards) overlaying on the RGB image |
|output | [nonviz_sensor, rgb_filled, depth]  | output of the environment to the robot, choose from  [nonviz_sensor, rgb_filled, depth]. These values are independent of `ui_components`, as `ui_components` determines what to show and `output` determines what the robot receives. |
|resolution | 512 | choose from [128, 256, 512] resolution of rgb/depth image |
|mode | gui/headless/web_ui  | gui or headless, if in a production environment (training), you need to turn this to headless. In gui mode, there will be visual output; in headless mode, there will be no visual output. In addition to that, if you set mode to web_ui, it will behave like in headless mode but the visual will be rendered to a web UI server. ([more information](#web-user-interface))|
|verbose |true/false  | show diagnostics in terminal |
|fast_lq_render| true/false| if there is fast_lq_render in yaml file, Gibson will use a smaller filler network, this will render faster but generate slightly lower quality camera output. This option is useful for training RL agents fast. |

#### Making Your Customized Environment
Gibson provides a set of methods for you to define your own environments. You can follow the existing environments inside `gibson/core/envs`.

| Method name        | Usage           |
|:------------------:|:---------------------------|
| robot.get_position() | Get current robot position. |
| robot.get_orientation() | Get current robot orientation. |
| robot.eyes.get_position() | Get current robot perceptive camera position. |
| robot.eyes.get_orientation() | Get current robot perceptive camera orientation. |
| robot.get_target_position() | Get robot target position. |
| robot.apply_action(action) | Apply action to robot. |
| robot.reset_new_pose(pos, orn) | Reset the robot to any pose. |
| robot.dist_to_target() | Get current distance from robot to target. |
