### Reset-free Trial-and-Error Learning for Robot Damage Recovery

#### Konstantinos Chatzilygeroudis, Vassilis Vassiliades and Jean-Baptiste Mouret

**Affiliations**:

- Inria Nancy Grand-Est, France
- CNRS, France
- University of Lorraine, France

### Citing this code

If you use our code for a scientific paper, please cite:

Chatzilygeroudis, K., Vassiliades, V., & Mouret, J.-B. (2018). [Reset-free Trial-and-Error Learning for Robot Damage Recovery](https://arxiv.org/abs/1610.04213). *Robotics and Autonomous Systems*.

In BibTex:

    @article{chatzilygeroudis2018resetfree,
      title = {{Reset-free Trial-and-Error Learning for Robot Damage Recovery}},
      journal = {Robotics and Autonomous Systems},
      year = {2018},
      issn = {0921-8890},
      doi = {https://doi.org/10.1016/j.robot.2017.11.010},
      url = {https://www.sciencedirect.com/science/article/pii/S0921889017302440},
      author = {Chatzilygeroudis, Konstantinos and Vassiliades, Vassilis and Mouret, Jean-Baptiste},
      organization={Elsevier}
    }

#### Dependencies

- Ubuntu 14.04, 15.04 (and it should run on 16.04)
- DART simulator, http://dartsim.github.io/ (release-6.1 branch)
- Eigen 3, http://eigen.tuxfamily.org/
- Boost
- ROS (Optional - only for the physical robot)

#### Components (in alphabetical order)

##### General

- `mcts` contains the implementation of a generic and lightweight C++14 library for Monte Carlo Tree Search algorithms

##### Hexapod Experiments (in rte-hexa folder)

- `hexapod_common` contains code for the low-level controller of the hexapod (both in simulation and real robot) and a URDF model of the hexapod needed for ROS/DART.
- `hexapod_ros` contains code for controlling our custom physical hexapod
- `hexapod_simu` contains a DART integration for our hexapod [requires hexapod_common]
- `limbo` contains the limbo workspace for Reset-free Trial-and-Error algorithm. The actual code is in folder `limbo/exp/rte-hexa/`
- `sferes` contains the sferes workspace for generating an action repertoire with the MAP-Elites algorithm. The actual code is in folder `sferes/exp/map_elites_hexapod/`

##### Mobile Robot Experiments (in rte-mobile folder)

- `libfastsim` contains code for simulating simple velocity controlled differential drive robots.
- `limbo` contains the limbo workspace for Reset-free Trial-and-Error algorithm. The actual code is in folder `limbo/exp/rte_mobile/`
- `sferes` contains the sferes workspace for generating an action repertoire with the MAP-Elites algorithm. The actual code is in folder `sferes/exp/map_elites_mobile/`

#### Installing

##### Hexapod Experiments

- Install dependencies
  - `export RESIBOTS_DIR=/path/to/workspace`
  - `cd path/to/hexapod_common/hexapod_models`
  - `./waf configure --prefix=$RESIBOTS_DIR`
  - `./waf install`
  - `cd path/to/hexapod_common/hexapod_controller`
  - `./waf configure --prefix=$RESIBOTS_DIR`
  - `./waf install`
  - `cd path/to/hexapod_simu/hexapod_dart/`
  - `./waf configure --prefix=$RESIBOTS_DIR`
  - `./waf install`
  - `cd path/to/mcts`
  - `./waf configure --prefix=$RESIBOTS_DIR`
  - `./waf install`

##### Mobile Robot Experiments

- Install dependencies
  - `export RESIBOTS_DIR=/path/to/workspace`
  - `cd path/to/libfastsim`
  - `./waf configure --prefix=$RESIBOTS_DIR`
  - `./waf install`
  - `cd path/to/mcts`
  - `./waf configure --prefix=$RESIBOTS_DIR`
  - `./waf install`

#### Compiling MAP-Elites to generate the Action Repertoire

- `cd path/to/sferes` (`rte_hexa/sferes` for the hexapod and `rte_mobile/sferes` for the mobile robot)
- `./waf configure --exp name_of_experiment`
- `./waf --exp name_of_experiment`
- `name_of_experiment` can be either `map_elites_hexapod` or `map_elites_mobile`

#### Running MAP-Elites to generate Action Repertoire

##### Hexapod Experiments

- `cd path/to/sferes` (`rte_hexa/sferes` for the hexapod)
- `./build/exp/map_elites_hexapod/hexa_position_text_position_only`
- This will generate an action repertoire in `path/to/sferes/build/exp/map_elites_hexapod` folder (\*.bin files). Typical runs take 1-2 days in a cluster.

##### Mobile Robot Experiments

- `cd path/to/sferes` (`rte_mobile/sferes` for the mobile robot)
- `./build/exp/map_elites_mobile/mobile_text`
- This will generate an action repertoire in `path/to/sferes/build/exp/map_elites_mobile` folder (\*.bin files). Typical runs take a few hours in a cluster.

#### Compiling Reset-free Trial-and-Error learning algorithm

- `cd path/to/limbo`
- `./waf configure --exp name_of_experiment`
- `./waf --exp name_of_experiment`
- `name_of_experiment` can be either `rte-hexa` or `rte_mobile`
- If you have the physical hexapod append `-- robot` to both commands (requires ROS packages, e.g. `hexapod_ros` and `tf`)

#### Running Reset-free Trial-and-Error learning algorithm

##### Hexapod Experiments

- `cd path/to/limbo` (`rte_hexa/limbo` for the hexapod)
- `./build/exp/rte-hexa/rte_hexa_graphic_low_dim -m path/to/action_repertoire.bin -l exp/rte-hexa/test_maps/map_{0,1,2,3}.txt -i 5000 -k 0.0 -d 0.03 -p 4 -c 150 -r [list of leg indices to remove] -s [list of leg indices to shorten]`
- We provide 2 sample action repertoires (`action_repertoire_1.bin`, `action_repertoire_2.bin` in `path/to/limbo` folder)

##### Mobile Robot Experiments

- `cd path/to/limbo` (`rte_mobile/limbo` for the mobile robot)
- `./build/exp/rte_mobile/rte_mobile_graphic -m path/to/action_repertoire.bin -l exp/rte_mobile/map_small.txt -p 4 -c 150 -a 0.5 -b 0.6 -i 5000 -v 30 -d 1`
- We provide the action repertoires (`action_repertoire.bin` in `path/to/limbo` folder)

#### Notes

This version of code does not include the rough terrain experiments and the TEXPLORE baseline for the hexapod robot. Do not hesitate to contact the authors for details and/or this missing code.

#### Contact

Please contact Konstantinos Chatzilygeroudis ([konstantinos.chatzilygeroudis@inria.fr](mailto:konstantinos.chatzilygeroudis@inria.fr)) for any inquires or difficulties.
