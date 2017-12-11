# Throwing Environment

An environment for BOLeRo that simulates a 3 DOF arm that throws a ball. This
is a contextual environment. The context defines the target for the throw.
This is a very simple environment that is used to demonstrate how MARS
environments can be implemented.

## Installation

Install BOLeRo first. See
[here](https://github.com/rock-learning/bolero#installation) for instructions.

Source bolero's env.sh. Then you can install this environment:

    git clone git@github.com:rock-learning/throwing_environment.git
    cd throwing_environment
    mkdir build
    cd build
    cmake_debug ..
    make install

## Getting Started

There is a very simple script that demonstrates how to use this environment in
the subfolder 'example': 'run_experiment.py'.

## License

This BOLeRo environment is distributed under the
[3-clause BSD license](https://opensource.org/licenses/BSD-3-Clause).

## Environment Parameters

When using this MARS based environments there are some basic environment
parameters, which can be changed within `learning_config.yml`:

* **calc_ms**: The time interval the physic engine does updates or the time
  between two physic updates.
* **stepTimeMs**: The time interval for one step of the environment. E.g. if
  *stepTimeMs = 20* and *calc_ms = 2* the environment will do *10* physics
  updates for each step.
* **graphicsUpdateTime**: The time interval for updating the graphical
  interface.
* **enableGUI**: If `false` MARS starts without GUI. Therefore the file
  `core_libs-nogui.txt` must exist.
* **armHeight**: Height of the arm above the ground. This is important to
  compute the point at which the ball hits the ground.
* **verbose**: Verbosity level (integer).

Example code for the `Environment` section in the
`learning_config.yml`:

    Environment:
      type: "throwing_environment"
      calc_ms: 10
      stepTimeMs: 10
      graphicsUpdateTime: 10
      enableGUI: true
      armHeight: 0.0
      verbose: 0
