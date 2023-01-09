# Hybrid A*

## Description

This repository contains an implementation of Hybrid A* using Dubins Airplane Curves to plan a path in 3D space as a simplified demo of the algorithm used in [this paper](https://doi.org/10.2514/1.I011044). Note that Dubins Airplane Curves can have have some sharp edges in the vertical dimension since they assume instantaneous control of the flight path angle. Hybrid A* can also be used with other motion primitives that may provide smoother curves and/or better dynamic feasibility such as those used in the above mentioned paper. 

## Usage

Each of the Python files in the repository contains a test/example script. See the bottom of each file for these examples. 

To see the set of curves used by the planner run:

```
python dubins.py
```

To see an example path generated with the Hybrid A* planner run:

```
python hybrid_astar.py
```
