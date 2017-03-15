hateb_local_planner ROS Package
===============================

The hateb_local_planner package implements a plugin to the base_local_planner of the 2D navigation stack.
It implements a human-robot cooperative navigation planner that predicts trajectories for human and plans accordingly
a trajectory for robot that satisfies certain kinodynamic and social constraints.

Details of the approach are discussed in
- Khambhaita H., Alami R.: A Human-Robot Cooperative Navigation Planner, Proc. of the Companion of the 2017 ACM/IEEE International Conference on Human-Robot Interaction, Vienna, Austria, March 2017.

It uses a method called Timed Elastic Band for optimization purpose, which is discussed in
- Rösmann C., Feiten W., Wösch T., Hoffmann F. and Bertram. T.: Trajectory modification considering dynamic constraints of autonomous robots. Proc. 7th German Conference on Robotics, Germany, Munich, May 2012, pp 74–79.
- Rösmann C., Feiten W., Wösch T., Hoffmann F. and Bertram. T.: Efficient trajectory optimization using a sparse model. Proc. IEEE European Conference on Mobile Robots, Spain, Barcelona, Sept. 2013, pp. 138–143.
- Rösmann C., Hoffmann F. and Bertram. T.: Planning of Multiple Robot Trajectories in Distinctive Topologies, Proc. IEEE European Conference on Mobile Robots, UK, Lincoln, Sept. 2015.

### License

The *hateb_local_planner* package is licensed under the BSD license.
It depends on other ROS packages, which are listed in the package.xml. They are also BSD licensed.

Some third-party dependencies are included that are licensed under different terms:
 - *Eigen*, MPL2 license, http://eigen.tuxfamily.org
 - *libg2o* / *g2o* itself is licensed under BSD, but the enabled *csparse_extension* is licensed under LGPL3+,
   https://github.com/RainerKuemmerle/g2o. [*CSparse*](http://www.cise.ufl.edu/research/sparse/CSparse/) is included as part of the *SuiteSparse* collection, http://www.suitesparse.com.
 - *Boost*, Boost Software License, http://www.boost.org

All packages included are distributed in the hope that they will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the licenses for more details.

### Requirements

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file) using *rosdep*:

    rosdep install teb_local_planner


