# Submission for the AImotive Planning and Decision Making assignment

A Model Predictive Controller (MPC) based planner and controller for controlling the ego vehicle in a simulated intersection scenario. The collision prediction algorithm is based on the concept of collision cone and velocity obstacle.

 Implements a Model Predictive Controller (MPC) for a kinematic point mass model of a vehicle, while taking into consideration the limits on velocity , acceleration and jerk. The current implementation of the MPC controller relies on the IPOPT solver (Interior Point OPTimizer) and the CppAD (C++ Algorithmic Differentiation) package
for solving the optimization problem and calculating the gradients, respectively. 

![simulation](./intersection_simulation.gif "Intersection simulation")

### TODO list 
- [ ] Update the description with more details.
- [ ] Include a 1 page PDF describing the solution.


