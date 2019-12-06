# CL_Measurement_Scheduling
Measurement Scheduling for Cooperative Localization in Resource-constrained Conditions, 2018-2019 at SJTU & EPFL.

This is the code of simulation for measurement scheduling CL and the approach is based on the following work under review:
Measurement scheduling for cooperative localization in resource-constrained conditions, Q. Yan, L. Jiang, Solmaz S. Kia IEEE Robotics and Automation Letters , submitted 2019 (under review)

## Abstract

In this paper, we study the measurement scheduling problem for a group of N mobile robots participating in a cooperative localization (CL) scheme. We consider a scenario in which due to the limited on-board resources such as battery life and communication bandwidth only a given number of relative measurements per robot are allowed at observation and update stage. Optimal selection of which teammates a robot should take a relative measurement from such that the updated joint localization uncertainty of the team is minimized is an NPhard problem. In this paper, we propose a suboptimal greedy approach that allows each robot to choose its landmark robots locally in polynomial time. Our method, unlike the known results in the literature, does not assume full-observability of CL algorithm. Moreover, it does not require inter-robot communication at scheduling stage. That is, there is no need for the robots to collaborate to carry out the landmark robot selections. We discuss the application of our method in the context of an state-of-the-art decentralized CL algorithm and demonstrate its effectiveness through numerical simulations.
