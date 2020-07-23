# CL_Measurement_Scheduling
Measurement Scheduling for Cooperative Localization in Resource-constrained Conditions, 2018-2020.

This is the code of simulation for measurement scheduling CL and the approach is based on the following work:
**[Measurement scheduling for cooperative localization in resource-constrained conditions](https://ieeexplore.ieee.org/abstract/document/8972554/)**

**Q. Yan**, L. Jiang, Solmaz S. Kia, *IEEE Robotics and Automation Letters*, 2020

<p float='left'>
	<img src="https://qiyan98.github.io/images/RAL2020.png" width="250"/>
</p>

[[arXiv](https://arxiv.org/abs/1912.04709)] [[video](https://www.youtube.com/watch?v=5KAiav6astY)] [[code](https://github.com/qiyan98/CL_Measurement_Scheduling)]

## Abstract

This paper studies the measurement scheduling problem for a group of N mobile robots moving on a flat surface that are preforming cooperative localization (CL). We consider a scenario in which due to the limited on-board resources such as battery life and communication bandwidth only a given number of relative measurements per robot are allowed at observation and update stage. Optimal selection of which teammates a robot should take a relative measurement from such that the updated joint localization uncertainty of the team is minimized is an NP-hard problem. In this paper, we propose a suboptimal greedy approach that allows each robot to choose its landmark robots locally in polynomial time. Our method, unlike the known results in the literature, does not assume full-observability of CL algorithm. Moreover, it does not require inter-robot communication at scheduling stage. That is, there is no need for the robots to collaborate to carry out the landmark robot selections. We discuss the application of our method in the context of an state-of the-art decentralized CL algorithm and demonstrate its effectiveness through numerical simulations. Even though our solution does not come with rigorous performance guarantees, its low computational cost along with no communication requirement makes it an appealing solution for operations with resource constrained robots.
