# CL_Measurement_Scheduling
Measurement Scheduling for Cooperative Localization in Resource-constrained Conditions, 2018-2020.

This is the code of simulation for measurement scheduling CL associated with this [paper](https://ieeexplore.ieee.org/abstract/document/8972554/).

Please cite our work if you use this code in your research/project:

```
@article{yan2020measurement,
  title={Measurement Scheduling for Cooperative Localization in Resource-Constrained Conditions},
  author={Yan, Qi and Jiang, Li and Kia, Solmaz S},
  journal={IEEE Robotics and Automation Letters},
  volume={5},
  number={2},
  pages={1991--1998},
  year={2020},
  publisher={IEEE}
}
```

<p float='left'>
	<img src="ral_cl.png" width="400"/>
</p>

[[arXiv](https://arxiv.org/abs/1912.04709)] [[video](https://www.youtube.com/watch?v=5KAiav6astY)]

## Abstract

This paper studies the measurement scheduling problem for a group of N mobile robots moving on a flat surface that are preforming cooperative localization (CL). We consider a scenario in which due to the limited on-board resources such as battery life and communication bandwidth only a given number of relative measurements per robot are allowed at observation and update stage. Optimal selection of which teammates a robot should take a relative measurement from such that the updated joint localization uncertainty of the team is minimized is an NP-hard problem. In this paper, we propose a suboptimal greedy approach that allows each robot to choose its landmark robots locally in polynomial time. Our method, unlike the known results in the literature, does not assume full-observability of CL algorithm. Moreover, it does not require inter-robot communication at scheduling stage. That is, there is no need for the robots to collaborate to carry out the landmark robot selections. We discuss the application of our method in the context of an state-of the-art decentralized CL algorithm and demonstrate its effectiveness through numerical simulations. Even though our solution does not come with rigorous performance guarantees, its low computational cost along with no communication requirement makes it an appealing solution for operations with resource constrained robots.

# Getting started

Run `Main_modul.m` (single run) and `Main_modul_Monte.m` (multiple runs) for CL with generated data.

Run `Main_modul_dataset.m` for CL with [UTIAS dataset](http://asrl.utias.utoronto.ca/datasets/mrclam/index.html). 
