# GPS2VSL APP Repository
This repository houses three Libpanda Apps. These application combine the vehicle interfaces from [can_to_ros](https://github.com/jmscslgroup/can_to_ros)
(which itself also uses the [libpanda](https://github.com/jmscslgroup/libpanda) repo), a few modular ROS-based controllers from [jmscslgroup](https://github.com/jmscslgroup/),
and new scripts contained herein. 

## List of Apps
1. 1. Middleway - Middle Way VSL experimental control, converts I24 speed limits to velocity set points, with a CBF-based safety filter and P-Control. Control will exceed the set speed somewhat if prevailing speeds locally measured exceed the set speed a lot. Outside of the TDOT Smart Corridor, the driver engaging the cruise control sets a 'speed limit' which is the entry velocity state of ACC.
2. 2. VSL - VSL experiment, converts I24 speed limits to velocity set points using CBF and PID. I.e. automatically follows variable speed limit inside TDOT Smart Corridor.
4. 3. Lead_Accel Estimator - Testing sensor-based estimation of the acceleration of the leader vehicle, investigating a Higher Order CBF-based safety filter.

These apps are combinations of the scripts in the scripts directory. These keep track of the GPS state (in/out geofenced areas, heading), as well as processing the forward facing radar sensor for a prevailing speed estimate, simple control laws for combining modular controllers, and multiplexing to keep the system state switching smooth and simplified. 

## Cite the app infrastructure:

@inproceedings{bunting2024libpanda,
  title={Libpanda Apps: Managing the Deployment and Reuse of a Cyber-Physical System},
  author={Bunting, Matthew and Nice, Matthew W and Richardson, Alex A and Sprinkle, Jonathan and Work, Daniel B},
  booktitle={2024 IEEE Workshop on Design Automation for CPS and IoT (DESTION)},
  pages={40--45},
  year={2024},
  organization={IEEE}
}

Cite the main app (MiddleWay):

@inproceedings{nice2024middle,
  title={A middle way to traffic enlightenment},
  author={Nice, Matthew W and Gunter, George and Ji, Junyi and Zhang, Yuhang and Bunting, Matthew and Barbour, Will and Sprinkle, Jonathan and Work, Daniel B},
  booktitle={2024 ACM/IEEE 15th International Conference on Cyber-Physical Systems (ICCPS)},
  pages={147--156},
  year={2024},
  organization={IEEE}
}

Cite vsl-following app:

@article{nice2023sailing,
  title={Sailing cavs: Speed-adaptive infrastructure-linked connected and automated vehicles},
  author={Nice, Matthew and Bunting, Matthew and Gunter, George and Barbour, William and Sprinkle, Jonathan and Work, Dan},
  journal={arXiv preprint arXiv:2310.06931},
  year={2023}
}
