---
layout: post
title: Using rpg-trajectory-evaluation on multiple trajectories
date: 2022-03-19
---

How to make up configuration in the rpg-trajectory-evaluation open-source

여러개의 dataset상 혹은 relative pose Error를 구할 때, single trajectory estimate보다는 다음과 같은 multiplke trajectory estimates가 주로 활용된다.    
```
rosrun rpg_trajectory_evaluation analyze_trajectories.py \
  kitti_lego.yaml --output_dir=./results/kitti_lidar --results_dir=./results/kitti_lidar --platform laptop --odometry_error_per_dataset --plot_trajectories --rmse_table --rmse_boxplot --mul_trials=1
```

이때, 위 코드를 기준으로 kitti\_lego.yaml을 기준으로 파일들을 읽어오게 되므로, 이 yaml파일을 잘 세팅하는 것이 중요하다. yaml파일은 analyze\_trajectories\_config 폴더에서 자동으로 검색하여 찾게된다.

```
Datasets:
  KTI_00:
    label: KITTI_00
  KTI_01:
    label: KITTI_01
  KTI_02:
    label: KITTI_02
  KTI_03:
    label: KITTI_03
  KTI_04:
    label: KITTI_04
  KTI_05:
    label: KITTI_05
  KTI_06:
    label: KITTI_06
  KTI_07:
    label: KITTI_07
  KTI_08:
    label: KITTI_08
  KTI_09:
    label: KITTI_09
  KTI_10:
    label: KITTI_10
Algorithms:
  lego-org:
    fn: traj_est
    label: lego-org
  lego-patch:
    fn: traj_est
    label: lego-patch
RelDistances: []
RelDistancePercentages: []
```

공식 사이트인 
[https://github.com/uzh-rpg/rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation) 에도 잘 설명되어 있듯이    

만약 위와같이 코드와 yaml파일을 작성했다면, ./results/kitti\_lidar폴더가 생성되어 있어야하며, platform인 ./results/kitti\_lidar/laptop 가 형성되어 있어야 한다.    

이후 코드는 run할때 Algorithms에서 lego-org, lego-patch에 해당하는 폴더를 찾고, platform\_label\_dataset의 폴더를 갖고 있어야한다.    
위 예시로는 ./results/kitti\_lidar/laptop/lego-org/laptop\_lego-org\_KTI_00 폴더가 있어야한다.     
그리고 이 폴더안에는 stamped\_groundtruth.txt와 stamped\_traj\_estimate.txt가 존재해야하고, 이를 기반으로 계산하게 된다. (optional: eval_cfg.yaml)    
존재할시에는 ./results/kitti\_lidar/laptop/lego-org/laptop\_lego-org\_KTI\_00/saved\_results/traj\_est에 각 결과가 저장되게 된다. (traj\_est는 fn에 해당하는 이름)    

    
    
**** mul\_trials가 1을 넘는경우 파일이나 폴더뒤에 숫자를 더 추가해주어야 한다.



