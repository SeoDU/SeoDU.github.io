---
layout: post
title: VINS-Fusion Code Review - (1) Image Processing 
gh-repo: HKUST-Aerial-Robotics/VINS-Fusion
tags: [Visual SLAM]
---
# (1) Image processing 파트 중요 코드 정리



## 1. Parameter 초기화

1. main()
    ```cpp
    readParameters(config_file); // yaml 파일형식으로 parameters(camera 등)를 읽어옴
    estimator.setParameter(); // Multi thread의　경우　ProcessMeasurements　실행
    ...
    std::thread sync_thread{sync_process}; // sync_process를 진행하는 thread 생성
    ...
    ```
    
    - global 변수로 선언된 estimator의 setParameter함수 호출.
    - publish, subscribe할 topic과 node, callback함수 지정.    
        이때, feature_callback은 사용되지 않는다. vins-mono에서는 독립적인 node로 분리되어 있으나, vins-fusion에서는 직접 map으로 전달하는 형식을 사용한다.    
    - sync_process 함수를 진행하는 thread를 생성.
    - 또한 estimator 생성시에 clearState()함수 call.    


2. estimator.setParameter() 
    ```cpp
    ...
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
    }
    ...
    td = TD;
    g = G;
    f_manager.setRic(ric);
    featureTracker.readIntrinsicParameter(CAM_NAMES);
    ...
    processThread = std::thread(&Estimator::processMeasurements, this);
    ...
    ```
    
    - readParameters() 에서 body_T_cam0으로 읽은 RIC, TIC의 값을 저장한다.
    - f_manager ric 초기화
    - time offset (td), gravity(G) 초기화
    - featureTracker.readIntrinsicParameter() → camera model 초기화
    - Mutl thread인 경우 processMeasurements함수를 추가 thread에서 진행한다

## 2. Image estimation

1. sync_process()
    
    ```cpp
    ...
    estimator.inputImage(time, image);
    ...
    ```
    
    - image message를 opencv Mat으로 변환 후, **estimator.inputImage(time,image)**를 호출
    
2. Estimator::inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
    
    ```cpp
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame; // 7x1 matrix
    if(_img1.empty())
        featureFrame = featureTracker.trackImage(t, _img);
    else
        featureFrame = featureTracker.trackImage(t, _img, _img1);
    ...
    featureBuf.push(make_pair(t, featureFrame));
    processMeasurements();
    ...
    ```
    
    - featureTracker.trackImage(t,_img)로 얻은 값으로 featureFrame map에 지정
    - queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>>>
    - Multithread가 아닌경우 여기서 processMeasurements()함수 진행
    
    type을 가지는 featureBuf에 pair를 만들어서 넣어준다.
    
3. ... FeatureTracker::trackImage(...)
    
    ```cpp
    **cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1,**
    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
    
    **setMask(); // track_cnt가 높은순으로 30pixel 반지름 원마다 feature를 추려냄.**
    cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
    for (auto &p : n_pts)
    {
        cur_pts.push_back(p); // n_pts에 담겨있는 코너들에 추가로 cur_pts에 넣는다.
        ids.push_back(n_id++); // 새로운 코너(feature)마다 새 id를 붙임
        track_cnt.push_back(1); // track_cnt 도 1로 intialization해줌
    }
    cur_un_pts = undistortedPts(cur_pts, m_camera[0]); // 3D로 바꾸고 다시 2D homogeneous로
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map); // 이동한 feature 찾으면 v구하고, 아니면 (0,0)
    
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;
    
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;
    
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    
    return featureFrame;
    ```
    
    - calcOpticalFlowPyrLK를 이용하여 prev_pts가 cur_img에서 찾아지면 cur_pts에 넣는다.
    - 같은 feature를 찾으면 status  = 1, 아니면 0으로 지정 (reverse check도 수행)
    - reduceVector에서는 status가 0인 부분은 지우고 resize한다.
    - mask를 지정하여 feature를 30pixel 이상 떨어지도록 분포시킨다
    - cv::goodFeaturesToTrack은 코너검출을 위해 4가지 커널을 통해 추출하는 것이다. mask로 지정한 부분은 추출되지 않는다. 이외의 새로운 feature를 max를 안넘게 추출하려는 의도이다.
    - undistortedPts 에서 distortion을 없애고, ptsVelocity에서 undistorted points를 이용하여 이전과 지금 feature간의 velocity들을 계산한다. (reduceVector로 인해 id에는 이미 이전과 지금 frame 두곳에 존재하는 것만 있음)
    - featureFrame의 feature_id index에, (camera_id , xyz_uv_velocity) (vector) 를 넣어준다.    
        xyz_uv_velocity는 undistort된 x,y와 z=1 (undistortPts에서 normalized됨), image의 x,y, **(u,v)**, 위에서 구한 velocity x, velocity y를 넣어준다.
        
    
    특징점(corners)를 찾는 알고리즘에 대해서는 다음을 참조한다.
    
    [reference] : [https://darkpgmr.tistory.com/131](https://darkpgmr.tistory.com/131)
    
4. FeatureTracker::setMask()
    
    ```cpp
    
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    
    for (unsigned int i = 0; i < cur_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
    {
      return a.first > b.first;
    });
    
    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            cur_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
    ```
    
    - cnt_pts_id에 [track_cnt, [cur_pts, ids] ]로 pair를 만들어 넣는다.
    - track_cnt가 높은 순으로 정렬하고,  mask는 처음에 255로 초기화 되있으므로, mask에서 **feature들 위치의 주변에 circle을 만들어 이 주변 값들은 cur_pts, ids, track_cnt에서 제외한다.**   



5. ... FeatureTracker::undistortedPts(vectorcv::Point2f &pts, camodocal::CameraPtr cam)
```cpp
vector<cv::Point2f> un_pts;
for (unsigned int i = 0; i < pts.size(); i++)
{
    Eigen::Vector2d a(pts[i].x, pts[i].y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
}
return un_pts;
```


- cam→liftProjective에서 intrinsic matrix를 이용해 normalize하고 recursive distortion model을 사용하여 cur_pts의 점들의 distortion을 제거한다. (Recursive distortion model 출처 확인 필요)



## 3. KeyFrame Selection

1.  Estimator::processMeasurements()
    ```cpp
    ...
    processImage(feature.second, feature.first);
    ...
    ```
    
    - 앞의 IMU 처리과정은 다음 Page에서 리뷰.
    - feature는 queue인 featureBuf에서 가장 앞에 것을 꺼낸 것으로, 가장 오래된 time과 featureFrame을 얻는다. → feature.second가 addFeatureCheckParallax의 image
    - processImage에서 f_manager.addFeatureCheckParallax(frame_count, image, td) 진행
    - frame_count는 처음에는 0이지만 window_size까지 증가한다.
    
2. bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
    
    ```cpp
    ...
    int feature_id = id_pts.first;
    for (auto &id_pts : image) // Image에　있는　feature points마다　iteration
    {
    	FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
    	auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
    	                          {
    	            return it.feature_id == feature_id;
    	                          });
    	if (it == feature.end()) // 없다면　해당　feature와　frame_count를　넣어준다．(아마　해당　frame에　그　feature가　있다고　하는듯　하다．)
            { 
                feature.push_back(FeaturePerId(feature_id, frame_count));
                feature.back().feature_per_frame.push_back(f_per_fra);
                new_feature_num++;
            }
            else if (it->feature_id == feature_id) // 만약　찾았으면　그　feature에　해당　FeaturePerFrame을　넣어준다．
            {
                it->feature_per_frame.push_back(f_per_fra);
                last_track_num++;
                if( it-> feature_per_frame.size() >= 4)
                    long_track_num++;
            }
    
    if (frame_count < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
          return true;
    
    for (auto &it_per_id : feature)
    {
        // 최소　2 step전에　찾아둔　frame이면서，이　feature를　가지는　frame이　start이후　계속　있다면
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count); // second, thrid　frame에서　feature의　위치차이를　보고
            parallax_num++;
        }
    }
    
    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        last_average_parallax = parallax_sum / parallax_num * FOCAL_LENGTH;
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
    ```
    
    - feature_id는 id_pts.first 즉, **featureFrame**요소의 first이므로 **feature_id**가 된다. 
    - FeaturePerId를 갖는 feature list에서 feature_id과 같은 것을 iterator로 가져온다.
        (앞에서 processMeasurements와는 다른 feature임)
        
    - 못찾으면 feature와 f_per_fra (FeaturePerFrame)을 feature에 넣어준다.
    - 즉 feature리스트는 **FeaturePerId**를 요소로 갖는 리스트이고, 이 FeaturePerId가 갖는 **feature_per_frame** vector에 FeaturePerFrame인 **f_epr_fra**를 넣어준다.

    - \<Keyframe 지정 알고리즘>    
      (1) frame_count < 2         
        (2) 지금 image에서 기존 feature를 별로 못찾은 경우 (last_track_num<20)      
      (3) image에서 기존 feature를 갖는 frame이 4개이상인 횟수가 적은경우 (long_track_num<40)     
        (4) 새로운 feature의 갯수가 기존 image의 feature갯수의 반보다 큰 경우      
      (5) second last third last frame에서의 parallax의 평균이 MIN_PARALLAX보다 큰 경우

<br/>


---

VINS-Fusion 코드를 정리한 포스트입니다.


1. [VINS-Fusion Code Review - (1) Image Processing](https://seodu.github.io/2022-01-05-VINS-Fusion-1/)    
2. [VINS-Fusion Code Review - (2) IMU Processing](https://seodu.github.io/2022-01-05-VINS-Fusion-2/)    
3. [VINS-Fusion Code Review - (3) Initialization](https://seodu.github.io/2022-01-05-VINS-Fusion-3/)    
4. [VINS-Fusion Code Review - (4) Sliding window & Optimization](https://seodu.github.io/2022-01-05-VINS-Fusion-4/)    
5. [VINS-Fusion Code Review - (5) Marginalization](https://seodu.github.io/2022-01-05-VINS-Fusion-5/)    
6. [VINS-Fusion Code Review - (6) Graph optimization](https://seodu.github.io/2022-01-05-VINS-Fusion-6/)    



## Reference

- [[2017] Quaternion kinematics for the error-state Kalman filter.pdf](https://arxiv.org/abs/1711.02508)    
- [[2005] Indirect Kalman Filter for 3D Attitude Estimation.pdf](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf)    
- [Formula Derivation and Analysis of the VINS-Mono.pdf](https://arxiv.org/abs/1912.11986)    
- [Marginalization&Shcurcomplement.pptx](https://slideplayer.com/slide/12551914/)    
- [[TRO2012] Visual-Inertial-Aided Navigation for High-Dynamic Motion in Built Environments Without Initial Conditions.pdf](https://kvmanohar22.github.io/notes/w03/main.pdf)    
- [VINS-Mono.pdf](https://ieeexplore.ieee.org/document/8421746)    


