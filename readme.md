# pysrt3d

Python binding for [*SRT3D: Region-Based 6DoF Object Tracking*](https://github.com/DLR-RM/3DObjectTracking/tree/master/SRT3D)

https://github.com/user-attachments/assets/bf9b0102-c6a2-4eba-bd31-6a6ffa3dec50

## Modification
#### 1. Automatical body-diameter calculation
You don't need to specify `max_body_diameter` manually, just as [*M3T*](https://github.com/DLR-RM/3DObjectTracking/tree/master/M3T).

#### 2. Kullback-Leibler divergence based confidence assessment 
If the predicted pose is good (close to the reality), the mean value of each correspondence line will be zero (at the center, without offset). We calculate KL divergence for each correspondence line and the optimal one: $\mathcal{N}(0, \sigma_{min}^2)$.

#### 3. Thresholds for initialization and tracking
We define thresholds for initialization and tracking (you can manually set them), so that you can easily align the object to the virtual one before starting tracking.

#### 4. Independent multi-model tracking
The confidences are calculated independently. You can easily add models and get their information (pose, conf., etc.).


## Installation
Make sure that you have installed the following packages:
- GLEW
- glfw3
- Eigen3
- OpenCV 3/4
- OpenMP (may have already installed on your system)

Your compilation environment should support `C++ 17`.

#### Dependency Instruction
- [method 1] Through system path
  ```
  $ sudo apt install libglew-dev libglfw3-dev libeigen3-dev
  # BUILD opencv from source code: https://github.com/opencv/opencv
  ```

- [method 2] Under conda env
  ```
  $ conda activate ${YOUR_ENV_NAME}
  $ conda install glew glfw eigen libopencv
  ```

- Other dependencies for `demo.py`
  ```
  pip install numpy
  pip install opencv-python
  ```

#### Package Installation
```
cd ${repo_root}
pip install .
```


## Demo
\* *The demo data is from [DeepAC](https://github.com/WangLongZJU/DeepAC)*.
```
cd ${repo_root}/example
python demo.py
```


## Interface
Just follow `example/demo.py` or `source/pysrt3d/pysrt3d.cpp`.

## Note
This algorithm is for object tracking only, without global pose estimation. In consequence, an initial pose (4 by 4 matrix under opencv coordinate) must be provided before you start tracking.
