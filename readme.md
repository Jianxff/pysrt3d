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
- OpenGL
- GLEW
- glfw3
- Eigen3
- OpenCV 3/4
- OpenMP

Your compilation environment should support `C++ 17`.

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
