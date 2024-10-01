# standard library
from typing import *
from pathlib import Path
import numpy as np
# third party
import cv2
import pysrt3d


# fetch images
DATA_DIR = Path(__file__).resolve().parent / 'data'
images_dir = DATA_DIR / 'images'
images = sorted(list(images_dir.glob('*.png')))
h, w = cv2.imread(str(images[0])).shape[:2]

# init model
model = pysrt3d.Model(
    name="Cat",
    model_path=str(DATA_DIR / 'Cat.obj'),
    meta_path=str(DATA_DIR / 'Cat.meta'),
    unit_in_meter=0.001,    # the model was made in 'mm'
    threshold_init = 0.0,   # no limit while initialization
    threshold_track = 0.0,  # no limit while tracking
    kl_threshold = 1.0      
)

# init tracker
tracker = pysrt3d.Tracker(
    imwidth=w,
    imheight=h,
    K=np.loadtxt(DATA_DIR / 'K.txt', dtype=np.float32)
)

# add model to tracker
tracker.add_model(model)
# setup tracker
tracker.setup()
# make renderer
renderer = pysrt3d.Renderer(tracker)

# set init pose
init_pose = np.loadtxt(DATA_DIR / 'pose.txt')
model.reset_pose(init_pose)


for image in images:
    # read image
    image = cv2.imread(str(image))
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # tracking iteration
    tracker.update(image=image_rgb)
    # get tracking info
    pose_uv = model.pose_uv
    conf = model.conf

    # render normal view
    normal_image = renderer.render()
    # write tracking info
    cv2.putText(normal_image, f"{model.name}: {conf:.2f}", pose_uv, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow('view', normal_image)
    cv2.waitKey(0)