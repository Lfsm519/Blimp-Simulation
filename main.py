import time

import mujoco
import mujoco.viewer
import numpy as np
import cv2


m = mujoco.MjModel.from_xml_path('test.xml')
d = mujoco.MjData(m)

camera_name = 'blimp_camera'

# Main simulation loop
while True:
    # Step the simulation
    mujoco.mj_step(m, d)

    # Get the camera observation
    # camera_observation = d.render(
    #     width=640, height=480, camera_name=camera_name)
    #
    # # Convert the observation to a NumPy array
    # image_array = np.asarray(camera_observation)
    #
    # # Display or process the image as needed
    # cv2.imshow('Camera Feed', image_array)
    # cv2.waitKey(1)  # Adjust the delay as needed
