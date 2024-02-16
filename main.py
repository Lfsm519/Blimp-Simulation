import time

import mujoco
import mujoco.viewer
import numpy as np
import cv2


m = mujoco.MjModel.from_xml_path('sano.xml')
d = mujoco.MjData(m)

camera_name = 'blimp_camera'

# Main simulation loop
with mujoco.viewer.launch_passive(m, d) as viewer:
    start = time.time()
    while viewer.is_running() and time.time() - start < 30:
        step_start = time.time()



        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
