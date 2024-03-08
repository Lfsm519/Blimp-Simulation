import time

import mujoco
import mujoco.viewer
import numpy as np
import cv2

height=480
width=620


size = (width, height) 
   
# Below VideoWriter object will create 
# a frame of above defined The output  
# is stored in 'filename.avi' file. 
# result = cv2.VideoWriter('filename.mp4',  
#                          cv2.VideoWriter_fourcc(*'MJPG'), 
#                          60, size) 


m = mujoco.MjModel.from_xml_path('sano.xml')
d = mujoco.MjData(m)

camera_name = 'blimpCamera'
renderer = mujoco.Renderer(m, height=height, width=width )

thrust_vec = [0,0,0]


def key_callback(keycode):
    global thrust_vec
    print(keycode)
    if keycode == 265:
        thrust_vec =[-0.4,0,thrust_vec[2]]
    if keycode == 263:
        thrust_vec =[0,-0.4,thrust_vec[2]]
    if keycode == 264:
        thrust_vec =[0.4,0,thrust_vec[2]]
    if keycode == 262:
        thrust_vec =[0,0.4,thrust_vec[2]] 
    if chr(keycode) == "U":
        thrust_vec = [0,0,-0.4]
    if chr(keycode) == "J":
        thrust_vec = [0,0,0.4]
#    else:
#        thrust_vec = [0 , 0 , 0]
# Main simulation loop
with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
    start = time.time()
    while True:
        step_start = time.time()

        mujoco.mj_step(m, d)
        renderer.update_scene(d, camera=camera_name)
        pixels = renderer.render()
        img = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
        # result.write(img)
        actuator1 = d.actuator('prop_joint')
        actuator2 = d.actuator('prop_joint2')
        actuator3 = d.actuator('prop_joint3')
        actuator4 = d.actuator('prop_joint4')
        actuator5 = d.actuator('prop_joint5')
        actuator6 = d.actuator('prop_joint6')
        actuator1.ctrl = [thrust_vec[0]];
        actuator2.ctrl = [thrust_vec[1]];
        actuator3.ctrl = [-thrust_vec[1]];
        actuator4.ctrl = [-thrust_vec[0]];
        actuator5.ctrl = [thrust_vec[2]];
        actuator6.ctrl = [thrust_vec[2]];
        # print(d.sensor('body_gyro').data.copy())
        #d.sensor('body_quat').data.copy()

        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

    result.release()
