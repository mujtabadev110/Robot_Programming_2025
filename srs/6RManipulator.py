import mujoco
import numpy as np
import os
import time
import math
import mujoco.viewer






xml_file = "6RManipulator.xml"

model = mujoco.MjModel.from_xml_path(xml_file)
data = mujoco.MjData(model)


# Launch the simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()

    while viewer.is_running():
        current_time = time.time() - start_time
        dt = model.opt.timestep
        

        mujoco.mj_step(model, data)
        
        viewer.sync()
        time.sleep(dt)
