import mujoco
import time
import mujoco.viewer


from inverse_kinematic import inverse_kinematics
from key_control import Target, make_key_callback  



xml_file = "6RManipulator.xml"

model = mujoco.MjModel.from_xml_path(xml_file)
data = mujoco.MjData(model)



target = Target(x=0.6, y=0.0, z=0.5, step=0.02)
key_callback = make_key_callback(target)


# Launch the simulation
with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    start_time = time.time()

    while viewer.is_running():
        current_time = time.time() - start_time
        dt = model.opt.timestep

        ee_position = data.site_xpos[0]
        theta1, theta2, theta3, theta4, theta5, theta6 = inverse_kinematics(target.x, target.y, target.z, elbow_up=False)

        data.qpos[model.joint("joint A").qposadr] = theta1
        data.qpos[model.joint("joint B").qposadr] = theta2
        data.qpos[model.joint("joint C").qposadr] = theta3
        data.qpos[model.joint("joint D").qposadr] = theta4
        data.qpos[model.joint("joint E").qposadr] = theta5
        data.qpos[model.joint("joint F").qposadr] = theta6



        mujoco.mj_step(model, data)
        
        viewer.sync()
        time.sleep(dt)