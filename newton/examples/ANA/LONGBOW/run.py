import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("./scene.xml")
data = mujoco.MjData(model)

data.ctrl[0] = 0.090
data.ctrl[1] = 0.130
data.ctrl[2] = 0.080
data.ctrl[3] = 0.050
data.ctrl[4] = 0.000

data.ctrl[5] = 0.090
data.ctrl[6] = 0.130
data.ctrl[7] = 0.080
data.ctrl[8] = 0.050
data.ctrl[9] = 0.000

mujoco.viewer.launch(model, data, show_left_ui=True, show_right_ui=True)
