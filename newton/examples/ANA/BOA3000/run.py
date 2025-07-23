import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("./scene.xml")
data = mujoco.MjData(model)
mujoco.viewer.launch(model, data, show_left_ui=True, show_right_ui=True)
