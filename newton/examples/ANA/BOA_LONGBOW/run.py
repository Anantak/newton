import mujoco
import mujoco.viewer

# Load the scene spec
scene_spec = mujoco.MjSpec.from_file("./scene.xml")

forklift_frame = scene_spec.frame("forklift_frame")
if forklift_frame is None:
    print("ERROR: Could not find forklift_frame")
    exit(1)
else:
    print("Found forklift_frame in scene")

# Get the forklift spec
forklift_spec = mujoco.MjSpec.from_file("../BOA3000/BOA3000.xml")

if forklift_spec is None:
    print("ERROR: could not load forklift_spec")
    exit(1)
else:
    print("Loaded forklift_spec")

add_longbow = False

if add_longbow:
    # Load implement spec
    implement_spec = mujoco.MjSpec.from_file("../LONGBOW/LONGBOW.xml")

    if implement_spec is None:
        print("ERROR: implement_spec is None")
        exit(1)
    else:
        print("Loaded implement_spec")

    # Find the frame on forklift where implement is to be attached
    implement_frame = forklift_spec.frame("longbow_frame")

    if implement_frame is None:
        print("ERROR: Could not find implement_frame")
        exit(1)
    else:
        print("Found implement_frame")

    # Find implement root body
    implement_body = implement_spec.body("longbow-chassis")

    if implement_body is None:
        print("ERROR: Could not find implement_body")
        exit(1)
    else:
        print("Found implement_body")

    # Attach implement to forklift
    implement_frame.attach_body(implement_body, prefix="arms_", suffix="")
    print("Attached implement to forklift")


# Get forklift body
forklift_body = forklift_spec.body("boa3000_body")

if forklift_body is None:
    print("ERROR: Could not find forklift_body")
    exit(1)
else:
    print("Found forklift_body")

# Attach forklift body to scene
forklift_frame.attach_body(forklift_body, prefix="lift_", suffix="")
print("Attached forklift to scene")


add_mx340 = True


if add_mx340:
    tugger_frame = scene_spec.frame("tugger_frame")
    if tugger_frame is None:
        print("ERROR: Could not find tugger_frame")
        exit(1)
    else:
        print("Found tugger_frame in scene")

    # Get the mx340 spec
    tugger_spec = mujoco.MjSpec.from_file("../MT340/MT340.xml")

    if tugger_spec is None:
        print("ERROR: could not load tugger_spec")
        exit(1)
    else:
        print("Loaded tugger_spec")

    # Get tugger body
    tugger_body = tugger_spec.body("mt340_body")

    if tugger_body is None:
        print("ERROR: Could not find tugger_body")
        exit(1)
    else:
        print("Found tugger_body")

    # Attach forklift body to scene
    tugger_frame.attach_body(tugger_body, prefix="tug_", suffix="")
    print("Attached tugger to scene")


add_quad_steer_cart = True


# Are we adding quad steer cart?
if add_quad_steer_cart:
    quad_steer_cart_frame = scene_spec.frame("quad_steer_cart_frame")
    if quad_steer_cart_frame is None:
        print("ERROR: Could not find quad_steer_cart_frame")
        exit(1)
    else:
        print("Found quad_steer_cart_frame in scene")

    # Get the quad_steer_cart spec
    quad_steer_cart_spec = mujoco.MjSpec.from_file("../QUAD_STEER/QUAD_STEER.xml")

    if quad_steer_cart_spec is None:
        print("ERROR: could not load quad_steer_cart_spec")
        exit(1)
    else:
        print("Loaded quad_steer_cart_spec")

    # Get quad_steer_cart body
    quad_steer_cart_body = quad_steer_cart_spec.body("quad_steer_cart_body")

    if quad_steer_cart_body is None:
        print("ERROR: Could not find quad_steer_cart_body")
        exit(1)
    else:
        print("Found quad_steer_cart_body")

    # Attach quad_steer_cart body to scene
    quad_steer_cart_frame.attach_body(quad_steer_cart_body, prefix="cart_", suffix="")
    print("Attached quad_steer_cart to scene")

    # Hitch the cart to tug
    eq = scene_spec.add_equality()
    eq.type = mujoco.mjtEq.mjEQ_CONNECT
    eq.name = "tow_connection"
    eq.objtype = mujoco.mjtObj.mjOBJ_SITE
    eq.name1 = "cart_hitch_point"
    eq.name2 = "tug_hitch_point"
    print("Added hitch equality to scene")


# Compile the combined model
model = scene_spec.compile()
data = mujoco.MjData(model)

# Forklift controls
data.ctrl[0] = 0.00
data.ctrl[1] = 0.00
data.ctrl[2] = 0.00
data.ctrl[3] = 0.00

# Long bow controls
if add_longbow:
    # Right arm
    data.ctrl[4] = 0.0075
    data.ctrl[5] = 0.27
    data.ctrl[6] = 0.17
    data.ctrl[7] = 0.050
    data.ctrl[8] = 0.000

    # Left arm
    data.ctrl[9] = 0.0075
    data.ctrl[10] = 0.27
    data.ctrl[11] = 0.17
    data.ctrl[12] = 0.050
    data.ctrl[13] = 0.000

# Launch the viewer
mujoco.viewer.launch(model, data, show_left_ui=True, show_right_ui=True)
