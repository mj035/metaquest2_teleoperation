import numpy as np
import mujoco
import time
import os
from mujoco.glfw import glfw

# Print current directory for debugging
print(f"Current working directory: {os.getcwd()}")

try:
    # Load the model from the existing XML file
    print("Attempting to load the model...")
    # model = mujoco.MjModel.from_xml_path('dual_arm_robot.xml')
    model = mujoco.MjModel.from_xml_path('/root/CookingBot/Mujoco/dual_arm_robot.xml')
    data = mujoco.MjData(model)
    print("Model loaded successfully!")
    
    # Initialize GLFW and create window
    print("Initializing GLFW...")
    glfw.init()
    window = glfw.create_window(1200, 900, "Dual Robot Box Lifting", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)
    
    # Set up MuJoCo visualization context
    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    scene = mujoco.MjvScene(model, maxgeom=10000)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)

    # Camera settings - positioned to view the facing arms and box
    cam.distance = 1.2  # Camera distance
    cam.elevation = -20.0  # Camera elevation angle
    cam.azimuth = 0.0  # Camera azimuth - looking from the side
    cam.lookat = np.array([0.0, 0.0, 0.3])  # Camera viewpoint (center between the robots)

    # Mouse interaction variables
    button_left = False
    button_middle = False
    button_right = False
    lastx = 0
    lasty = 0

    # Keyboard/mouse callback functions
    def keyboard(window, key, scancode, act, mods):
        if act == glfw.PRESS:
            if key == glfw.KEY_ESCAPE:
                glfw.set_window_should_close(window, True)
            elif key == glfw.KEY_R:
                # Reset position
                reset_position()
                print("Reset to initial position")
            elif key == glfw.KEY_SPACE:
                # Start box lifting task
                box_lifting_task()
                print("Starting box lifting task")
            elif key == glfw.KEY_H:
                # Ready position
                ready_position()
                print("Moving to ready position")

    def mouse_button(window, button, act, mods):
        global button_left, button_middle, button_right
        
        if button == glfw.MOUSE_BUTTON_LEFT:
            button_left = (act == glfw.PRESS)
        elif button == glfw.MOUSE_BUTTON_MIDDLE:
            button_middle = (act == glfw.PRESS)
        elif button == glfw.MOUSE_BUTTON_RIGHT:
            button_right = (act == glfw.PRESS)
        
        # Update mouse position
        global lastx, lasty
        lastx, lasty = glfw.get_cursor_pos(window)

    def mouse_move(window, xpos, ypos):
        global lastx, lasty, button_left, button_middle, button_right
        
        # Only move camera when mouse button is pressed
        if button_left:
            # Camera rotation
            dy = 0.01 * (ypos - lasty)
            dx = 0.01 * (xpos - lastx)
            cam.elevation = np.clip(cam.elevation - dy*100, -90, 90)
            cam.azimuth = (cam.azimuth + dx*100) % 360
        elif button_middle:
            # Camera panning
            dx = 0.001 * (xpos - lastx)
            dy = 0.001 * (ypos - lasty)
            cam.lookat[0] += -dx*cam.distance
            cam.lookat[1] += dy*cam.distance
        elif button_right:
            # Camera zoom
            dy = 0.01 * (ypos - lasty)
            cam.distance = np.clip(cam.distance + dy, 0.1, 5.0)
        
        lastx = xpos
        lasty = ypos

    def scroll(window, xoffset, yoffset):
        # Zoom in/out with scroll
        cam.distance = np.clip(cam.distance - 0.1 * yoffset, 0.1, 5.0)

    # Register callbacks
    glfw.set_key_callback(window, keyboard)
    glfw.set_mouse_button_callback(window, mouse_button)
    glfw.set_cursor_pos_callback(window, mouse_move)
    glfw.set_scroll_callback(window, scroll)

    # Scene rendering function
    def render_scene():
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mujoco.MjrRect(0, 0, viewport_width, viewport_height)
        
        # Update scene
        mujoco.mjv_updateScene(
            model, data, opt, None, cam, 
            mujoco.mjtCatBit.mjCAT_ALL.value, scene
        )
        
        # Render
        mujoco.mjr_render(viewport, scene, context)
        
        # Text overlay (key guide)
        text = [
            "ESC: Exit",
            "SPACE: Start box lifting task",
            "R: Reset position",
            "H: Ready position"
        ]
        overlay = "\n".join(text)
        mujoco.mjr_overlay(
            mujoco.mjtFont.mjFONT_NORMAL, 
            mujoco.mjtGridPos.mjGRID_TOPLEFT, 
            viewport, 
            overlay, 
            "", 
            context
        )
        
        # Swap buffers
        glfw.swap_buffers(window)

    # Get actuator indices for both robots
    left_joints = [
        model.actuator('left_actuator_joint1').id,
        model.actuator('left_actuator_joint2').id,
        model.actuator('left_actuator_joint3').id,
        model.actuator('left_actuator_joint4').id,
        model.actuator('left_actuator_gripper_joint').id
    ]

    right_joints = [
        model.actuator('right_actuator_joint1').id,
        model.actuator('right_actuator_joint2').id,
        model.actuator('right_actuator_joint3').id,
        model.actuator('right_actuator_joint4').id,
        model.actuator('right_actuator_gripper_joint').id
    ]

    # Reset to initial pose - both arms in position to reach the box
    def reset_position():
        # Left arm initial position (facing right)
        data.ctrl[left_joints[0]] = 0.0       # Base joint
        data.ctrl[left_joints[1]] = -0.3      # Shoulder joint slightly down
        data.ctrl[left_joints[2]] = 0.6       # Elbow joint forward
        data.ctrl[left_joints[3]] = 0.0       # Wrist joint straight
        data.ctrl[left_joints[4]] = 0.01      # Gripper open

        # Right arm initial position (facing left)
        data.ctrl[right_joints[0]] = 0.0      # Base joint
        data.ctrl[right_joints[1]] = -0.3     # Shoulder joint slightly down
        data.ctrl[right_joints[2]] = 0.6      # Elbow joint forward
        data.ctrl[right_joints[3]] = 0.0      # Wrist joint straight
        data.ctrl[right_joints[4]] = 0.01     # Gripper open

    # Move to ready position - arms positioned to reach for the box
    def ready_position():
        # Left arm ready position (reaching toward center)
        data.ctrl[left_joints[0]] = 0.0       # Base joint 
        data.ctrl[left_joints[1]] = -0.4      # Shoulder joint down
        data.ctrl[left_joints[2]] = 0.8       # Elbow joint forward
        data.ctrl[left_joints[3]] = 0.2       # Wrist joint tilted for grip
        data.ctrl[left_joints[4]] = 0.015     # Gripper open
        
        # Right arm ready position (reaching toward center)
        data.ctrl[right_joints[0]] = 0.0      # Base joint
        data.ctrl[right_joints[1]] = -0.4     # Shoulder joint down
        data.ctrl[right_joints[2]] = 0.8      # Elbow joint forward
        data.ctrl[right_joints[3]] = 0.2      # Wrist joint tilted for grip
        data.ctrl[right_joints[4]] = 0.015    # Gripper open

    # Box manipulation task: Both robots pick up and hand off the box
    def box_lifting_task():
        print("Starting box manipulation task...")
        
        # 1. Move both arms to box position
        # Left arm moves to box from left side
        data.ctrl[left_joints[0]] = 0.0        # Base joint
        data.ctrl[left_joints[1]] = -0.6       # Shoulder down
        data.ctrl[left_joints[2]] = 1.0        # Elbow bent
        data.ctrl[left_joints[3]] = 0.3        # Wrist tilted
        data.ctrl[left_joints[4]] = 0.015      # Gripper open
        
        # Right arm moves to box from right side
        data.ctrl[right_joints[0]] = 0.0       # Base joint
        data.ctrl[right_joints[1]] = -0.6      # Shoulder down
        data.ctrl[right_joints[2]] = 1.0       # Elbow bent
        data.ctrl[right_joints[3]] = 0.3       # Wrist tilted
        data.ctrl[right_joints[4]] = 0.015     # Gripper open
        
        simulate(1.5)
        
        # 2. Close grippers to hold box from both sides
        data.ctrl[left_joints[4]] = -0.005     # Close left gripper
        data.ctrl[right_joints[4]] = -0.005    # Close right gripper
        simulate(1.0)
        
        # 3. Lift box together
        for i in range(15):
            # Gradually lift both arms
            data.ctrl[left_joints[1]] = -0.6 + i * 0.04   # Lift shoulder joint
            data.ctrl[right_joints[1]] = -0.6 + i * 0.04  # Lift shoulder joint
            simulate(0.1)
        
        # 4. Hold at highest position
        simulate(1.5)
        
        # 5. Pass box from left to right (left loosens, right holds)
        data.ctrl[left_joints[4]] = 0.015     # Open left gripper
        simulate(1.0)
        
        # 6. Left arm moves away
        data.ctrl[left_joints[2]] = 0.5       # Left arm pulls back
        data.ctrl[left_joints[1]] = -0.3      # Left arm moves up
        simulate(1.0)
        
        # 7. Right arm lowers box
        for i in range(15):
            # Gradually lower right arm
            data.ctrl[right_joints[1]] = -0.0 - i * 0.04  # Lower shoulder joint
            simulate(0.1)
        
        # 8. Place box down and release
        data.ctrl[right_joints[4]] = 0.015    # Open right gripper
        simulate(1.0)
        
        # 9. Return both arms to original positions
        data.ctrl[left_joints[0]] = 0.0       # Reset left base joint
        data.ctrl[left_joints[1]] = -0.3      # Reset left shoulder
        data.ctrl[left_joints[2]] = 0.6       # Reset left elbow
        data.ctrl[left_joints[3]] = 0.0       # Reset left wrist
        
        data.ctrl[right_joints[0]] = 0.0      # Reset right base joint  
        data.ctrl[right_joints[1]] = -0.3     # Reset right shoulder
        data.ctrl[right_joints[2]] = 0.6      # Reset right elbow  
        data.ctrl[right_joints[3]] = 0.0      # Reset right wrist
        simulate(2.0)
        
        print("Box manipulation task completed")

    # Simulate for specified duration with visualization
    def simulate(duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            mujoco.mj_step(model, data)
            render_scene()
            glfw.poll_events()
            if glfw.window_should_close(window):
                return False
            time.sleep(0.01)
        return True

    # Main loop
    try:
        print("=== Dual Robot Box Lifting Control ===")
        print("ESC: Exit")
        print("SPACE: Start box lifting task")
        print("R: Reset position")
        print("H: Ready position")
        
        print("Initializing robots...")
        reset_position()
        
        while not glfw.window_should_close(window):
            # Advance simulation step
            mujoco.mj_step(model, data)
            
            # Render scene
            render_scene()
            
            # Process events
            glfw.poll_events()
            
            # Delay to control simulation speed
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Simulation terminated by user")
    finally:
        # Clean up GLFW on exit
        glfw.terminate()

except Exception as e:
    print(f"Error: {e}")
    print("\nTroubleshooting steps:")
    print("1. Make sure dual_arm_robot.xml is in the current directory")
    print("2. Check that all mesh files referenced in the XML are in the assets directory")
    print("3. Try running the script again")