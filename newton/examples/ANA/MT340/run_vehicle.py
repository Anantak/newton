# import mujoco
# import mujoco.viewer
# import time
# import numpy as np

# model = mujoco.MjModel.from_xml_path("./scene.xml")
# data = mujoco.MjData(model)


# with mujoco.viewer.launch_passive(model, data) as viewer:
#     last_print_time = 0
#     start_time = time.time()

#     while viewer.is_running():
#         current_time = time.time()
#         simulation_time = current_time - start_time

#         # Print status at intervals
#         if current_time - last_print_time > 1:
#             # self.print_status(site_names)
#             print(f"\n Simulation time: {simulation_time:.1f}s  {data.time}s")
#             print("=" * 60)
#             last_print_time = current_time

#         # Step simulation
#         mujoco.mj_step(model, data)
#         viewer.sync()
#         time.sleep(1e-8) # 10 nano seconds


import time

import mujoco
import mujoco.viewer
import numpy as np


class VehiclePoseTracker:
    def __init__(self, model_path):
        """Initialize pose and velocity tracker"""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Find vehicle body and site indices
        self.vehicle_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "mt340_body")

        # Track pose history for analysis
        self.pose_history = []
        self.time_history = []

        print("🚗 Vehicle Pose and Velocity Tracker")
        print("=" * 50)
        print(f"Vehicle body ID: {self.vehicle_body_id}")
        print("Tracking: X, Y, Heading Angle, Site Velocities")
        print("=" * 50)

    def get_vehicle_pose(self):
        """Extract vehicle pose (x, y, heading) from freejoint"""
        if self.vehicle_body_id == -1:
            print("⚠️ Vehicle body not found!")
            return 0, 0, 0

        # For freejoint, qpos contains [x, y, z, qw, qx, qy, qz]
        # Position
        x = self.data.qpos[0]  # X position
        y = self.data.qpos[1]  # Y position
        # z = self.data.qpos[2]  # Z position

        # Quaternion to heading angle
        qw = self.data.qpos[3]  # Quaternion w
        qx = self.data.qpos[4]  # Quaternion x
        qy = self.data.qpos[5]  # Quaternion y
        qz = self.data.qpos[6]  # Quaternion z

        # Convert quaternion to heading angle (yaw)
        heading = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        return x, y, heading

    def get_vehicle_velocity(self):
        """Get vehicle linear and angular velocities"""
        # For freejoint, qvel contains [vx, vy, vz, wx, wy, wz]
        vx = self.data.qvel[0]  # Linear velocity X
        vy = self.data.qvel[1]  # Linear velocity Y
        vz = self.data.qvel[2]  # Linear velocity Z

        wx = self.data.qvel[3]  # Angular velocity X (roll rate)
        wy = self.data.qvel[4]  # Angular velocity Y (pitch rate)
        wz = self.data.qvel[5]  # Angular velocity Z (yaw rate)

        # Calculate speed
        speed = np.sqrt(vx**2 + vy**2)

        return vx, vy, vz, wx, wy, wz, speed

    def get_site_velocity(self, site_name):
        """Get instantaneous velocity of a specific site"""
        try:
            # Get site ID by name
            site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)

            if site_id == -1:
                return None, f"Site '{site_name}' not found"

            # Get site position in world coordinates
            site_pos = self.data.site_xpos[site_id].copy()

            # Calculate site velocity using finite differences
            # First, we need the site's body and local position
            # site_body_id = self.model.site_bodyid[site_id]

            # Get body velocity (linear and angular)
            body_vel = np.zeros(6)
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_SITE, site_id, body_vel, 0)

            linear_vel = body_vel[:3]  # [vx, vy, vz]
            angular_vel = body_vel[3:]  # [wx, wy, wz]

            return {
                "position": site_pos,
                "linear_velocity": linear_vel,
                "angular_velocity": angular_vel,
                "speed": np.linalg.norm(linear_vel),
            }, None

        except Exception as e:
            return None, f"Error getting site velocity: {e}"

    def get_all_sites(self):
        """Get list of all available sites in the model"""
        sites = []
        for i in range(self.model.nsite):
            site_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_SITE, i)
            if site_name:
                sites.append(site_name)
        return sites

    def log_pose_data(self, current_time):
        """Log pose data for later analysis"""
        x, y, heading = self.get_vehicle_pose()
        self.pose_history.append([x, y, heading])
        self.time_history.append(current_time)

    def print_status(self, site_names=None):
        """Print comprehensive vehicle status"""
        # Get pose
        # x, y, heading = self.get_vehicle_pose()
        # heading_deg = np.degrees(heading)

        # # Get velocities
        # vx, vy, vz, wx, wy, wz, speed = self.get_vehicle_velocity()

        # print(f"\n📍 VEHICLE POSE:")
        # print(f"   Position: ({x:7.3f}, {y:7.3f}) m")
        # print(f"   Heading:  {heading_deg:7.1f}° ({heading:6.3f} rad)")

        # print(f"\n🏃 VEHICLE VELOCITY:")
        # print(f"   Linear:  ({vx:6.3f}, {vy:6.3f}, {vz:6.3f}) m/s")
        # print(f"   Angular: ({wx:6.3f}, {wy:6.3f}, {wz:6.3f}) rad/s")
        # print(f"   Speed:   {speed:6.3f} m/s")

        # Get site velocities if specified
        if site_names:
            print("\nSITE VELOCITIES:")
            for site_name in site_names:
                site_data, error = self.get_site_velocity(site_name)
                if site_data:
                    pos = site_data["position"]
                    vel = site_data["linear_velocity"]
                    print(f"   {site_name}:")
                    print(f"     Position: ({pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}) m")
                    print(f"     Velocity: ({vel[0]:6.3f}, {vel[1]:6.3f}, {vel[2]:6.3f}) m/s")
                    print(f"     Speed:    {site_data['speed']:6.3f} m/s")
                else:
                    print(f"   {site_name}: {error}")

    def run_tracker(self, site_names=None, print_interval=1.0):
        """Run the pose tracker with MuJoCo viewer"""
        print("🚀 Starting pose tracker...")

        # Show available sites
        all_sites = self.get_all_sites()
        if all_sites:
            print(f"\n📌 Available sites: {', '.join(all_sites)}")
        else:
            print("\n⚠️ No sites defined in model")

        if site_names is None:
            site_names = []

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            last_print_time = 0
            start_time = time.time()

            while viewer.is_running():
                current_time = time.time()
                passed_time = current_time - start_time
                simulation_time = self.data.time

                # Log data
                # self.log_pose_data(simulation_time)

                # Print status at intervals
                if current_time - last_print_time > print_interval:
                    self.print_status(site_names)
                    print(f"\nSimulation time: {simulation_time:.1f}s  passed: {passed_time:.1f}s")
                    print("=" * 60)
                    last_print_time = current_time

                # Step simulation
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                time.sleep(0.01)

    def save_trajectory(self, filename="vehicle_trajectory.csv"):
        """Save recorded trajectory to CSV file"""
        if not self.pose_history:
            print("No trajectory data to save")
            return

        import csv

        with open(filename, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["time", "x", "y", "heading_rad", "heading_deg"])

            for _i, (pose, t) in enumerate(zip(self.pose_history, self.time_history)):
                x, y, heading = pose
                writer.writerow([t, x, y, heading, np.degrees(heading)])

        print(f"💾 Trajectory saved to {filename}")


def add_tracking_sites_to_xml():
    """
    Example of how to add sites to your XML for tracking specific points.
    Add these to your <worldbody> section in the vehicle body:
    """
    xml_sites = """
    <!-- Add these inside your mt340_body -->
    <site name="vehicle_center" pos="0 0 0" size="0.05" rgba="1 0 0 1"/>
    <site name="front_bumper" pos="1.0 0 0" size="0.05" rgba="0 1 0 1"/>
    <site name="rear_bumper" pos="-1.0 0 0" size="0.05" rgba="0 0 1 1"/>
    <site name="left_side" pos="0 0.5 0" size="0.05" rgba="1 1 0 1"/>
    <site name="right_side" pos="0 -0.5 0" size="0.05" rgba="1 0 1 1"/>
    """
    print("💡 To track specific points, add sites to your XML:")
    print(xml_sites)


def main():
    """Main function"""
    model_path = "./scene.xml"  # Update this path

    try:
        # Show how to add sites
        add_tracking_sites_to_xml()

        # Create tracker
        tracker = VehiclePoseTracker(model_path)

        # Define sites to track (update these based on your XML)
        sites_to_track = [
            "rear_tire_center",
            # "front_bumper",
            # "rear_bumper"
            # Add your site names here
        ]

        # Run tracker
        print(f"\n🎯 Tracking sites: {sites_to_track if sites_to_track else 'None specified'}")
        tracker.run_tracker(site_names=sites_to_track, print_interval=2.0)

        # Save trajectory when done
        tracker.save_trajectory()

    except FileNotFoundError:
        print(f"❌ Error: Could not find model file '{model_path}'")
    except KeyboardInterrupt:
        print("\n👋 Tracking stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")


if __name__ == "__main__":
    main()
