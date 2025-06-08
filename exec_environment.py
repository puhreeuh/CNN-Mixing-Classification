import sys
import cv2
import numpy as np
import os
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class Simulation():
    def __init__(self, scenario="top_bottom", output_dir="dataset"):
        self.directions = ['Up', 'Down', 'Left', 'Right']
        self.object_shapes_handles = []
        self.obj_type = "Cylinder"
        self.scenario = scenario  # "left_right" or "top_bottom"
        self.output_dir = output_dir
        
        # Create output directories if they don't exist
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        if not os.path.exists(os.path.join(output_dir, "not-mixed")):
            os.makedirs(os.path.join(output_dir, "not-mixed"))
        if not os.path.exists(os.path.join(output_dir, "well-mixed")):
            os.makedirs(os.path.join(output_dir, "well-mixed"))
        
        self.initializeSim()

    def initializeSim(self):
        self.client = RemoteAPIClient()
        self.client.setStepping(True)
        self.sim = self.client.getObject('sim')

        # Ensure simulation is stopped before loading the scene
        try:
            if self.sim.getSimulationState() != self.sim.simulation_stopped:
                print("Stopping simulation...")
                self.sim.stopSimulation()
                time.sleep(1)
        except Exception as e:
            print(f"Error stopping simulation: {e}")

        # Set idle loop to full speed
        self.defaultIdleFps = self.sim.getInt32Param(self.sim.intparam_idle_fps)
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)

        # Correct scene path with proper backslashes
        scene_path = r'C:\DEEP LEARNING\project1_start_code_new\mix_intro_AI.ttt'
        try:
            print(f"Loading scene: {scene_path}")
            self.sim.loadScene(scene_path)
        except Exception as e:
            print(f"Error loading scene: {e}")

        self.getObjectHandles()

        try:
            # Get vision sensor handle
            self.vision_sensor_handle = self.sim.getObject("/Vision_sensor")
            # Enable explicit handling for the vision sensor - THIS FIXES THE ERROR
            self.sim.setExplicitHandling(self.vision_sensor_handle, 1)
        except Exception as e:
            print(f"Error configuring vision sensor: {e}")

        try:
            self.sim.startSimulation()
            print("Simulation started")
        except Exception as e:
            print(f"Error starting simulation: {e}")

        # Drop objects based on scenario
        if self.scenario == "left_right":
            self.dropObjectsLeftRight()
        else:  # "top_bottom"
            self.dropObjectsTopBottom()

    def getObjectHandles(self):
        self.tableHandle = self.sim.getObject('/Table')
        self.boxHandle = self.sim.getObject('/Table/Box')
        self.sim.setEngineFloatParam(self.sim.bullet_body_friction, self.boxHandle, 0.06)
    
    def dropObjectsLeftRight(self):
        """Drop 4 red cylinders to the left and 4 blue cylinders to the right of the box"""
        print("Dropping objects in left-right configuration...")
        self.blocks = 8
        frictionCube = 0.06
        blockLength = 0.016
        massOfBlock = 14.375e-03

        self.client.step()

        base_position = self.sim.getObjectPosition(self.boxHandle)
        delta = 0.016

        blue_color = [0.0, 0.0, 1.0]
        red_color = [1.0, 0.0, 0.0]

        # Drop 4 blue cylinders on the right side
        for i in range(2):
            for j in range(2):
                new_position = base_position.copy()
                new_position[0] += delta * 1  # Right side
                new_position[1] += (j - 0.5) * delta * 2
                new_position[2] += (i + 0.5) * delta * 2

                blue_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_cylinder, 
                                                          [blockLength, blockLength, blockLength], 0)

                self.sim.setObjectPosition(blue_handle, -1, new_position)
                self.sim.setShapeColor(blue_handle, None, self.sim.colorcomponent_ambient_diffuse, blue_color)
                self.sim.setEngineFloatParam(self.sim.bullet_body_friction, blue_handle, frictionCube)
                self.sim.setObjectFloatParam(blue_handle, self.sim.shapefloatparam_mass, massOfBlock)
                self.sim.setObjectInt32Param(blue_handle, self.sim.shapeintparam_static, 0)
                self.sim.setObjectInt32Param(blue_handle, self.sim.shapeintparam_respondable, 1)
                self.sim.resetDynamicObject(blue_handle)
                self.object_shapes_handles.append(blue_handle)

        # Drop 4 red cylinders on the left side
        for i in range(2):
            for j in range(2):
                new_position = base_position.copy()
                new_position[0] -= delta * 1  # Left side
                new_position[1] += (j - 0.5) * delta * 2
                new_position[2] += (i + 0.5) * delta * 2

                red_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_cylinder, 
                                                         [blockLength, blockLength, blockLength], 0)
                
                self.sim.setObjectPosition(red_handle, -1, new_position)
                self.sim.setShapeColor(red_handle, None, self.sim.colorcomponent_ambient_diffuse, red_color)
                self.sim.setEngineFloatParam(self.sim.bullet_body_friction, red_handle, frictionCube)
                self.sim.setObjectFloatParam(red_handle, self.sim.shapefloatparam_mass, massOfBlock)
                self.sim.setObjectInt32Param(red_handle, self.sim.shapeintparam_static, 0)
                self.sim.setObjectInt32Param(red_handle, self.sim.shapeintparam_respondable, 1)
                self.sim.resetDynamicObject(red_handle)
                self.object_shapes_handles.append(red_handle)

        # Wait for objects to settle
        for _ in range(50):
            self.stepSim()
            
        print('Blocks finished dropping in left-right configuration')

    def dropObjectsTopBottom(self):
        """Drop 4 red cylinders to the top and 4 blue cylinders to the bottom of the box"""
        print("Dropping objects in top-bottom configuration...")
        self.blocks = 8
        frictionCube = 0.06
        blockLength = 0.016
        massOfBlock = 14.375e-03

        self.client.step()

        base_position = self.sim.getObjectPosition(self.boxHandle)
        delta = 0.016

        blue_color = [0.0, 0.0, 1.0]
        red_color = [1.0, 0.0, 0.0]

        # Drop 4 red cylinders on the top half
        for i in range(2):
            for j in range(2):
                new_position = base_position.copy()
                new_position[0] += (j - 0.5) * delta * 2
                new_position[1] += delta * 1  # Top half
                new_position[2] += (i + 0.5) * delta * 2

                red_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_cylinder, 
                                                         [blockLength, blockLength, blockLength], 0)
                
                self.sim.setObjectPosition(red_handle, -1, new_position)
                self.sim.setShapeColor(red_handle, None, self.sim.colorcomponent_ambient_diffuse, red_color)
                self.sim.setEngineFloatParam(self.sim.bullet_body_friction, red_handle, frictionCube)
                self.sim.setObjectFloatParam(red_handle, self.sim.shapefloatparam_mass, massOfBlock)
                self.sim.setObjectInt32Param(red_handle, self.sim.shapeintparam_static, 0)
                self.sim.setObjectInt32Param(red_handle, self.sim.shapeintparam_respondable, 1)
                self.sim.resetDynamicObject(red_handle)
                self.object_shapes_handles.append(red_handle)

        # Drop 4 blue cylinders on the bottom half
        for i in range(2):
            for j in range(2):
                new_position = base_position.copy()
                new_position[0] += (j - 0.5) * delta * 2
                new_position[1] -= delta * 1  # Bottom half
                new_position[2] += (i + 0.5) * delta * 2

                blue_handle = self.sim.createPrimitiveShape(self.sim.primitiveshape_cylinder, 
                                                          [blockLength, blockLength, blockLength], 0)

                self.sim.setObjectPosition(blue_handle, -1, new_position)
                self.sim.setShapeColor(blue_handle, None, self.sim.colorcomponent_ambient_diffuse, blue_color)
                self.sim.setEngineFloatParam(self.sim.bullet_body_friction, blue_handle, frictionCube)
                self.sim.setObjectFloatParam(blue_handle, self.sim.shapefloatparam_mass, massOfBlock)
                self.sim.setObjectInt32Param(blue_handle, self.sim.shapeintparam_static, 0)
                self.sim.setObjectInt32Param(blue_handle, self.sim.shapeintparam_respondable, 1)
                self.sim.resetDynamicObject(blue_handle)
                self.object_shapes_handles.append(blue_handle)

        # Wait for objects to settle
        for _ in range(50):
            self.stepSim()
            
        print('Blocks finished dropping in top-bottom configuration')
    
    def action(self, direction=None):
        if direction not in self.directions:
            print(f'Direction: {direction} invalid, please choose one from {self.directions}')
            return
        box_position = self.sim.getObjectPosition(self.boxHandle, self.sim.handle_world)
        _box_position = box_position.copy()
        span = 0.02
        steps = 5
        if direction == 'Up':
            idx = 1
            dirs = [1, -1]
        elif direction == 'Down':
            idx = 1
            dirs = [-1, 1]
        elif direction == 'Right':
            idx = 0
            dirs = [1, -1]
        elif direction == 'Left':
            idx = 0
            dirs = [-1, 1]

        for _dir in dirs:
            for _ in range(steps):
                _box_position[idx] += _dir*span / steps
                self.sim.setObjectPosition(self.boxHandle, self.sim.handle_world, _box_position)
                self.stepSim()
    
    def captureImage(self, label, episode_num, step_num=None):
        """Capture an image from the vision sensor and save it"""
        # Handle the vision sensor
        self.sim.handleVisionSensor(self.vision_sensor_handle)
        
        # Get the image from the vision sensor
        img, res = self.sim.getVisionSensorImg(self.vision_sensor_handle)
        
        # Convert the image format
        img = np.frombuffer(img, dtype=np.uint8).reshape(res[0], res[1], 3)
        img = cv2.flip(img, 0)  # Flip the image vertically
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  # Convert from RGB to BGR for OpenCV
        
        # Create filename
        if step_num is not None:
            filename = f"{self.scenario}_ep{episode_num:04d}_step{step_num:03d}.jpg"
        else:
            filename = f"{self.scenario}_ep{episode_num:04d}_{label}.jpg"
        
        filepath = os.path.join(self.output_dir, label, filename)
        
        # Save the image
        cv2.imwrite(filepath, img)
        return img

    def stepSim(self):
        self.client.step()

    def stopSim(self):
        self.sim.stopSimulation()

    def cleanup(self):
        """Remove all objects and reset the simulation"""
        for handle in self.object_shapes_handles:
            try:
                self.sim.removeObject(handle)
            except:
                pass
        self.object_shapes_handles = []


def main():
    # Settings
    num_episodes = 1000
    num_shakes = 100
    scenarios = ["left_right", "top_bottom"]
    
    for scenario in scenarios:
        print(f"Starting scenario: {scenario}")
        output_dir = f"dataset_{scenario}"
        
        for episode in range(num_episodes):
            print(f'Running episode: {episode + 1}/{num_episodes} for {scenario} scenario')
            
            # Initialize simulation
            env = Simulation(scenario=scenario, output_dir=output_dir)
            
            # Capture initial image (not-mixed)
            initial_img = env.captureImage("not-mixed", episode)
            
            # Shake the box left, right, up, down for the specified number of times
            for shake in range(num_shakes):
                # Alternate between left-right and up-down
                if shake % 2 == 0:
                    env.action(direction="Left")
                else:
                    env.action(direction="Up")
                
                # Step the simulation to let physics settle
                for _ in range(5):
                    env.stepSim()
            
            # Capture final image (well-mixed)
            final_img = env.captureImage("well-mixed", episode)
            
            # Stop and clean up simulation
            env.stopSim()
            
            # Wait for simulation to fully stop
            while env.sim.getSimulationState() != env.sim.simulation_stopped:
                pass
            
            # Clean up objects before next episode
            env.cleanup()
            
            # Give some time between episodes to prevent resource issues
            time.sleep(0.5)
        
        print(f"Completed {num_episodes} episodes for {scenario} scenario")

if __name__ == '__main__':
    main()