import sys
import math
from pathlib import Path

# Setup Panda3D configuration for offscreen rendering before importing ShowBase
from panda3d.core import loadPrcFileData
loadPrcFileData("", "window-type offscreen")
loadPrcFileData("", "audio-active #f")
loadPrcFileData("", "show-frame-rate-meter #f")
loadPrcFileData("", "gl-debug #f")

from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    Texture, GraphicsOutput, DirectionalLight, AmbientLight, 
    LRotation, LPoint3, LVector3, LColor, WindowProperties, 
    NodePath, LineSegs, CardMaker
)
from PyQt6.QtGui import QImage

class Panda3DRenderer(ShowBase):
    def __init__(self, width=800, height=800):
        # Configure default window/buffer size before initializing ShowBase
        loadPrcFileData("", f"win-size {width} {height}")
        
        # Initialize ShowBase
        super().__init__()
        
        self.win_width = width
        self.win_height = height
        
        # Setup offscreen buffer texture for readback
        self.tex = Texture("offscreen_tex")
        self.win.addRenderTexture(self.tex, GraphicsOutput.RTMCopyRam)
        
        # Load the GLB car model
        model_path = "/Users/deepansadhukhan/Downloads/v9.3/CarConcept.glb"
        self.car = self.loader.loadModel(model_path)
        self.car.reparentTo(self.render)
        
        # Re-orient the car from Y-up (glTF default) to Z-up (Panda3D default)
        # The GLB model already has a -90 pitch internally, so a +90 pitch rotates it to be perfectly upright.
        self.car.set_p(90)
        
        # Enable auto shader generation for proper PBR/lighting rendering
        self.render.set_shader_auto()
        
        # Find wheel and steering nodes
        self.wheel_fl = self.car.find("**/WheelFrontL")
        self.wheel_fr = self.car.find("**/WheelFrontR")
        self.wheel_rl = self.car.find("**/WheelRearL")
        self.wheel_rr = self.car.find("**/WheelRearR")
        self.body = self.car.find("**/BodyUnderside")
        self.steer_col = self.car.find("**/InteriorSteeringCylinder")
        
        # Align all wheels relative to body to (0,0,0) at startup.
        # This fixes the wobbly/tilted wheel spinning bug caused by pre-baked skewed GLB transforms,
        # ensuring the wheel coordinate systems are perfectly aligned with the chassis.
        for wnode in [self.wheel_fl, self.wheel_fr, self.wheel_rl, self.wheel_rr]:
            if not wnode.is_empty() and not self.body.is_empty():
                wnode.set_hpr(self.body, 0, 0, 0)
                
        # Create chassis_body dummy node for suspension sprung mass
        self.chassis_body = self.body.attach_new_node("chassis_body") if not self.body.is_empty() else None
        
        # Reparent all children of body to chassis_body EXCEPT the wheels and chassis_body itself.
        # This allows the chassis body to pitch and roll (suspension travel) while wheels remain on the ground.
        if self.chassis_body is not None:
            for child in list(self.body.get_children()):
                name = child.get_name()
                if name not in ["WheelFrontL", "WheelFrontR", "WheelRearL", "WheelRearR", "chassis_body"]:
                    child.reparent_to(self.chassis_body)
        
        # Cache initial steering column quaternion for local rotations
        if not self.steer_col.is_empty():
            self.steer_col_init_quat = self.steer_col.get_quat()
        else:
            self.steer_col_init_quat = None
            
        # Setup ground and environment
        self._setup_environment()
        self._setup_lighting()
        
        # Camera orbit parameters (Default to classic racing game chase view: behind, slightly elevated, and further back)
        self.cam_distance = 8.5   # meters (further back so the car doesn't cover the whole view)
        self.cam_yaw = 180.0      # degrees (directly behind)
        self.cam_pitch = 12.0     # degrees (looking slightly down to see the road ahead)
        
    def _setup_environment(self):
        # Create a ground plane
        cm = CardMaker("ground")
        cm.set_frame(-250, 250, -250, 250)
        self.ground = self.render.attach_new_node(cm.generate())
        self.ground.set_pos(0, 0, 0)
        self.ground.set_p(-90)  # Make it horizontal
        self.ground.set_color(0.83, 0.85, 0.87, 1.0)  # Clean bright studio floor
        
        # Draw a grid on the ground using LineSegs
        segs = LineSegs()
        segs.set_color(0.2, 0.24, 0.32, 1.0)
        segs.set_thickness(1.2)
        
        grid_range = 100
        step = 2
        for i in range(-grid_range, grid_range + 1, step):
            # Major lines are brighter
            if i % 10 == 0:
                segs.set_color(0.70, 0.72, 0.75, 1.0)
            else:
                segs.set_color(0.76, 0.78, 0.81, 1.0)
            
            segs.move_to(i, -grid_range, 0.005) # slightly above ground to prevent z-fighting
            segs.draw_to(i, grid_range, 0.005)
            segs.move_to(-grid_range, i, 0.005)
            segs.draw_to(grid_range, i, 0.005)
            
        self.grid = self.render.attach_new_node(segs.create())

    def _setup_lighting(self):
        # Ambient light
        ambient = AmbientLight("ambient")
        ambient.set_color((0.65, 0.65, 0.65, 1.0))
        self.ambient_np = self.render.attach_new_node(ambient)
        self.render.set_light(self.ambient_np)
        
        # Directional light (sun)
        sun = DirectionalLight("sun")
        sun.set_color((0.95, 0.95, 0.95, 1.0))
        self.sun_np = self.render.attach_new_node(sun)
        self.sun_np.set_pos(10, -15, 20)
        self.sun_np.look_at(0, 0, 0)
        self.render.set_light(self.sun_np)
        
        # Fill light (secondary directional)
        fill = DirectionalLight("fill")
        fill.set_color((0.35, 0.35, 0.35, 1.0))
        self.fill_np = self.render.attach_new_node(fill)
        self.fill_np.set_pos(-15, 15, -5)
        self.fill_np.look_at(0, 0, 0)
        self.render.set_light(self.fill_np)

    def set_clear_color(self, qcolor):
        # Override 2D clear colors to look like a premium bright studio showroom
        self.win.set_clear_color(LColor(0.88, 0.90, 0.92, 1.0))
        self.ground.set_color(0.83, 0.85, 0.87, 1.0)

    def update_buffer_size(self, w, h):
        # Offscreen buffer size is fixed at startup to avoid GraphicsBuffer requestProperties crashes.
        # Scaling is handled by the QPainter drawing step.
        pass

    def render_3d(self, x, y, heading_deg, steer_deg, roll_left_rad, roll_right_rad, susp_roll_deg=0.0, susp_pitch_deg=0.0):
        # 1. Update vehicle position and heading in 3D world space (Z-up)
        # 2D x maps to 3D X
        # 2D y maps to 3D -Y (since 2D y is positive downwards)
        self.car.set_pos(x, -y, 0)
        # Set heading (yaw). In 2D, th increases CW. In Panda3D Z-up, heading increases CCW.
        # We negate it and add 180 degrees because the GLB model faces negative Y (backwards) by default.
        self.car.set_h(-heading_deg + 180.0)
        
        # 2. Update front wheel steering (rotate around local Z-axis of body, which corresponds to Heading)
        # Positive steer_deg is right, which requires a negative heading offset in body coordinates.
        if not self.wheel_fl.is_empty() and not self.body.is_empty():
            self.wheel_fl.set_hpr(self.body, -steer_deg, 0, 0)
        if not self.wheel_fr.is_empty() and not self.body.is_empty():
            self.wheel_fr.set_hpr(self.body, -steer_deg, 0, 0)
            
        # 3. Update wheel rolling (spin around local X axis of wheels)
        roll_l_deg = math.degrees(roll_left_rad)
        roll_r_deg = math.degrees(roll_right_rad)
        
        # Spin only tyre/rim meshes, leaving calipers/brake pads static.
        # Right wheels (fr, rr) spin in the opposite direction of left wheels (fl, rl) relative to their X axles.
        for wnode, roll_deg in [(self.wheel_fl, roll_l_deg), (self.wheel_rl, roll_l_deg),
                               (self.wheel_fr, -roll_r_deg), (self.wheel_rr, -roll_r_deg)]:
            if not wnode.is_empty():
                for child in wnode.get_children():
                    if "pad" not in child.get_name().lower():
                        child.set_p(roll_deg)
                        
        # 4. Update steering wheel rotation around its column axis (local Y axis in Y-up, which is Roll)
        if not self.steer_col.is_empty() and self.steer_col_init_quat is not None:
            # Negate the angle so steering right rotates the steering wheel CW (clockwise)
            steer_rot_angle = -steer_deg * 2.5  # amplify for visual effect
            # Create local rotation around column Y-axis
            rot = LRotation(LVector3(0, 1, 0), steer_rot_angle)
            # Combine with initial column quat (apply local rotation first)
            self.steer_col.set_quat(self.steer_col_init_quat * rot)
            
        # 4b. Update suspension pitch and roll of the chassis sprung mass
        if self.chassis_body is not None and not self.chassis_body.is_empty():
            self.chassis_body.set_hpr(0, susp_pitch_deg, susp_roll_deg)
            
        # 5. Position chase camera relative to car position & heading
        # Car is facing along -heading_deg + 180.
        # So we align camera yaw relative to this direction.
        total_yaw = -heading_deg + self.cam_yaw
        yaw_rad = math.radians(total_yaw)
        pitch_rad = math.radians(self.cam_pitch)
        
        dx = self.cam_distance * math.sin(yaw_rad) * math.cos(pitch_rad)
        dy = self.cam_distance * math.cos(yaw_rad) * math.cos(pitch_rad)
        dz = self.cam_distance * math.sin(pitch_rad)
        
        # Position camera and point at car (with Z-offset to look at car center)
        self.cam.set_pos(x + dx, -y + dy, dz + 0.4)
        self.cam.look_at(x, -y, 0.4)
        
        # 6. Render frame to texture
        self.graphicsEngine.renderFrame()
        
        # 7. Copy texture RAM into QImage
        if self.tex.mightHaveRamImage():
            self.tex.setFormat(Texture.FRgba32)
            ram_image = self.tex.getRamImage()
            data = ram_image.getData()
            
            # Construct QImage from raw BGRA data
            img = QImage(
                data,
                self.win_width,
                self.win_height,
                QImage.Format.Format_RGBA8888
            ).mirrored(False, True)  # Vertical flip because OpenGL texture coords are bottom-up
            return img
        
        return None
