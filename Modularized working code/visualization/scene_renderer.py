"""
OpenGL 3D scene rendering
Draws grid, axes, and IMU cube

CHANGES:
- Fixed cube rotation order to ZYX (yaw, pitch, roll)
- Cube now rotates exactly as hand does
- Rotation axes aligned with control axes
"""

from OpenGL.GL import *
import math

class SceneRenderer:
    """
    Handles 3D scene rendering for visualization
    """
    
    
    _ur10_model = None
    
    @classmethod
    def initialize(cls):
        """Initialize renderer resources (must correspond to GL context)"""
        from visualization.ur10_model import UR10Model
        cls._ur10_model = UR10Model()
    
    @staticmethod
    def draw_grid():
        """Draw 3D reference grid"""
        glBegin(GL_LINES)
        glColor3f(0.3, 0.3, 0.3)
        for i in range(-10, 11):
            glVertex3f(i, -2, -10)
            glVertex3f(i, -2, 10)
            glVertex3f(-10, -2, i)
            glVertex3f(10, -2, i)
        glEnd()
    
    @classmethod
    def draw_robot(cls, joint_angles):
        """
        Draw UR10 robot model
        
        Args:
            joint_angles: List of 6 joint angles in radians
        """
        if cls._ur10_model:
            cls._ur10_model.render(joint_angles)
        else:
            # Fallback if not initialized
            pass

    @staticmethod
    def draw_axes(length=2.0):
        """Draw RGB coordinate axes"""
        glLineWidth(3.0)
        glBegin(GL_LINES)
        # X axis - Red
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0, 0, 0)
        glVertex3f(length, 0, 0)
        # Y axis - Green
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, length, 0)
        # Z axis - Blue
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, length)
        glEnd()
        glLineWidth(1.0)
    
    @staticmethod
    def draw_cube(color_multiplier=(1, 1, 1)):
        """
        Draw colored cube with edges representing IMU orientation
        
        CHANGED: Rotation order fixed - cube now rotates exactly as hand
        """
        # Define vertices relative to the IMU sensor (1 unit = 1 dimension)
        vertices = (
            (1, -1, 1), (1, 1, 1), (-1, 1, 1), (-1, -1, 1),     # Front Face (Z+)
            (1, -1, -1), (1, 1, -1), (-1, 1, -1), (-1, -1, -1)  # Back Face (Z-)
        )
        
        faces = (
            (0, 1, 2, 3),  # Front
            (4, 7, 6, 5),  # Back
            (0, 4, 5, 1),  # Right
            (2, 6, 7, 3),  # Left
            (1, 5, 6, 2),  # Top
            (0, 3, 7, 4)   # Bottom
        )
        
        colors = (
            (1.0, 0.2, 0.2),  # Front (Reddish) - Forward
            (0.2, 0.2, 1.0),  # Back (Blueish)
            (0.2, 1.0, 0.2),  # Right (Greenish)
            (1.0, 1.0, 0.2),  # Left (Yellowish)
            (1.0, 0.2, 1.0),  # Top (Magenta)
            (0.2, 1.0, 1.0)   # Bottom (Cyan)
        )
        
        # Draw faces
        glBegin(GL_QUADS)
        for i, face in enumerate(faces):
            final_color = tuple(c * m for c, m in zip(colors[i], color_multiplier))
            glColor3fv(final_color)
            for vertex_idx in face:
                glVertex3fv(vertices[vertex_idx])
        glEnd()
        
        # Draw edges for definition
        glColor3f(0.1, 0.1, 0.1)
        glLineWidth(2.0)
        glBegin(GL_LINES)
        edges = ((0,1), (1,2), (2,3), (3,0), (4,5), (5,6), (6,7), (7,4), 
                 (0,4), (1,5), (2,6), (3,7))
        for edge in edges:
            for vertex in edge:
                glVertex3fv(vertices[vertex])
        glEnd()
        glLineWidth(1.0)
    
    @staticmethod
    def draw_workspace_limit(radius=1.3, segments=32):
        """
        Draw robot workspace boundary (wireframe sphere)
        
        Args:
            radius: Radius in meters (UR10 = 1.3m relative to our scale)
            segments: Number of segments for circles
        """
        glColor4f(0.2, 0.2, 0.3, 0.3) # Transparent dark blue
        glLineWidth(1.0)
        
        # Draw 3 circles to represent sphere bounds
        
        # XY Plane Circle
        glBegin(GL_LINE_LOOP)
        for i in range(segments):
            angle = 2 * math.pi * i / segments
            glVertex3f(math.cos(angle) * radius, math.sin(angle) * radius, 0)
        glEnd()
        
        # XZ Plane Circle
        glBegin(GL_LINE_LOOP)
        for i in range(segments):
            angle = 2 * math.pi * i / segments
            glVertex3f(math.cos(angle) * radius, 0, math.sin(angle) * radius)
        glEnd()
        
        # YZ Plane Circle
        glBegin(GL_LINE_LOOP)
        for i in range(segments):
            angle = 2 * math.pi * i / segments
            glVertex3f(0, math.cos(angle) * radius, math.sin(angle) * radius)
        glEnd()
        
    @staticmethod
    def render_cube_at_pose(position, quaternion, color_multiplier=(1, 1, 1)):
        """
        Render cube at specific position with quaternion rotation
        
        CHANGED: Fixed rotation application to use proper Euler conversion
        
        Args:
            position: [x, y, z] position
            quaternion: [qi, qj, qk, qr] orientation
            color_multiplier: RGB color scaling tuple
        """
        glPushMatrix()
        glTranslatef(position[0], position[1], position[2])
        
        # CHANGED: Convert quaternion to Euler angles properly
        # Extract quaternion components
        qi, qj, qk, qr = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        
        # Convert to Euler angles (ZYX convention)
        # Roll (X-axis rotation)
        sinr_cosp = 2 * (qr * qi + qj * qk)
        cosr_cosp = 1 - 2 * (qi**2 + qj**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (Y-axis rotation)
        sinp = 2 * (qr * qj - qk * qi)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (Z-axis rotation)
        siny_cosp = 2 * (qr * qk + qi * qj)
        cosy_cosp = 1 - 2 * (qj**2 + qk**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # CHANGED: Apply rotations in ZYX order (yaw, pitch, roll)
        # This makes the cube rotate exactly as your hand does
        glRotatef(math.degrees(yaw), 0, 0, 1)      # Yaw around Z
        glRotatef(math.degrees(pitch), 0, 1, 0)    # Pitch around Y
        glRotatef(math.degrees(roll), 1, 0, 0)     # Roll around X
        
        SceneRenderer.draw_cube(color_multiplier)
        glPopMatrix()