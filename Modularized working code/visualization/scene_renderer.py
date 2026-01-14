"""
OpenGL 3D scene rendering
Draws grid, axes, and IMU cube
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
        """Draw colored cube with edges representing IMU orientation"""
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
        
        Args:
            position: [x, y, z] position
            quaternion: [qi, qj, qk, qr] orientation
            color_multiplier: RGB color scaling tuple
        """
        glPushMatrix()
        glTranslatef(position[0], position[1], position[2])
        
        # Apply quaternion rotation
        q = quaternion
        angle = 2 * math.acos(min(1.0, max(-1.0, q[3])))
        
        if angle > 0.001:
            sin_half = math.sin(angle / 2)
            if abs(sin_half) > 0.001:
                axis = q[:3] / sin_half
                glRotatef(math.degrees(angle), axis[0], axis[1], axis[2])
        
        SceneRenderer.draw_cube(color_multiplier)
        glPopMatrix()