"""
UR10 Robot Model Visualizer
Renders a 3D model of the UR10 robot using OpenGL

FIXES:
- Corrected DH parameter implementation
- Fixed link drawing direction
- Better visual representation
- Proper transformation sequence
"""

from OpenGL.GL import *
from OpenGL.GLU import *
import math
import numpy as np

class UR10Model:
    """Renders a UR10 robot arm based on joint angles"""
    
    def __init__(self):
        # UR10 DH Parameters (Standard DH Convention)
        # [d, a, alpha] where:
        #   d: offset along previous Z
        #   a: length along X
        #   alpha: twist around X
        self.dh_params = [
            [0.1273, 0, math.pi/2],       # Joint 1 (Base)
            [0, -0.612, 0],               # Joint 2 (Shoulder)
            [0, -0.5723, 0],              # Joint 3 (Elbow)
            [0.163941, 0, math.pi/2],     # Joint 4 (Wrist 1)
            [0.1157, 0, -math.pi/2],      # Joint 5 (Wrist 2)
            [0.0922, 0, 0]                # Joint 6 (Wrist 3)
        ]
        
        # Colors for each link (Base to Tip)
        self.link_colors = [
            (0.2, 0.2, 0.2),  # Base (Dark Grey)
            (0.5, 0.5, 0.5),  # Shoulder (Grey)
            (0.5, 0.5, 0.5),  # Elbow (Grey)
            (0.3, 0.6, 0.8),  # Wrist 1 (Light Blue)
            (0.3, 0.6, 0.8),  # Wrist 2 (Light Blue)
            (0.7, 0.7, 0.7),  # Wrist 3 (Silver)
        ]
        
        self.quadric = gluNewQuadric()
        gluQuadricNormals(self.quadric, GLU_SMOOTH)
    
    def render(self, joint_angles):
        """
        Render the robot arm with the given joint angles
        
        Args:
            joint_angles: List of 6 joint angles in radians
        """
        if len(joint_angles) != 6:
            return
        
        # Save state
        glPushAttrib(GL_ALL_ATTRIB_BITS)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # Set light position
        glLightfv(GL_LIGHT0, GL_POSITION, [5, 5, 10, 1])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.3, 0.3, 0.3, 1])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.7, 0.7, 0.7, 1])
        
        glPushMatrix()
        
        # Draw Base Pedestal
        glColor3f(0.15, 0.15, 0.15)
        glPushMatrix()
        glRotatef(-90, 1, 0, 0)
        gluCylinder(self.quadric, 0.12, 0.12, 0.08, 32, 1)
        glPopMatrix()
        
        # Iterate through DH transforms
        for i in range(6):
            theta = joint_angles[i]
            d, a, alpha = self.dh_params[i]
            color = self.link_colors[i]
            
            glColor3f(*color)
            
            # DH Transformation Sequence (Standard):
            # 1. Rotate around Z by theta
            glRotatef(math.degrees(theta), 0, 0, 1)
            
            # Draw joint sphere at current position
            glPushMatrix()
            gluSphere(self.quadric, 0.06, 16, 16)
            glPopMatrix()
            
            # 2. Translate along Z by d
            if abs(d) > 0.01:
                self._draw_cylinder_z(0.04, abs(d), color)
                glTranslatef(0, 0, d)
            
            # 3. Translate along X by a
            if abs(a) > 0.01:
                # Draw link along X
                glPushMatrix()
                glRotatef(90, 0, 1, 0)  # Rotate to align cylinder with X
                length = abs(a)
                self._draw_cylinder_z(0.04, length, color)
                glPopMatrix()
                glTranslatef(a, 0, 0)
            
            # 4. Rotate around X by alpha
            glRotatef(math.degrees(alpha), 1, 0, 0)
        
        # Draw TCP (Tool Center Point)
        glColor3f(1.0, 0.2, 0.2)
        gluSphere(self.quadric, 0.04, 16, 16)
        
        # Draw TCP axes
        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        glBegin(GL_LINES)
        # X - Red
        glColor3f(1, 0, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(0.1, 0, 0)
        # Y - Green
        glColor3f(0, 1, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0.1, 0)
        # Z - Blue
        glColor3f(0, 0, 1)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 0.1)
        glEnd()
        
        glPopMatrix()
        
        # Restore state
        glPopAttrib()
    
    def _draw_cylinder_z(self, radius, height, color):
        """Draw a cylinder along the Z axis"""
        glPushMatrix()
        glColor3f(*color)
        gluCylinder(self.quadric, radius, radius, height, 16, 1)
        glPopMatrix()