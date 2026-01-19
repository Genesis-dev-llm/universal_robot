"""
UR5e Robot Model Visualizer
Renders a 3D model of the UR5e robot using OpenGL
"""

from OpenGL.GL import *
from OpenGL.GLU import *
import math
import numpy as np

class UR5eModel:
    """
    Renders a UR5e robot arm based on joint angles
    """
    
    def __init__(self):
        # UR5e DH Parameters (Standard DH Convention)
        # s: http://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
        # [d, a, alpha]
        self.dh_params = [
            # d, a, alpha
            [0.1625, 0, math.pi/2],       # Joint 1 (Base)
            [0, -0.425, 0],               # Joint 2 (Shoulder)
            [0, -0.3922, 0],              # Joint 3 (Elbow)
            [0.1333, 0, math.pi/2],       # Joint 4 (Wrist 1)
            [0.0997, 0, -math.pi/2],      # Joint 5 (Wrist 2)
            [0.0996, 0, 0]                # Joint 6 (Wrist 3)
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
        gluCylinder(self.quadric, 0.08, 0.08, 0.05, 32, 1) # Slightly smaller base for UR5e
        glPopMatrix()
        
        # Iterate through DH transforms
        for i in range(6):
            theta = joint_angles[i]
            d, a, alpha = self.dh_params[i]
            color = self.link_colors[i]
            
            glColor3f(*color)
            
            # 1. Rotate around Z (theta)
            glRotatef(math.degrees(theta), 0, 0, 1)
            
            # Draw Joint Cylinder (z-axis)
            glPushMatrix()
            gluSphere(self.quadric, 0.05, 16, 16) # Smaller joints for UR5e
            glPopMatrix()
            
            # 2. Translate along Z (d)
            if abs(d) > 0.001:
                self._draw_cylinder_z(0.04, abs(d), color)
                glTranslatef(0, 0, d)
            
            # 3. Translate along X (a)
            if abs(a) > 0.001:
                glPushMatrix()
                glRotatef(90, 0, 1, 0) # Rotate to draw along X
                self._draw_cylinder_z(0.04, abs(a), color)
                glPopMatrix()
                glTranslatef(a, 0, 0) # a is negative, translate along negative X
            
            # 4. Rotate around X (alpha)
            glRotatef(math.degrees(alpha), 1, 0, 0)
            
        # Draw TCP
        glColor3f(1.0, 0.0, 0.0)
        gluSphere(self.quadric, 0.03, 16, 16)
        
        # Draw TCP axes
        glDisable(GL_LIGHTING)
        glLineWidth(2.0)
        glBegin(GL_LINES)
        glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(0.1, 0, 0)
        glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, 0.1, 0)
        glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, 0.1)
        glEnd()
        
        glPopMatrix()
        glPopAttrib()

    def _draw_cylinder_z(self, radius, height, color):
        """Draw a cylinder along the Z axis"""
        glPushMatrix()
        glColor3f(*color)
        gluCylinder(self.quadric, radius, radius, height, 16, 1)
        glPopMatrix()
