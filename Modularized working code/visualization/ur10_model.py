"""
UR10 Robot Model Visualizer
Renders a 3D model of the UR10 robot using OpenGL
"""

from OpenGL.GL import *
from OpenGL.GLU import *
import math
import numpy as np

class UR10Model:
    """
    Renders a UR10 robot arm based on joint angles
    """
    
    def __init__(self):
        # UR10 DH Parameters (Standard)
        # d: offset along z
        # a: length along x
        # alpha: twist around x
        self.dh_params = [
            # d, a, alpha
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
            
        glPushMatrix()
        
        # Scale down to fit visualizer (1 meter = 1 unit usually, but check camera)
        # Our previous visualizer used a 20 distance camera, so 1:1 scale might be fine.
        # But let's verify visual scale compared to the 2.0 unit cube.
        # UR10 reach is ~1.3m. So 1:1 scale is good.
        
        # Draw Base Pedestal
        glColor3f(0.1, 0.1, 0.1)
        glPushMatrix()
        glRotatef(-90, 1, 0, 0)
        gluCylinder(self.quadric, 0.1, 0.1, 0.05, 32, 1)
        glPopMatrix()
        
        # Iterate through transforms
        for i in range(6):
            theta = joint_angles[i]
            d, a, alpha = self.dh_params[i]
            color = self.link_colors[i]
            
            glColor3f(*color)
            
            # 1. Rotate around Z (theta)
            glRotatef(math.degrees(theta), 0, 0, 1)
            
            # Draw Joint Cylinder (z-axis)
            glPushMatrix()
            glTranslatef(0, 0, -d/2 if d > 0 else -0.05)
            # gluCylinder(self.quadric, 0.06, 0.06, max(abs(d), 0.1), 16, 1)
            # Simplified joint visualization
            gluSphere(self.quadric, 0.07, 16, 16)
            glPopMatrix()
            
            # 2. Translate along Z (d)
            glTranslatef(0, 0, d)
            
            # Draw Link (x-axis)
            if abs(a) > 0.001:
                glPushMatrix()
                glRotatef(90, 0, 1, 0) # Rotate to draw along X
                gluCylinder(self.quadric, 0.05, 0.05, abs(a), 16, 1) # Note: this draws along +Z usually
                # We need to draw from 0 to a along X.
                # Since gluCylinder draws along +Z, we rotated Y by 90.
                # But 'a' is negative for UR10 standard DH.
                # So we draw length abs(a).
                # Check direction: 'a' is along X_new (after Z rotation/translation? No, DH convention is specific)
                # Standard DH: Translate d along Z_i-1, Rotate theta around Z_i-1, Translate a along X_i, Rotate alpha around X_i
                
                # We already did Rotate theta, Translate d. Now we are at Frame i (almost).
                # Actually sequence:
                # 1. Rot_z(theta)
                # 2. Trans_z(d)
                # 3. Trans_x(a)  <-- Draw link here
                # 4. Rot_x(alpha)
                glPopMatrix()
                
                # Draw visual link along X
                glPushMatrix()
                if a < 0:
                    glRotatef(-90, 0, 1, 0)
                else:
                    glRotatef(90, 0, 1, 0)
                gluCylinder(self.quadric, 0.04, 0.04, abs(a), 16, 1)
                glPopMatrix()

            # 3. Translate along X (a)
            glTranslatef(a, 0, 0)
            
            # 4. Rotate around X (alpha)
            glRotatef(math.degrees(alpha), 1, 0, 0)
            
        # Draw TCP
        glColor3f(1.0, 0.0, 0.0)
        gluSphere(self.quadric, 0.03, 16, 16)
        
        glPopMatrix()
