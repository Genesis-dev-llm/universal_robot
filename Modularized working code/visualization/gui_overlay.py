"""
GUI overlay rendering
Status text and speed indicators

UPDATED: Added gripper status display
"""

import pygame
from OpenGL.GL import *

class GUIOverlay:
    """
    Renders 2D GUI elements over 3D scene
    Now includes gripper status
    """
    
    def __init__(self, font_name='monospace', font_size=14):
        """
        Initialize GUI overlay
        
        Args:
            font_name: Font name for text rendering
            font_size: Font size in points
        """
        pygame.font.init()
        self.font = pygame.font.SysFont(font_name, font_size)
    
    def render_text(self, text, color, x, y):
        """
        Render text at specified position
        
        Args:
            text: Text string to render
            color: RGB color tuple (0-255)
            x, y: Screen position
        """
        text_surface = self.font.render(text, True, color)
        text_data = pygame.image.tostring(text_surface, "RGBA", True)
        
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        glRasterPos2f(x, y)
        glDrawPixels(text_surface.get_width(), text_surface.get_height(), 
                    GL_RGBA, GL_UNSIGNED_BYTE, text_data)
        
        glDisable(GL_BLEND)
    
    def draw_status_text(self, status_lines, display_size):
        """
        Draw status text lines
        
        Args:
            status_lines: List of status strings to display
            display_size: Tuple (width, height) of display
        """
        y_offset = 10
        
        for i, line in enumerate(status_lines):
            # Color coding based on content
            if "ERROR" in line or "NOT READY" in line or "BLOCKED" in line or "WARNING" in line or "DISCONNECTED" in line:
                color = (255, 0, 0)  # Red
            elif "READY" in line or "Connected" in line:
                color = (0, 255, 0)  # Green
            elif "DISABLED" in line:
                color = (200, 200, 0)  # Yellow
            else:
                color = (255, 255, 255)  # White
            
            self.render_text(line, color, 10, y_offset + i * 20)
    
    def setup_2d_projection(self, display_size):
        """
        Setup OpenGL for 2D rendering
        
        Args:
            display_size: Tuple (width, height) of display
        """
        glMatrixMode(GL_PROJECTION)
        glPushMatrix()
        glLoadIdentity()
        glOrtho(0, display_size[0], display_size[1], 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        glLoadIdentity()
        glDisable(GL_DEPTH_TEST)
    
    def restore_3d_projection(self):
        """Restore 3D projection after 2D rendering"""
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glPopMatrix()
        glMatrixMode(GL_MODELVIEW)
        glPopMatrix()