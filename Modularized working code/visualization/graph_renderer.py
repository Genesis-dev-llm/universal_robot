"""
Real-time Graph Renderer
Renders scrolling line graphs for data visualization using OpenGL

FIXES:
- Proper OpenGL state management
- Fixed text rendering in 2D overlay
- Better auto-scaling
- Cleaner grid lines
"""

from OpenGL.GL import *
from collections import deque
import numpy as np
import pygame

class GraphRenderer:
    """Renders a scrolling real-time graph"""
    
    def __init__(self, title, x, y, width, height, max_points=200, color=(0, 255, 0), y_range=(0, 1.0)):
        """
        Initialize graph
        
        Args:
            title: Graph title
            x, y: Screen position (top-left)
            width, height: Dimensions
            max_points: History size
            color: Line color (R, G, B)
            y_range: Tuple (min_y, max_y)
        """
        self.title = title
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.max_points = max_points
        self.color = color
        self.y_range = y_range
        
        self.data = deque(maxlen=max_points)
        # Initialize with zeros
        for _ in range(max_points):
            self.data.append(0.0)
            
        self.font = pygame.font.SysFont('monospace', 10)
        
    def add_value(self, value):
        """Add new data point"""
        self.data.append(value)
    
    def render(self):
        """Render the graph"""
        # Save OpenGL state
        glPushAttrib(GL_ALL_ATTRIB_BITS)
        
        # Draw background with transparency
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        glColor4f(0.05, 0.05, 0.05, 0.85)
        glBegin(GL_QUADS)
        glVertex2f(self.x, self.y)
        glVertex2f(self.x + self.width, self.y)
        glVertex2f(self.x + self.width, self.y + self.height)
        glVertex2f(self.x, self.y + self.height)
        glEnd()
        
        # Draw border
        glLineWidth(1.0)
        glColor4f(0.4, 0.4, 0.4, 1.0)
        glBegin(GL_LINE_LOOP)
        glVertex2f(self.x, self.y)
        glVertex2f(self.x + self.width, self.y)
        glVertex2f(self.x + self.width, self.y + self.height)
        glVertex2f(self.x, self.y + self.height)
        glEnd()
        
        # Calculate scaling
        min_y, max_y = self.y_range
        data_min = min(self.data) if self.data else 0
        data_max = max(self.data) if self.data else 1
        
        # Auto-scale if needed
        if data_max > max_y:
            max_y = data_max * 1.1
        
        y_scale = self.height / (max_y - min_y) if (max_y - min_y) > 0 else 1.0
        x_step = self.width / (self.max_points - 1) if self.max_points > 1 else 1.0
        
        # Draw grid lines (horizontal)
        glColor4f(0.2, 0.2, 0.2, 0.5)
        glBegin(GL_LINES)
        # Mid line
        mid_y = self.y + self.height / 2
        glVertex2f(self.x, mid_y)
        glVertex2f(self.x + self.width, mid_y)
        glEnd()
        
        # Draw data line
        glLineWidth(1.5)
        glColor3ub(*self.color)
        glBegin(GL_LINE_STRIP)
        
        for i, val in enumerate(self.data):
            # Clamp value
            clamped_val = max(min_y, min(max_y, val))
            
            px = self.x + i * x_step
            
            # Calculate Y position (remember Y increases DOWN in 2D ortho)
            normalized = (clamped_val - min_y) / (max_y - min_y) if (max_y - min_y) > 0 else 0
            py = (self.y + self.height) - (normalized * self.height)
            
            glVertex2f(px, py)
            
        glEnd()
        
        # Render Title
        self._render_text(self.title, (200, 200, 200), self.x + 5, self.y + 5)
        
        # Render Min/Max labels
        self._render_text(f"{max_y:.2f}", (150, 150, 150), self.x + 5, self.y + 15)
        self._render_text(f"{min_y:.2f}", (150, 150, 150), self.x + 5, self.y + self.height - 15)
        
        # Restore OpenGL state
        glPopAttrib()

    def _render_text(self, text, color, x, y):
        """Helper to render text using pygame"""
        try:
            text_surface = self.font.render(text, True, color)
            text_data = pygame.image.tostring(text_surface, "RGBA", True)
            
            # Save current state
            glPushAttrib(GL_COLOR_BUFFER_BIT)
            
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            
            glRasterPos2f(x, y)
            glDrawPixels(text_surface.get_width(), text_surface.get_height(), 
                        GL_RGBA, GL_UNSIGNED_BYTE, text_data)
            
            glPopAttrib()
        except Exception as e:
            # Silently fail if text rendering doesn't work
            pass