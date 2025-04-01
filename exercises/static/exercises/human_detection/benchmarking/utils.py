from enum import Enum
import os
import sys
from typing import List, Dict, Tuple, Optional, Union
import glob
import cv2
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
sns.set()


class MethodAveragePrecision(Enum):
    """Enum for different methods of calculating average precision."""
    EveryPointInterpolation = 1
    ElevenPointInterpolation = 2


class CoordinatesType(Enum):
    """Enum representing if coordinates are relative or absolute."""
    Relative = 1
    Absolute = 2


class BBType(Enum):
    """Enum representing if bounding box is ground truth or detection."""
    GroundTruth = 1
    Detected = 2


class BBFormat(Enum):
    """Enum representing bounding box format (XYWH or XYX2Y2)."""
    XYWH = 1
    XYX2Y2 = 2


def convert_to_relative_values(size: Tuple[int, int], box: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    """Convert absolute coordinates to relative values (0-1 range)."""
    if not all(isinstance(x, (int, float)) for x in box):
        raise ValueError("Box coordinates must be numeric")
    if size[0] <= 0 or size[1] <= 0:
        raise ValueError("Image size must be positive")

    dw = 1.0 / size[0]
    dh = 1.0 / size[1]
    cx = (box[1] + box[0]) / 2.0
    cy = (box[3] + box[2]) / 2.0
    w = box[1] - box[0]
    h = box[3] - box[2]
    x = cx * dw
    y = cy * dh
    w = w * dw
    h = h * dh
    return (x, y, w, h)


def convert_to_absolute_values(size: Tuple[int, int], box: Tuple[float, float, float, float]) -> Tuple[int, int, int, int]:
    """Convert relative coordinates back to absolute pixel values."""
    if not all(0 <= x <= 1 for x in box[2:4]):
        raise ValueError("Relative width and height must be between 0 and 1")
    
    x_in = round(((2 * float(box[0]) - float(box[2])) * size[0] / 2))
    y_in = round(((2 * float(box[1]) - float(box[3])) * size[1] / 2)
    x_end = x_in + round(float(box[2]) * size[0])
    y_end = y_in + round(float(box[3]) * size[1])
    
    # Clip values to image boundaries
    x_in = max(0, x_in)
    y_in = max(0, y_in)
    x_end = min(size[0] - 1, x_end)
    y_end = min(size[1] - 1, y_end)
    
    return (x_in, y_in, x_end, y_end)


def add_bb_into_image(
    image: np.ndarray,
    bb,
    color: Tuple[int, int, int] = (255, 0, 0),
    thickness: int = 2,
    label: Optional[str] = None,
    font_scale: float = 0.5,
    font_thickness: int = 1
) -> np.ndarray:
    """Add bounding box to image with optional label."""
    if not isinstance(image, np.ndarray):
        raise TypeError("Image must be a numpy array")
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Image must be 3-channel BGR")
    
    r, g, b = int(color[0]), int(color[1]), int(color[2])
    font = cv2.FONT_HERSHEY_SIMPLEX

    x1, y1, x2, y2 = bb.getAbsoluteBoundingBox(BBFormat.XYX2Y2)
    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
    cv2.rectangle(image, (x1, y1), (x2, y2), (b, g, r), thickness)
    
    if label is not None:
        # Get text size
        (tw, th), _ = cv2.getTextSize(label, font, font_scale, font_thickness)
        
        # Text position
        text_x = x1 + thickness
        text_y = y1 - th + int(12.5 * font_scale)
        
        # Adjust if text would go above image
        if text_y - th <= 0:
            text_y = y1 + th
            
        # Background rectangle coordinates
        bg_x1 = x1 - int(thickness / 2)
        bg_y1 = y1 - th - int(thickness / 2)
        bg_x2 = bg_x1 + tw + thickness * 3
        bg_y2 = bg_y1 + th + int(12.5 * font_scale)
        
        # Draw background and text
        cv2.rectangle(image, (bg_x1, bg_y1), (bg_x2, bg_y2), (b, g, r), -1)
        cv2.putText(image, label, (text_x, text_y), font, font_scale, 
                   (0, 0, 0), font_thickness, cv2.LINE_AA)
    
    return image
