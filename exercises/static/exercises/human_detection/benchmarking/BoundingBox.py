from utils import *


class BoundingBox:
    """Class representing a bounding box with various attributes and conversion methods."""
    
    def __init__(
        self,
        image_name: str,
        class_id: str,
        x: float,
        y: float,
        w: float,
        h: float,
        type_coordinates: CoordinatesType = CoordinatesType.Absolute,
        img_size: Optional[Tuple[int, int]] = None,
        bb_type: BBType = BBType.GroundTruth,
        class_confidence: Optional[float] = None,
        format: BBFormat = BBFormat.XYWH
    ):
        """Initialize bounding box with given parameters."""
        self._image_name = image_name
        self._type_coordinates = type_coordinates
        
        if type_coordinates == CoordinatesType.Relative and img_size is None:
            raise ValueError("Image size is required for relative coordinates")
        if bb_type == BBType.Detected and class_confidence is None:
            raise ValueError("Class confidence is required for detections")
        
        self._class_confidence = class_confidence
        self._bb_type = bb_type
        self._class_id = class_id
        self._format = format
        
        # Convert relative to absolute coordinates if needed
        if type_coordinates == CoordinatesType.Relative:
            self._x, self._y, self._w, self._h = convert_to_absolute_values(
                img_size, (x, y, w, h))
            self._width_img, self._height_img = img_size
            
            if format == BBFormat.XYWH:
                self._x2 = self._w
                self._y2 = self._h
                self._w = self._x2 - self._x
                self._h = self._y2 - self._y
            else:
                raise ValueError("For relative coordinates, format must be XYWH")
        else:
            self._x = x
            self._y = y
            if format == BBFormat.XYWH:
                self._w = w
                self._h = h
                self._x2 = self._x + self._w
                self._y2 = self._y + self._h
            else:  # XYX2Y2 format
                self._x2 = w
                self._y2 = h
                self._w = self._x2 - self._x
                self._h = self._y2 - self._y
        
        self._width_img, self._height_img = img_size if img_size else (None, None)

    def get_absolute_bounding_box(self, format: BBFormat = BBFormat.XYWH) -> Tuple[float, float, float, float]:
        """Get bounding box coordinates in specified format."""
        if format == BBFormat.XYWH:
            return (self._x, self._y, self._w, self._h)
        elif format == BBFormat.XYX2Y2:
            return (self._x, self._y, self._x2, self._y2)
        raise ValueError(f"Unknown format: {format}")

    def get_relative_bounding_box(self, img_size: Optional[Tuple[int, int]] = None) -> Tuple[float, float, float, float]:
        """Convert bounding box to relative coordinates."""
        if img_size is None and (self._width_img is None or self._height_img is None):
            raise ValueError("Image size is required for conversion")
        
        target_size = img_size if img_size is not None else (self._width_img, self._height_img)
        return convert_to_relative_values(target_size, (self._x, self._x2, self._y, self._y2))

    @property
    def image_name(self) -> str:
        return self._image_name

    @property
    def confidence(self) -> Optional[float]:
        return self._class_confidence

    @property
    def format(self) -> BBFormat:
        return self._format

    @property
    def class_id(self) -> str:
        return self._class_id

    @property
    def image_size(self) -> Tuple[Optional[int], Optional[int]]:
        return (self._width_img, self._height_img)

    @property
    def coordinates_type(self) -> CoordinatesType:
        return self._type_coordinates

    @property
    def bb_type(self) -> BBType:
        return self._bb_type

    @staticmethod
    def compare(det1, det2) -> bool:
        """Compare two bounding boxes for equality."""
        det1_bb = det1.get_absolute_bounding_box()
        det1_img_size = det1.image_size
        det2_bb = det2.get_absolute_bounding_box()
        det2_img_size = det2.image_size

        return (det1.class_id == det2.class_id and
                det1.confidence == det2.confidence and
                det1_bb[0] == det2_bb[0] and
                det1_bb[1] == det2_bb[1] and
                det1_bb[2] == det2_bb[2] and
                det1_bb[3] == det2_bb[3] and
                det1_img_size[0] == det2_img_size[0] and
                det1_img_size[1] == det2_img_size[1])

    @staticmethod
    def clone(bounding_box):
        """Create a deep copy of the bounding box."""
        abs_bb = bounding_box.get_absolute_bounding_box(format=BBFormat.XYWH)
        return BoundingBox(
            bounding_box.image_name,
            bounding_box.class_id,
            abs_bb[0],
            abs_bb[1],
            abs_bb[2],
            abs_bb[3],
            type_coordinates=bounding_box.coordinates_type,
            img_size=bounding_box.image_size,
            bb_type=bounding_box.bb_type,
            class_confidence=bounding_box.confidence,
            format=BBFormat.XYWH
        )
