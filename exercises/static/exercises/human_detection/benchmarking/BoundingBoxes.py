from BoundingBox import *
from utils import *


class BoundingBoxes:
    """Container class for managing collections of bounding boxes."""
    
    def __init__(self):
        self._bounding_boxes = []

    def add_bounding_box(self, bb: BoundingBox) -> None:
        """Add a bounding box to the collection."""
        if not isinstance(bb, BoundingBox):
            raise TypeError("Only BoundingBox objects can be added")
        self._bounding_boxes.append(bb)

    def remove_bounding_box(self, bounding_box: BoundingBox) -> None:
        """Remove a specific bounding box from the collection."""
        self._bounding_boxes = [bb for bb in self._bounding_boxes 
                              if not BoundingBox.compare(bb, bounding_box)]

    def remove_all_bounding_boxes(self) -> None:
        """Clear all bounding boxes from the collection."""
        self._bounding_boxes = []

    @property
    def bounding_boxes(self) -> List[BoundingBox]:
        return self._bounding_boxes

    def get_bounding_boxes_by_class(self, class_id: str) -> List[BoundingBox]:
        """Get all bounding boxes of a specific class."""
        return [bb for bb in self._bounding_boxes if bb.class_id == class_id]

    def get_classes(self) -> List[str]:
        """Get list of all unique class IDs in the collection."""
        return list(set(bb.class_id for bb in self._bounding_boxes))

    def get_bounding_boxes_by_type(self, bb_type: BBType) -> List[BoundingBox]:
        """Get bounding boxes of a specific type (GroundTruth or Detected)."""
        return [bb for bb in self._bounding_boxes if bb.bb_type == bb_type]

    def get_bounding_boxes_by_image_name(self, image_name: str) -> List[BoundingBox]:
        """Get all bounding boxes associated with a specific image."""
        return [bb for bb in self._bounding_boxes if bb.image_name == image_name]

    def count(self, bb_type: Optional[BBType] = None) -> int:
        """Count bounding boxes, optionally filtered by type."""
        if bb_type is None:
            return len(self._bounding_boxes)
        return sum(1 for bb in self._bounding_boxes if bb.bb_type == bb_type)

    def clone(self):
        """Create a deep copy of the BoundingBoxes collection."""
        new_bounding_boxes = BoundingBoxes()
        for bb in self._bounding_boxes:
            new_bounding_boxes.add_bounding_box(BoundingBox.clone(bb))
        return new_bounding_boxes

    def draw_all_bounding_boxes(
        self, 
        image: np.ndarray, 
        image_name: str,
        gt_color: Tuple[int, int, int] = (0, 255, 0),
        det_color: Tuple[int, int, int] = (255, 0, 0),
        thickness: int = 2
    ) -> np.ndarray:
        """Draw all bounding boxes for a specific image."""
        bboxes = self.get_bounding_boxes_by_image_name(image_name)
        for bb in bboxes:
            color = gt_color if bb.bb_type == BBType.GroundTruth else det_color
            label = f"{bb.class_id} {bb.confidence:.2f}" if bb.confidence is not None else bb.class_id
            image = add_bb_into_image(image, bb, color=color, thickness=thickness, label=label)
        return image
