from os import path
from BoundingBox import BoundingBox
from BoundingBoxes import BoundingBoxes
from utils import *
from Evaluator import *

def get_bounding_boxes(
    gt_folder: str = 'groundtruths',
    det_folder: str = 'detections',
    gt_format: BBFormat = BBFormat.XYWH,
    det_format: BBFormat = BBFormat.XYWH
) -> BoundingBoxes:
    """
    Read ground truth and detection bounding boxes from text files.
    
    Args:
        gt_folder: Folder containing ground truth text files
        det_folder: Folder containing detection text files
        gt_format: Format of ground truth bounding boxes
        det_format: Format of detection bounding boxes
        
    Returns:
        BoundingBoxes object containing all loaded boxes
    """
    all_bounding_boxes = BoundingBoxes()
    
    # Read ground truths
    if not os.path.isdir(gt_folder):
        raise FileNotFoundError(f"Ground truth folder not found: {gt_folder}")
    
    gt_files = sorted(glob.glob(os.path.join(gt_folder, "*.txt")))
    
    for gt_file in gt_files:
        image_name = os.path.splitext(os.path.basename(gt_file))[0]
        
        with open(gt_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                
                parts = line.split()
                if len(parts) < 5:
                    continue
                
                class_id = parts[0]
                coords = list(map(float, parts[1:5]))
                
                bb = BoundingBox(
                    image_name,
                    class_id,
                    *coords,
                    type_coordinates=CoordinatesType.Absolute,
                    img_size=None,
                    bb_type=BBType.GroundTruth,
                    format=gt_format
                )
                all_bounding_boxes.add_bounding_box(bb)
    
    # Read detections
    if not os.path.isdir(det_folder):
        raise FileNotFoundError(f"Detection folder not found: {det_folder}")
    
    det_files = sorted(glob.glob(os.path.join(det_folder, "*.txt")))
    
    for det_file in det_files:
        image_name = os.path.splitext(os.path.basename(det_file))[0]
        
        with open(det_file, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                
                parts = line.split()
                if len(parts) < 6:
                    continue
                
                class_id = parts[0]
                confidence = float(parts[1])
                coords = list(map(float, parts[2:6]))
                
                bb = BoundingBox(
                    image_name,
                    class_id,
                    *coords,
                    type_coordinates=CoordinatesType.Absolute,
                    img_size=None,
                    bb_type=BBType.Detected,
                    class_confidence=confidence,
                    format=det_format
                )
                all_bounding_boxes.add_bounding_box(bb)
    
    return all_bounding_boxes


def main():
    """Example usage of the evaluation framework."""
    # Load bounding boxes
    try:
        bounding_boxes = get_bounding_boxes()
    except Exception as e:
        print(f"Error loading bounding boxes: {e}")
        return
    
    # Initialize evaluator
    evaluator = Evaluator()
    
    # Calculate metrics
    results = evaluator.get_pascal_voc_metrics(bounding_boxes)
    
    # Print results
    print("\nEvaluation Results:")
    print("-" * 50)
    for result in results:
        print(f"Class: {result['class']}")
        print(f"AP: {result['AP']:.4f}")
        print(f"Total Positives: {result['total positives']}")
        print(f"True Positives: {result['total TP']}")
        print(f"False Positives: {result['total FP']}")
        print("-" * 50)
    
    # Plot precision-recall curves
    evaluator.plot_precision_recall_curve(
        bounding_boxes,
        save_path='output_plots',
        show_graphic=True
    )


if __name__ == "__main__":
    main()
