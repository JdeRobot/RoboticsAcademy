import os
import sys
from collections import Counter

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
sns.set()


from BoundingBox import *
from BoundingBoxes import *
from utils import *


class Evaluator:
    """Class for evaluating object detection performance using Pascal VOC metrics."""
    
    def get_pascal_voc_metrics(
        self,
        bounding_boxes: BoundingBoxes,
        iou_threshold: float = 0.5,
        method: MethodAveragePrecision = MethodAveragePrecision.EveryPointInterpolation
    ) -> List[Dict]:
        """
        Calculate Pascal VOC evaluation metrics for object detection.
        
        Args:
            bounding_boxes: Collection of ground truth and detected bounding boxes
            iou_threshold: IoU threshold for considering a detection as correct
            method: Method for calculating average precision
            
        Returns:
            List of dictionaries containing metrics for each class
        """
        if not isinstance(bounding_boxes, BoundingBoxes):
            raise TypeError("bounding_boxes must be a BoundingBoxes instance")
        if not 0 <= iou_threshold <= 1:
            raise ValueError("iou_threshold must be between 0 and 1")
            
        ret = []
        ground_truths = []
        detections = []
        
        # Separate ground truths and detections
        for bb in bounding_boxes.bounding_boxes:
            if bb.bb_type == BBType.GroundTruth:
                ground_truths.append([
                    bb.image_name,
                    bb.class_id,
                    1,  # confidence for ground truth is always 1
                    bb.get_absolute_bounding_box(BBFormat.XYX2Y2)
                ])
            else:
                detections.append([
                    bb.image_name,
                    bb.class_id,
                    bb.confidence,
                    bb.get_absolute_bounding_box(BBFormat.XYX2Y2)
                ])
        
        classes = sorted(set(bb.class_id for bb in bounding_boxes.bounding_boxes))
        
        # Calculate metrics for each class
        for class_id in classes:
            # Get detections and ground truths for this class
            class_detections = [d for d in detections if d[1] == class_id]
            class_detections = sorted(class_detections, key=lambda x: x[2], reverse=True)
            
            # Create dictionary of ground truths per image
            class_gts = {}
            n_pos = 0
            for gt in ground_truths:
                if gt[1] == class_id:
                    n_pos += 1
                    class_gts[gt[0]] = class_gts.get(gt[0], []) + [gt]
            
            # Initialize true positives and false positives
            tp = np.zeros(len(class_detections))
            fp = np.zeros(len(class_detections))
            
            # Create dictionary to track which ground truths have been detected
            detected_gt = {img_name: np.zeros(len(gts)) for img_name, gts in class_gts.items()}
            
            # Evaluate each detection
            for i, detection in enumerate(class_detections):
                img_name = detection[0]
                max_iou = 0.0
                best_gt_idx = -1
                
                # Find ground truth with highest IoU
                if img_name in class_gts:
                    for j, gt in enumerate(class_gts[img_name]):
                        current_iou = Evaluator.iou(detection[3], gt[3])
                        if current_iou > max_iou:
                            max_iou = current_iou
                            best_gt_idx = j
                
                # Assign as true positive or false positive
                if max_iou >= iou_threshold:
                    if detected_gt[img_name][best_gt_idx] == 0:
                        tp[i] = 1  # True positive
                        detected_gt[img_name][best_gt_idx] = 1  # Mark as detected
                    else:
                        fp[i] = 1  # False positive (duplicate detection)
                else:
                    fp[i] = 1  # False positive (low IoU)
            
            # Compute cumulative sums
            cum_fp = np.cumsum(fp)
            cum_tp = np.cumsum(tp)
            
            # Compute precision and recall
            recall = cum_tp / n_pos if n_pos > 0 else np.zeros_like(cum_tp)
            precision = np.divide(cum_tp, (cum_fp + cum_tp), 
                                out=np.zeros_like(cum_tp), 
                                where=(cum_fp + cum_tp) != 0)
            
            # Calculate average precision
            if method == MethodAveragePrecision.EveryPointInterpolation:
                ap, mpre, mrec, _ = Evaluator.calculate_average_precision(recall, precision)
            else:
                ap, mpre, mrec, _ = Evaluator.eleven_point_interpolated_ap(recall, precision)
            
            # Store results for this class
            ret.append({
                'class': class_id,
                'precision': precision,
                'recall': recall,
                'AP': ap,
                'interpolated precision': mpre,
                'interpolated recall': mrec,
                'total positives': n_pos,
                'total TP': np.sum(tp),
                'total FP': np.sum(fp)
            })
        
        return ret

    def plot_precision_recall_curve(
        self,
        bounding_boxes: BoundingBoxes,
        iou_threshold: float = 0.5,
        method: MethodAveragePrecision = MethodAveragePrecision.EveryPointInterpolation,
        show_ap: bool = False,
        show_interpolated_precision: bool = False,
        save_path: Optional[str] = None,
        show_graphic: bool = True
    ) -> List[Dict]:
        """
        Plot precision-recall curves for each class.
        
        Args:
            bounding_boxes: Collection of ground truth and detected bounding boxes
            iou_threshold: IoU threshold for considering a detection as correct
            method: Method for calculating average precision
            show_ap: Whether to show AP in the plot title
            show_interpolated_precision: Whether to show interpolated precision
            save_path: Directory to save plots (None to not save)
            show_graphic: Whether to display the plot
            
        Returns:
            List of evaluation results for each class
        """
        results = self.get_pascal_voc_metrics(bounding_boxes, iou_threshold, method)
        
        for result in results:
            class_id = result['class']
            precision = result['precision']
            recall = result['recall']
            ap = result['AP']
            mpre = result['interpolated precision']
            mrec = result['interpolated recall']
            npos = result['total positives']
            total_tp = result['total TP']
            total_fp = result['total FP']
            
            plt.figure(figsize=(10, 7))
            plt.grid(True)
            
            if show_interpolated_precision:
                if method == MethodAveragePrecision.EveryPointInterpolation:
                    plt.plot(mrec, mpre, '--r', label='Interpolated precision (every point)')
                else:
                    # For 11-point interpolation
                    unique_recall = np.unique(mrec)
                    max_precision = [max(mpre[mrec >= r]) for r in unique_recall]
                    plt.plot(unique_recall, max_precision, 'or', label='11-point interpolated precision')
            
            plt.plot(recall, precision, label='Precision', linewidth=2)
            plt.xlabel('Recall', fontsize=12)
            plt.ylabel('Precision', fontsize=12)
            
            title = f'Precision x Recall curve\nClass: {class_id}'
            if show_ap:
                title += f', AP: {ap*100:.2f}%'
            plt.title(title, fontsize=14)
            
            plt.legend(loc='upper right', fontsize=10)
            plt.xlim([0, 1])
            plt.ylim([0, 1])
            
            if save_path is not None:
                os.makedirs(save_path, exist_ok=True)
                suffix = '_every' if method == MethodAveragePrecision.EveryPointInterpolation else '_11'
                plt.savefig(os.path.join(save_path, f'{class_id}{suffix}.png'), bbox_inches='tight')
            
            if show_graphic:
                plt.show()
                plt.pause(0.05)
                plt.close()
        
        return results

    @staticmethod
    def calculate_average_precision(
        rec: np.ndarray,
        prec: np.ndarray
    ) -> Tuple[float, np.ndarray, np.ndarray, List[int]]:
        """
        Calculate average precision using every-point interpolation.
        
        Args:
            rec: Recall values
            prec: Precision values
            
        Returns:
            Tuple containing (ap, interpolated_precision, interpolated_recall, recall_indices)
        """
        mrec = np.concatenate(([0], rec, [1]))
        mpre = np.concatenate(([0], prec, [0]))
        
        # Ensure precision decreases monotonically
        for i in range(len(mpre)-2, -1, -1):
            mpre[i] = max(mpre[i], mpre[i+1])
        
        # Find recall points where recall changes
        recall_change_indices = np.where(mrec[1:] != mrec[:-1])[0] + 1
        
        # Calculate AP as area under the curve
        ap = np.sum((mrec[recall_change_indices] - mrec[recall_change_indices-1]) * 
                   mpre[recall_change_indices])
        
        return ap, mpre[:-1], mrec[:-1], recall_change_indices.tolist()

    @staticmethod
    def eleven_point_interpolated_ap(
        rec: np.ndarray,
        prec: np.ndarray
    ) -> Tuple[float, np.ndarray, np.ndarray, None]:
        """
        Calculate 11-point interpolated average precision.
        
        Args:
            rec: Recall values
            prec: Precision values
            
        Returns:
            Tuple containing (ap, interpolated_precision, interpolated_recall, None)
        """
        recall_values = np.linspace(0, 1, 11)[::-1]
        rho_interp = []
        
        for r in recall_values:
            # Find precision values where recall >= r
            prec_at_recall = prec[rec >= r]
            if prec_at_recall.size > 0:
                rho_interp.append(np.max(prec_at_recall))
            else:
                rho_interp.append(0)
        
        ap = np.mean(rho_interp)
        
        # Generate values for plotting
        recall_plot = np.concatenate(([0], recall_values, [0]))
        prec_plot = np.concatenate(([0], rho_interp, [0]))
        
        return ap, prec_plot, recall_plot, None

    @staticmethod
    def iou(box_a: Tuple[float, float, float, float], box_b: Tuple[float, float, float, float]) -> float:
        """
        Calculate Intersection over Union (IoU) between two bounding boxes.
        
        Args:
            box_a: First bounding box (x1, y1, x2, y2)
            box_b: Second bounding box (x1, y1, x2, y2)
            
        Returns:
            IoU value between 0 and 1
        """
        if not Evaluator._boxes_intersect(box_a, box_b):
            return 0.0
        
        inter_area = Evaluator._get_intersection_area(box_a, box_b)
        union_area = Evaluator._get_union_areas(box_a, box_b, inter_area)
        
        return inter_area / union_area

    @staticmethod
    def _boxes_intersect(box_a: Tuple[float, float, float, float], box_b: Tuple[float, float, float, float]) -> bool:
        """Check if two boxes intersect."""
        return not (box_a[0] > box_b[2] or 
                   box_b[0] > box_a[2] or 
                   box_a[3] < box_b[1] or 
                   box_a[1] > box_b[3])

    @staticmethod
    def _get_intersection_area(box_a: Tuple[float, float, float, float], box_b: Tuple[float, float, float, float]) -> float:
        """Calculate intersection area of two boxes."""
        x_a = max(box_a[0], box_b[0])
        y_a = max(box_a[1], box_b[1])
        x_b = min(box_a[2], box_b[2])
        y_b = min(box_a[3], box_b[3])
        return max(0, x_b - x_a + 1) * max(0, y_b - y_a + 1)

    @staticmethod
    def _get_union_areas(
        box_a: Tuple[float, float, float, float],
        box_b: Tuple[float, float, float, float],
        inter_area: Optional[float] = None
    ) -> float:
        """Calculate union area of two boxes."""
        area_a = Evaluator._get_area(box_a)
        area_b = Evaluator._get_area(box_b)
        if inter_area is None:
            inter_area = Evaluator._get_intersection_area(box_a, box_b)
        return float(area_a + area_b - inter_area)

    @staticmethod
    def _get_area(box: Tuple[float, float, float, float]) -> float:
        """Calculate area of a box."""
        return (box[2] - box[0] + 1) * (box[3] - box[1] + 1)
