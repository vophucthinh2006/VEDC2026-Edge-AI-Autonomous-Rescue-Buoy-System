import numpy as np
from src.config import LOWER_BODY_INDICES, LOWER_BODY_WATER_THRESH

class RiskAnalyzer:
    @staticmethod
    def is_person_on_boat(person_box, boat_detections, overlap_thresh=0.4):
        """
        Kiểm tra xem Bounding Box của Người có nằm trong/đè lên Bounding Box của Thuyền không.
        - person_box: [x, y, w, h]
        - boat_detections: danh sách các thuyền [(box, conf, cls_id), ...]
        - overlap_thresh: tỷ lệ diện tích người nằm trong thuyền (mặc định >= 40% là ở trên thuyền)
        """
        if not boat_detections:
            return False

        px, py, pw, ph = person_box
        px2, py2 = px + pw, py + ph
        person_area = pw * ph

        if person_area <= 0:
            return False

        for (bx, by, bw, bh), _, _ in boat_detections:
            bx2, by2 = bx + bw, by + bh

            # Tính diện tích giao nhau (Intersection) giữa Người và Thuyền
            ix1 = max(px, bx)
            iy1 = max(py, by)
            ix2 = min(px2, bx2)
            iy2 = min(py2, by2)

            iw = max(0, ix2 - ix1)
            ih = max(0, iy2 - iy1)
            intersection_area = iw * ih

            # Nếu phần lớn diện tích BBox của người đè lên thuyền -> Xác nhận ở trên thuyền
            overlap_ratio = intersection_area / person_area
            if overlap_ratio >= overlap_thresh:
                return True

        return False

    @staticmethod
    def evaluate_water_risk(keypoints):
        """Đánh giá mức độ nguy hiểm dựa trên điểm Confidence khớp thân dưới."""
        lower_scores = [keypoints[i][2] for i in LOWER_BODY_INDICES]
        avg_lower_score = np.mean(lower_scores) if lower_scores else 0.0

        if avg_lower_score < LOWER_BODY_WATER_THRESH:
            return "IN_WATER_HIGH_RISK", avg_lower_score
        else:
            return "NORMAL_VISIBLE", avg_lower_score


class KeypointSmoother:
    def __init__(self, alpha=0.65):
        self.alpha = alpha
        self.prev_kpts = None

    def smooth(self, current_kpts):
        """Làm mượt tọa độ Keypoint giảm giật lắc ở cự ly xa bằng EMA."""
        if self.prev_kpts is None:
            self.prev_kpts = current_kpts.copy()
            return current_kpts

        smoothed = current_kpts.copy()
        for i in range(len(current_kpts)):
            if current_kpts[i][2] > 0.15:
                smoothed[i][0] = self.alpha * current_kpts[i][0] + (1 - self.alpha) * self.prev_kpts[i][0]
                smoothed[i][1] = self.alpha * current_kpts[i][1] + (1 - self.alpha) * self.prev_kpts[i][1]
        
        self.prev_kpts = smoothed.copy()
        return smoothed


class BoxTracker:
    """Bộ đệm giữ Bounding Box ổn định, chống chập chờn khi bị che khuất tạm thời"""
    def __init__(self, max_missing_frames=5, iou_thresh=0.3):
        self.max_missing_frames = max_missing_frames
        self.iou_thresh = iou_thresh
        self.tracked_boxes = [] # List chứa: [x, y, w, h, conf, cls_id, missing_count]

    def update(self, current_detections):
        updated_tracks = []
        used_detection_indices = set()

        # 1. Cập nhật các box cũ với detection mới
        for track in self.tracked_boxes:
            tb, t_conf, t_cls, missing_count = track[:4], track[4], track[5], track[6]
            best_iou = 0.0
            best_match_idx = -1

            for idx, (db, d_conf, d_cls) in enumerate(current_detections):
                if idx in used_detection_indices or int(t_cls) != int(d_cls):
                    continue
                
                iou = self._calc_iou(tb, db)
                if iou > best_iou:
                    best_iou = iou
                    best_match_idx = idx

            if best_iou >= self.iou_thresh and best_match_idx != -1:
                db, d_conf, d_cls = current_detections[best_match_idx]
                updated_tracks.append([db[0], db[1], db[2], db[3], d_conf, d_cls, 0])
                used_detection_indices.add(best_match_idx)
            else:
                # Nếu không tìm thấy match, tăng missing count lên 1
                if missing_count + 1 < self.max_missing_frames:
                    updated_tracks.append([tb[0], tb[1], tb[2], tb[3], t_conf, t_cls, missing_count + 1])

        # 2. Thêm các detection mới chưa được match
        for idx, (db, d_conf, d_cls) in enumerate(current_detections):
            if idx not in used_detection_indices:
                updated_tracks.append([db[0], db[1], db[2], db[3], d_conf, d_cls, 0])

        self.tracked_boxes = updated_tracks
        
        # Trả về danh sách detections dạng chuẩn [(box, conf, cls_id), ...]
        return [(t[:4], t[4], t[5]) for t in self.tracked_boxes]

    @staticmethod
    def _calc_iou(b1, b2):
        x1, y1, w1, h1 = b1
        x2, y2, w2, h2 = b2
        
        ix1, iy1 = max(x1, x2), max(y1, y2)
        ix2, iy2 = min(x1 + w1, x2 + w2), min(y1 + h1, y2 + h2)
        
        iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
        i_area = iw * ih
        u_area = (w1 * h1) + (w2 * h2) - i_area
        return i_area / u_area if u_area > 0 else 0