"""Selection-only duplicate handling; never changes detector NMS or thresholds."""
import math

from sar_yolo_detector.box_fusion import iou


def unique_roi_matches(matches, class_groups):
    """Collapse near-identical raw queries, keeping the best original-ROI fit.

    Different stable identities and displaced/part/whole boxes stay separate.
    Every member must agree with every other member, avoiding transitive merges.
    The caller still applies its original ambiguity and confidence gates.
    """
    groups = []

    def duplicate(a, b):
        if a.stable and b.stable and a.local_id >= 0 and b.local_id >= 0 and a.local_id != b.local_id:
            return False
        if not (a.class_id == b.class_id or any(
                a.class_id in group and b.class_id in group for group in class_groups)):
            return False
        if iou(a.box, b.box) < .65:
            return False
        ax, ay, aw, ah = a.box
        bx, by, bw, bh = b.box
        distance = math.hypot(ax + aw / 2 - bx - bw / 2,
                              ay + ah / 2 - by - bh / 2)
        return distance <= .10 * min(math.hypot(aw, ah), math.hypot(bw, bh))

    for match in sorted(matches, key=lambda item: (item[1], item[0].confidence), reverse=True):
        group = next((group for group in groups
                      if all(duplicate(match[0], member[0]) for member in group)), None)
        if group is None:
            groups.append([match])
        else:
            group.append(match)
    return [group[0] for group in groups]
