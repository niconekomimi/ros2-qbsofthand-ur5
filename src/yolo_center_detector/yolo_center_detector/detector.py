from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Tuple, Union, Optional

import numpy as np
from ultralytics import YOLO

BBox = Tuple[int, int, int, int]
Center = Tuple[int, int]

@dataclass
class DetItem:
    cls_name: str
    center: Center
    bbox: BBox
    conf: float

class YoloCenterDetector:
    def __init__(
        self,
        model_path: str,
        device: str = "cuda:0",
        conf: float = 0.25,
        iou: float = 0.7,
        imgsz: int = 640,
        half: bool = False,
        verbose: bool = False,
    ) -> None:
        self.model = YOLO(model_path)
        self.device = device
        self.conf = conf
        self.iou = iou
        self.imgsz = imgsz
        self.half = half
        self.verbose = verbose

        names = self.model.names
        if isinstance(names, dict):
            self.id2name = names
        else:
            self.id2name = {i: n for i, n in enumerate(names)}
        self.name2id = {v: k for k, v in self.id2name.items()}

    def detect(
        self,
        image: Union[np.ndarray, str],
        target_list: List[str],
        topk_per_class: Optional[int] = None,
    ) -> List[DetItem]:
        # 清单映射到 class ids（跳过模型不认识的类名）
        target_ids = []
        valid_names = []
        for n in target_list:
            if n in self.name2id:
                valid_names.append(n)
                target_ids.append(self.name2id[n])

        if not target_ids:
            return []

        preds = self.model.predict(
            source=image,
            device=self.device,
            conf=self.conf,
            iou=self.iou,
            imgsz=self.imgsz,
            half=self.half,
            verbose=self.verbose,
            classes=target_ids,  # 关键：按清单前置过滤
        )

        r0 = preds[0]
        if r0.boxes is None or len(r0.boxes) == 0:
            return []

        boxes = r0.boxes.xyxy.cpu().numpy()
        confs = r0.boxes.conf.cpu().numpy()
        clss = r0.boxes.cls.cpu().numpy().astype(int)

        # 先按类聚合，方便 topk
        tmp: Dict[str, List[DetItem]] = {n: [] for n in valid_names}
        for (x1, y1, x2, y2), cf, cid in zip(boxes, confs, clss):
            cls_name = self.id2name.get(int(cid), None)
            if cls_name is None or cls_name not in tmp:
                continue
            cx = int(round((float(x1) + float(x2)) / 2.0))
            cy = int(round((float(y1) + float(y2)) / 2.0))
            tmp[cls_name].append(
                DetItem(
                    cls_name=cls_name,
                    center=(cx, cy),
                    bbox=(int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2))),
                    conf=float(cf),
                )
            )

        out: List[DetItem] = []
        for cls_name, items in tmp.items():
            items.sort(key=lambda z: z.conf, reverse=True)
            if topk_per_class is not None:
                items = items[:topk_per_class]
            out.extend(items)

        # 整体按置信度排序（可选）
        out.sort(key=lambda z: z.conf, reverse=True)
        return out
