#!/usr/bin/env python3
"""
简单测试脚本：验证 yolo_center_detector 包的检测功能
"""

import cv2
import numpy as np
import sys

# 导入检测器
sys.path.insert(0, "/home/rvl/ros2_ws/src/yolo_center_detector")
from yolo_center_detector.detector import YoloCenterDetector

def test_detector():
    """测试检测器"""
    print("=" * 60)
    print("YOLO Center Detector 包功能测试")
    print("=" * 60)
    
    # 模型路径
    model_path = "/home/rvl/ros2_ws/src/yolo_center_detector/resource/models/yolo26s.pt"
    
    print(f"\n1. 加载模型: {model_path}")
    try:
        detector = YoloCenterDetector(
            model_path=model_path,
            device="cuda:0",
            conf=0.25,
            iou=0.7,
            imgsz=640,
            half=False,
            verbose=False,
        )
        print("   ✓ 模型加载成功！")
        print(f"   支持的类别数: {len(detector.id2name)}")
        print(f"   类别列表: {list(detector.id2name.values())}")
    except Exception as e:
        print(f"   ✗ 模型加载失败: {e}")
        return False
    
    # 创建一个测试图像（随机颜色）
    print("\n2. 创建测试图像...")
    test_image = np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8)
    print("   ✓ 测试图像创建成功 (640x640x3)")
    
    # 测试检测功能
    print("\n3. 测试检测功能...")
    target_list = ["person"]  # 只检测 person
    try:
        detections = detector.detect(test_image, target_list, topk_per_class=None)
        print(f"   ✓ 检测完成，检测到 {len(detections)} 个目标")
        for det in detections:
            cx, cy = det.center
            x1, y1, x2, y2 = det.bbox
            print(f"     - {det.cls_name}: 中心点 ({cx}, {cy}), 置信度 {det.conf:.2f}")
    except Exception as e:
        print(f"   ✗ 检测失败: {e}")
        return False
    
    # 测试不同的检测清单
    print("\n4. 测试不同的检测清单...")
    test_lists = [
        ["person", "car"],
        ["dog", "cat"],
        ["bottle"],
    ]
    
    for target_list in test_lists:
        try:
            detections = detector.detect(test_image, target_list, topk_per_class=None)
            valid_targets = [t for t in target_list if t in detector.name2id]
            print(f"   ✓ 清单 {valid_targets}: 检测到 {len(detections)} 个目标")
        except Exception as e:
            print(f"   ✗ 清单 {target_list} 检测失败: {e}")
            return False
    
    print("\n" + "=" * 60)
    print("所有测试通过！✓")
    print("=" * 60)
    return True

if __name__ == "__main__":
    success = test_detector()
    sys.exit(0 if success else 1)
