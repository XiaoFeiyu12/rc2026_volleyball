#!/usr/bin/env python3
"""
测试脚本：读取 mv.mp4 视频文件，使用 YOLOv11 OpenVINO 模型进行检测，
计算检测框中心与图像中心的像素偏移（pixel_diff_x, pixel_diff_y），
并实时打印到终端。
"""

import cv2
import numpy as np
import openvino as ov
import argparse
import time
from pathlib import Path


def preprocess(frame: np.ndarray, input_size: tuple = (640, 640)):
    """
    预处理：精确匹配 detector.cpp 的 PreProcessing。
    - resize 到 640x640 (INTER_AREA)
    - BGR → RGB（模型 PrePostProcessor 有 convert_color(RGB)）
    - u8 → f32 + scale(1/255)
    - HWC → NCHW
    """
    h, w = frame.shape[:2]
    scale_x = w / input_size[0]
    scale_y = h / input_size[1]

    resized = cv2.resize(frame, input_size, interpolation=cv2.INTER_AREA)
    # BGR → RGB（匹配 PrePostProcessor 的 .convert_color(RGB)）
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    # u8 → f32 + scale(1/255)（匹配 .convert_element_type(f32) + .scale(255)）
    blob = rgb.astype(np.float32) / 255.0
    # HWC → CHW → NCHW（匹配模型 NCHW layout）
    blob = blob.transpose(2, 0, 1)
    blob = np.expand_dims(blob, axis=0)
    return blob, scale_x, scale_y


def postprocess(output_tensor: np.ndarray, scale_x: float, scale_y: float,
                confidence_threshold: float = 0.7, nms_threshold: float = 0.5):
    """
    后处理：精确匹配 detector.cpp 的 PostProcessing。
    模型输出 shape [1, 5, 8400]，每列 = [cx, cy, w, h, confidence]
    - NMS 在 640x640 坐标系上进行（同 C++）
    - NMS 后再缩放到原图尺寸（同 GetBoundingBox）
    """
    detections = output_tensor[0]  # [5, 8400]

    box_list_640 = []  # NMS 前的框（640x640 坐标系，同 C++ box_list）
    confidence_list = []

    for i in range(detections.shape[1]):  # 遍历 8400 列，同 C++ detection_outputs.cols
        cx = detections[0, i]
        cy = detections[1, i]
        w = detections[2, i]
        h = detections[3, i]
        conf = detections[4, i]

        if conf < confidence_threshold:
            continue

        # 框坐标在 640x640 空间（同 C++ 的 box.x/box.y/box.width/box.height）
        x = int(cx - w / 2)
        y = int(cy - h / 2)
        w_int = int(w)
        h_int = int(h)

        box_list_640.append([x, y, w_int, h_int])
        confidence_list.append(float(conf))

    # NMS 在 640x640 空间进行（同 C++ cv::dnn::NMSBoxes）
    if len(box_list_640) > 0:
        indices = cv2.dnn.NMSBoxes(box_list_640, confidence_list,
                                   confidence_threshold, nms_threshold)
        if len(indices) > 0:
            indices = indices.flatten()
            results = []
            for idx in indices:
                # 缩放到原图（同 C++ GetBoundingBox）
                x_orig = int(box_list_640[idx][0] * scale_x)
                y_orig = int(box_list_640[idx][1] * scale_y)
                w_orig = int(box_list_640[idx][2] * scale_x)
                h_orig = int(box_list_640[idx][3] * scale_y)
                # 框中心在原图坐标（同 C++ result.cx/cy）
                cx_orig = x_orig + w_orig / 2
                cy_orig = y_orig + h_orig / 2
                results.append((cx_orig, cy_orig, w_orig, h_orig, confidence_list[idx]))
            return results
    return []


def main():
    parser = argparse.ArgumentParser(description='视频 PID 检测测试')
    parser.add_argument('--video', type=str, default='mv.mp4',
                        help='输入视频文件路径 (默认: mv.mp4)')
    parser.add_argument('--model', type=str,
                        default='model/volleyball_yolov11n_int8/best.xml',
                        help='OpenVINO 模型路径')
    parser.add_argument('--conf', type=float, default=0.7,
                        help='置信度阈值 (默认: 0.7)')
    parser.add_argument('--nms', type=float, default=0.5,
                        help='NMS 阈值 (默认: 0.5)')
    parser.add_argument('--input-size', type=int, default=640,
                        help='模型输入尺寸 (默认: 640)')
    parser.add_argument('--show', action='store_true',
                        help='显示检测结果窗口')
    parser.add_argument('--fps', action='store_true',
                        help='显示 FPS')
    parser.add_argument('--output', type=str, default=None,
                        help='输出结果视频路径 (如: mv_result.mp4)')
    args = parser.parse_args()

    # 检查视频文件
    video_path = Path(args.video)
    if not video_path.exists():
        print(f"[错误] 视频文件不存在: {video_path.resolve()}")
        return

    # 检查模型文件
    model_path = Path(args.model)
    if not model_path.exists():
        # 尝试常见位置
        script_dir = Path(__file__).resolve().parent      # test/
        pid_camera_dir = script_dir.parent                 # volleyball_pid_camera/
        src_dir = pid_camera_dir.parent                     # src/
        ws_dir = src_dir.parent                             # workspace root
        candidates = [
            Path(args.model),
            Path.cwd() / args.model,
            ws_dir / "src" / "volleyball_detect" / "model" / "volleyball_yolov11n_int8" / "best.xml",
            ws_dir / "install" / "volleyball_detect" / "share" / "volleyball_detect" /
                "model" / "volleyball_yolov11n_int8" / "best.xml",
        ]
        found = False
        for c in candidates:
            if c.exists():
                model_path = c
                found = True
                break
        if not found:
            print(f"[错误] 找不到模型文件。请使用 --model 指定路径")
            print(f"       已尝试:")
            for c in candidates:
                print(f"         - {c}")
            return

    # 加载 OpenVINO 模型
    print("[信息] 加载模型中...")
    core = ov.Core()
    model = core.read_model(str(model_path))
    compiled_model = core.compile_model(model, "CPU")
    infer_request = compiled_model.create_infer_request()

    input_size = (args.input_size, args.input_size)

    # 打开视频
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f"[错误] 无法打开视频: {video_path}")
        return

    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print(f"[信息] 视频: {video_path.name}")
    print(f"      分辨率: {frame_width}x{frame_height}")
    print(f"      帧率: {fps:.1f} FPS")
    print(f"      总帧数: {total_frames}")
    print(f"[信息] 模型: {model_path.name}")
    print(f"[信息] 置信度阈值: {args.conf}, NMS 阈值: {args.nms}")

    # 输出视频
    writer = None
    if args.output:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(args.output, fourcc, fps, (frame_width, frame_height))
        print(f"[信息] 输出视频: {args.output}")
    print()
    print("=" * 70)
    print(f"{'帧号':>6} | {'检测':>4} | {'像素偏移 X':>10} | {'像素偏移 Y':>10} | {'置信度':>8} | {'耗时(ms)':>8}")
    print("=" * 70)

    frame_idx = 0
    total_infer_time = 0.0
    detect_count = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame_idx += 1
            img_h, img_w = frame.shape[:2]

            # 预处理（返回 FP32 NCHW blob）
            blob, scale_x, scale_y = preprocess(frame, input_size)

            # 推理
            infer_start = time.perf_counter()
            infer_request.set_input_tensor(ov.Tensor(blob))
            infer_request.infer()
            output_tensor = infer_request.get_output_tensor().data
            infer_time = (time.perf_counter() - infer_start) * 1000  # ms
            total_infer_time += infer_time

            # 后处理
            results = postprocess(output_tensor, scale_x, scale_y,
                                  args.conf, args.nms)

            # ===== 绘制标注（始终绘制，用于输出视频） =====
            # 图像中心十字
            cv2.circle(frame, (img_w // 2, img_h // 2), 5, (0, 255, 0), 2)
            cv2.line(frame, (img_w // 2, 0), (img_w // 2, img_h - 1),
                     (0, 255, 0), 1)
            cv2.line(frame, (0, img_h // 2), (img_w - 1, img_h // 2),
                     (0, 255, 0), 1)

            if len(results) > 0:
                # 取置信度最高的框
                best = max(results, key=lambda r: r[4])
                cx, cy, w, h, conf = best
                pixel_diff_x = cx - img_w / 2
                pixel_diff_y = cy - img_h / 2
                detect_count += 1

                # 检测框
                cv2.rectangle(frame,
                              (int(cx - w / 2), int(cy - h / 2)),
                              (int(cx + w / 2), int(cy + h / 2)),
                              (255, 0, 0), 2)
                # 检测框中心
                cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                # 图像中心 → 检测框中心 连线
                cv2.line(frame,
                         (img_w // 2, img_h // 2),
                         (int(cx), int(cy)),
                         (255, 0, 0), 2)
                # 偏移量文字
                cv2.putText(frame,
                            f"dx:{pixel_diff_x:+.0f} dy:{pixel_diff_y:+.0f}",
                            (int(cx - w / 2), int(cy - h / 2) - 10),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(frame,
                            f"conf:{conf:.2f}",
                            (int(cx - w / 2), int(cy - h / 2) - 28),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 0), 1)

                print(f"{frame_idx:>6} | {'是':>4} | {pixel_diff_x:>+10.1f} | {pixel_diff_y:>+10.1f} | {conf:>7.3f} | {infer_time:>7.1f}")
            else:
                pixel_diff_x = 0.0
                pixel_diff_y = 0.0
                print(f"{frame_idx:>6} | {'否':>4} | {pixel_diff_x:>+10.1f} | {pixel_diff_y:>+10.1f} | {'N/A':>8} | {infer_time:>7.1f}")

            # FPS
            if args.fps or writer is not None:
                avg_time = total_infer_time / frame_idx
                cv2.putText(frame,
                            f"FPS: {1000.0 / avg_time:.1f}",
                            (16, 32),
                            cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)

            # 写入输出视频
            if writer is not None:
                writer.write(frame)

            # 显示窗口
            if args.show:
                cv2.imshow("PID Camera Test", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    print("\n[信息] 用户中断")
                    break

    except KeyboardInterrupt:
        print("\n[信息] 用户中断")

    finally:
        cap.release()
        if writer is not None:
            writer.release()
            print(f"[信息] 结果视频已保存: {args.output}")
        if args.show:
            cv2.destroyAllWindows()

    # 汇总
    print("=" * 70)
    avg_time = total_infer_time / frame_idx if frame_idx > 0 else 0
    print(f"\n[汇总]")
    print(f"      处理帧数: {frame_idx}")
    print(f"      检测到目标帧数: {detect_count}")
    print(f"      平均推理时间: {avg_time:.1f} ms ({1000.0 / avg_time:.1f} FPS)")
    print(f"      检测率: {detect_count / frame_idx * 100:.1f}%" if frame_idx > 0 else "")
    print(f"\n[完成] 测试结束")


if __name__ == "__main__":
    main()
