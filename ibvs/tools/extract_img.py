#!/usr/bin/env python3
import cv2
import numpy as np
from cv_bridge import CvBridge
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sys
import os

def extract_images_from_bag(bag_path, topic_name, output_dir, step=5, 
                            image_format='jpg', use_time_stamp=True):
    """
    从rosbag中提取图像并抽帧保存
    
    :param bag_path: bag文件路径（例如 'your_bag.db3'）
    :param topic_name: 图像话题名
    :param output_dir: 输出文件夹
    :param step: 抽帧间隔（每隔step帧保存一帧）
    :param image_format: 保存格式 ('jpg', 'png')
    :param use_time_stamp: 是否用时间戳命名文件（否则用序号）
    """
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    # 初始化 CvBridge
    bridge = CvBridge()
    
    # 打开bag
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    # 获取话题类型
    topics = reader.get_all_topics_and_types()
    topic_type = None
    for t in topics:
        if t.name == topic_name:
            topic_type = t.type
            break
    if topic_type is None:
        print(f"Topic '{topic_name}' not found in bag.")
        return
    
    print(f"Topic '{topic_name}' type: {topic_type}")
    
    # 判断是否是压缩图像
    is_compressed = 'CompressedImage' in topic_type
    
    # 准备消息反序列化
    msg_class = get_message(topic_type)
    
    frame_count = 0
    saved_count = 0
    
    # 遍历所有消息
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != topic_name:
            continue
        
        # 反序列化消息
        msg = deserialize_message(data, msg_class)
        
        # 抽帧：按帧数间隔
        if frame_count % step != 0:
            frame_count += 1
            continue
        
        # 转换为OpenCV图像
        try:
            if is_compressed:
                # 压缩图像处理
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                # 普通图像
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            print(f"Error converting frame {frame_count}: {e}")
            frame_count += 1
            continue
        
        if cv_img is None:
            print(f"Empty image at frame {frame_count}")
            frame_count += 1
            continue
        
        # 生成文件名
        if use_time_stamp:
            # 时间戳（纳秒转秒）
            sec = timestamp // 10**9
            nsec = timestamp % 10**9
            filename = f"{sec}_{nsec}.{image_format}"
        else:
            filename = f"frame_{saved_count:06d}.{image_format}"
        
        save_path = os.path.join(output_dir, filename)
        cv2.imwrite(save_path, cv_img)
        saved_count += 1
        print(f"Saved {save_path}")
        
        frame_count += 1
    
    print(f"Done. Processed {frame_count} frames, saved {saved_count} images.")

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: python extract_images.py <bag_path> <topic_name> <output_dir> [step] [format]")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    topic_name = sys.argv[2]
    output_dir = sys.argv[3]
    step = int(sys.argv[4]) if len(sys.argv) > 4 else 10
    fmt = sys.argv[5] if len(sys.argv) > 5 else 'jpg'
    
    extract_images_from_bag(bag_path, topic_name, output_dir, step, fmt)