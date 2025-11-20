# Video Processor Node

这是一个ROS2节点，用于处理视频文件并通过装甲板检测器进行标注。

## 功能特点

- 📹 读取本地视频文件
- 🎯 将视频帧发布到检测节点
- 🔍 接收检测结果
- ✏️ 在视频帧上绘制检测框
- 💾 导出标注后的视频
- 📊 显示处理进度和统计信息

## 依赖项

- ROS2 (Humble/Foxy)
- OpenCV 4.x
- cv_bridge
- auto_aim_interfaces
- armor_detector

## 编译

```bash
cd ~/Codespace/RM/auto_aim_rm
colcon build --packages-select video_processor
source install/setup.bash
```

## 使用方法

### 基本用法

```bash
# 启动视频处理器和检测器
ros2 launch video_processor video_processor.launch.py \
  input_video:=/path/to/your/video.mp4 \
  output_video:=/path/to/output_annotated.mp4
```

### 参数说明

- `input_video`: 输入视频文件路径（必需）
- `output_video`: 输出标注视频路径（默认: output_annotated.mp4）
- `playback_speed`: 播放速度倍数（默认: 1.0，2.0表示2倍速）
- `show_preview`: 是否显示预览窗口（默认: false）

### 示例

#### 1. 处理单个视频

```bash
ros2 launch video_processor video_processor.launch.py \
  input_video:=~/Videos/test.mp4 \
  output_video:=~/Videos/test_annotated.mp4
```

#### 2. 2倍速处理并显示预览

```bash
ros2 launch video_processor video_processor.launch.py \
  input_video:=~/Videos/test.mp4 \
  output_video:=~/Videos/test_annotated.mp4 \
  playback_speed:=2.0 \
  show_preview:=true
```

#### 3. 批量处理脚本

创建一个脚本 `batch_process.sh`:

```bash
#!/bin/bash

INPUT_DIR="$HOME/Videos/raw"
OUTPUT_DIR="$HOME/Videos/annotated"

mkdir -p "$OUTPUT_DIR"

for video in "$INPUT_DIR"/*.mp4; do
    filename=$(basename "$video")
    echo "Processing: $filename"
    
    ros2 launch video_processor video_processor.launch.py \
      input_video:="$video" \
      output_video:="$OUTPUT_DIR/${filename%.mp4}_annotated.mp4" \
      playback_speed:=2.0
    
    echo "Completed: $filename"
done

echo "All videos processed!"
```

运行批处理：

```bash
chmod +x batch_process.sh
./batch_process.sh
```

## 输出格式

标注视频包含以下信息：

- ✅ 检测到的装甲板位置（圆圈标记）
- 🎨 颜色标识（蓝色/红色/灰色/紫色）
- 📏 装甲板类型（SMALL/LARGE）
- 📍 到图像中心的距离
- 📊 当前帧数和进度百分比
- 🔢 检测到的装甲板数量

## 话题说明

### 发布的话题

- `/image_raw` (sensor_msgs/Image): 视频帧图像
- `/camera_info` (sensor_msgs/CameraInfo): 相机信息

### 订阅的话题

- `/detector/armors` (auto_aim_interfaces/Armors): 检测结果

## 注意事项

1. **视频格式**: 支持常见视频格式（mp4, avi, mov等）
2. **处理时间**: 处理时间取决于视频长度和检测器性能
3. **内存使用**: 大分辨率视频会占用更多内存
4. **输出编码**: 输出视频使用mp4v编码器

## 性能优化

- 使用 `playback_speed` 参数加速处理（但检测精度可能降低）
- 关闭预览窗口（`show_preview:=false`）可提高处理速度
- 考虑降低输入视频分辨率以加快处理

## 故障排除

### 视频打不开

```bash
# 检查视频文件是否存在
ls -lh /path/to/your/video.mp4

# 检查OpenCV是否支持该格式
python3 -c "import cv2; print(cv2.getBuildInformation())"
```

### 检测器未响应

```bash
# 检查检测器节点是否运行
ros2 node list

# 检查话题连接
ros2 topic list
ros2 topic echo /detector/armors
```

### 输出视频无法播放

```bash
# 安装必要的编解码器
sudo apt install ubuntu-restricted-extras

# 使用ffmpeg转换格式
ffmpeg -i output_annotated.mp4 -vcodec libx264 output_fixed.mp4
```

## 示例输出

处理过程中会显示进度信息：

```
[video_processor]: Video info: 1280x720 @ 30 fps, total frames: 1500
[video_processor]: Processing video at 1.00x speed
[video_processor]: Output will be saved to: output_annotated.mp4
[video_processor]: Progress: 100/1500 frames (6.7%), Detected: 45
[video_processor]: Progress: 200/1500 frames (13.3%), Detected: 98
...
[video_processor]: End of video reached
[video_processor]: Video processing completed!
[video_processor]: Total frames: 1500, Processed: 1500, Detected: 750 (50.00%)
```

## 许可证

Apache-2.0
