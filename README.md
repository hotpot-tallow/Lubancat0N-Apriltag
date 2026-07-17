# AprilTag Landing Target for LubanCat 0N and Raspberry Pi 4B

鲁班猫 0N 或 Raspberry Pi 4B 读取摄像头画面，识别嵌套 `tag36h11` AprilTag，
并向 ArduPilot 飞控发送 MAVLink `LANDING_TARGET` 消息。

Raspberry Pi 4B + Camera Module 2 使用 Picamera2 灰度取流，迁移步骤见
[`docs/raspberry-pi-4b.md`](docs/raspberry-pi-4b.md)。

当前默认嵌套码尺寸：

| tag_id | black border size |
| --- | --- |
| 0 | 500 mm |
| 1 | 128 mm |
| 2 | 30 mm |

## 1. 安装系统依赖

Ubuntu 20.04 上先安装基础编译、OpenCV 和 NumPy：

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv python3-dev build-essential cmake pkg-config
sudo apt install -y python3-opencv python3-numpy v4l-utils
```

OpenCV 和 NumPy 建议使用 apt 安装的 `python3-opencv`、`python3-numpy`，不要优先用 pip 在板子上编译。

## 2. 安装 Python 环境

进入仓库：

```bash
cd ~/Lubancat0N-Apriltag
python3 -m venv venv --system-site-packages
source venv/bin/activate
python3 -m pip install --upgrade pip setuptools wheel
```

## 3. 安装 AprilTag 和 MAVLink 包

本项目使用 `pupil-apriltags` 做 AprilTag 识别和位姿估计，使用 `pymavlink` 发送 `LANDING_TARGET`。

推荐直接安装：

```bash
python3 -m pip install pupil-apriltags pymavlink pyserial
```

或者使用仓库里的依赖文件：

```bash
python3 -m pip install -r requirements.txt
```

安装完成后检查：

```bash
python3 -c "from pupil_apriltags import Detector; print('pupil-apriltags ok')"
python3 -c "from pymavlink import mavutil; print('pymavlink ok')"
```

如果 `pupil-apriltags` 安装失败，通常是缺少编译工具，先确认已经执行过：

```bash
sudo apt install -y python3-dev build-essential cmake pkg-config
```

然后重新运行：

```bash
python3 -m pip install --no-cache-dir pupil-apriltags
```

## 4. 修改配置

复制配置：

```bash
cp config/example_config.json config/lubancat0n.json
```

重点修改：

```json
"camera": {
  "device": 0,
  "width": 1920,
  "height": 1080,
  "fx": 1800.0,
  "fy": 1350.0,
  "cx": 960.0,
  "cy": 540.0
},
"mavlink": {
  "connection": "/dev/ttyS8",
  "baud": 115200
}
```

`width/height` 是摄像头采集分辨率，不是预览窗口大小。`fx/fy/cx/cy` 必须换成该分辨率下你自己的相机标定结果。没有内参时可以先用近似值跑通识别，但不要直接拿去飞。

如果你使用鲁班猫 MIPI/RKISP 摄像头，V4L2 可能不接受 OpenCV 设置的 `1280x720`，实际会打开成 `3264x2160`。这会导致 CPU 压力变大，也会让 1280x720 标定出的内参和实际图像分辨率不一致。建议改用 GStreamer 管线强制输出 1280x720：

```json
"camera": {
  "device": "v4l2src device=/dev/video0 io-mode=4 ! video/x-raw,format=NV12,width=1280,height=720 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false",
  "backend": "gstreamer",
  "fourcc": "",
  "buffer_size": 1,
  "width": 1280,
  "height": 720,
  "fps": 30,
  "fx": 982.6,
  "fy": 733.2,
  "cx": 653.1,
  "cy": 361.5
}
```

### V4L2 和 GStreamer 的区别

`V4L2` 是 Linux 内核提供的视频设备接口，`/dev/video0`、`/dev/video1` 这类设备就是通过它访问的。OpenCV 使用 `backend: "v4l2"` 时，基本就是直接让 OpenCV 去和 V4L2 设备要帧。优点是简单；缺点是对 MIPI/RKISP 这种多层摄像头链路，格式转换、分辨率协商、缓冲控制都比较粗，容易出现实际分辨率不是你配置的值，或者长时间运行后卡在 `cap.read()`。

`GStreamer` 是一条可配置的视频处理管线。它底层仍然可以从 V4L2 取 `/dev/video0`，但你可以明确写出输入格式、分辨率、帧率、格式转换和输出缓冲策略。对鲁班猫 MIPI/RKISP 摄像头，推荐用 GStreamer，是因为它能强制 `1280x720` 输出，并用 `appsink drop=true max-buffers=1 sync=false` 丢掉旧帧，只保留最新帧，减少缓冲堆积导致的卡死。

简单理解：

```text
V4L2: OpenCV 直接找摄像头要帧，简单但控制少。
GStreamer: 先搭一条取流/转格式/丢旧帧的管线，再把最新帧交给 OpenCV，控制更细。
```

### 从根源减少摄像头卡死

`camera read timeout` 自动重启只是保险措施；真正要减少卡死，优先把摄像头链路调稳定：

1. 强制使用 GStreamer 管线，不要让 OpenCV/V4L2 自动协商到 `3264x2160`。
2. 分辨率先固定为 `1280x720`，并确保相机内参也是这个分辨率下标定的结果。
3. `appsink` 必须保留 `drop=true max-buffers=1 sync=false`，处理不过来时丢旧帧，不堆积旧画面。
4. `fourcc` 设为空字符串 `""`，不要给 MIPI 摄像头强行设置 `MJPG`。
5. 如果仍然卡，先把 AprilTag 的 `quad_decimate` 从 `2.0` 调到 `3.0`，降低 CPU 压力。
6. 用 `tools/test_camera.py --headless` 确认实际打开分辨率是 `1280x720`，不是高分辨率。

推荐配置如下：

```json
"camera": {
  "device": "v4l2src device=/dev/video0 io-mode=4 ! video/x-raw,format=NV12,width=1280,height=720,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false",
  "backend": "gstreamer",
  "fourcc": "",
  "buffer_size": 1,
  "width": 1280,
  "height": 720,
  "fps": 30,
  "fx": 982.6,
  "fy": 733.2,
  "cx": 653.1,
  "cy": 361.5
}
```

改完后按这个顺序验证：

```bash
PYTHONPATH=src python3 tools/test_camera.py --config config/lubancat0n.json --headless
PYTHONPATH=src python3 tools/test_tags.py --config config/lubancat0n.json --headless --print-every 0.5
PYTHONPATH=src python3 tools/landing_target.py --config config/lubancat0n.json --dry-run
```

如果 `NV12` 管线打不开，再用下面命令查看 `/dev/video0` 支持的格式：

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

然后把管线里的 `format=NV12` 改成实际支持且稳定的格式，例如 `UYVY`。如果 GStreamer 报缺少插件，先安装：

```bash
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
```

如果终端反复出现下面这种 OpenCV/GStreamer 警告：

```text
GStreamer warning: GStreamer: unhandled property
```

说明程序打开的是 GStreamer 管线，但又额外调用了 OpenCV 的 `cap.set(width/height/fps)`。当前代码已经对管线模式跳过这些 `setProperty` 调用；更新代码后仍然反复 `camera read failed`，就要单独测试管线本身能不能稳定吐帧。

先用 `gst-launch-1.0` 绕过 OpenCV 测试：

```bash
gst-launch-1.0 -v v4l2src device=/dev/video0 io-mode=4 ! \
  'video/x-raw,format=NV12,width=1280,height=720,framerate=30/1' ! \
  videoconvert ! fpsdisplaysink video-sink=fakesink sync=false
```

如果这个命令本身就报错或没有稳定 FPS，说明不是 Python 代码问题，而是这条 GStreamer 管线不适合当前摄像头。依次尝试：

```bash
gst-launch-1.0 -v v4l2src device=/dev/video0 io-mode=2 ! \
  'video/x-raw,format=NV12,width=1280,height=720,framerate=30/1' ! \
  videoconvert ! fpsdisplaysink video-sink=fakesink sync=false

gst-launch-1.0 -v v4l2src device=/dev/video0 ! \
  'video/x-raw,format=NV12,width=1280,height=720,framerate=30/1' ! \
  videoconvert ! fpsdisplaysink video-sink=fakesink sync=false

gst-launch-1.0 -v v4l2src device=/dev/video0 io-mode=4 ! \
  'video/x-raw,format=UYVY,width=1280,height=720,framerate=30/1' ! \
  videoconvert ! fpsdisplaysink video-sink=fakesink sync=false
```

哪一条 `gst-launch-1.0` 能稳定显示 FPS，就把同样的 `v4l2src ... ! video/x-raw ... ! videoconvert ... ! appsink ...` 写回 `lubancat0n.json`。

旧版单家族嵌套码尺寸在 `tag_sizes_m` 里：

```json
"tag_sizes_m": {
  "0": 0.5,
  "1": 0.128,
  "2": 0.03
}
```

这里单位是米，填的是 AprilTag 检测边界实际边长。程序会选择当前已接受标签中物理尺寸最大的一个，因此上面的配置同时识别到 `0`、`1`、`2` 时会使用 `0.5m` 的标签。

AprilTag 3 原生用于递归标签的家族是 `tagCustom48h12`，它在中心预留了可放置小标签的区域。需要同时识别原生递归外框和 `tag36h11 ID 1` 时使用多家族配置：

```json
"families": [
  {
    "name": "tagCustom48h12",
    "bits_corrected": 2,
    "max_codes": 10,
    "tag_sizes_m": {"0": 0.5}
  },
  {
    "name": "tag36h11",
    "bits_corrected": 2,
    "tag_sizes_m": {"1": 0.128}
  }
]
```

程序会把两个家族挂载到同一个检测器中，一帧图像只执行一次四边形搜索，再按检测结果所属家族和实际尺寸选择大码。多家族仍会增加候选解码开销，但不会重复扫描整张图像。树莓派上建议先从 `quad_decimate=2.0` 开始测试。

`tagCustom48h12` 有四万多个合法码，直接为整个家族建立2位纠错表可能占用数GB内存。模板用 `max_codes=10` 将它限制为只加载和识别 ID `0～9`，因此可以保留 `bits_corrected=2`。配置的标签 ID 必须小于 `max_codes`。全局 `max_hamming` 是检测完成后的二次过滤，与这里的码表大小不是同一个参数。

注意 `tagCustom48h12` 官方PNG的完整宽度与检测尺寸之比是 `10/6`，`tag36h11` 含白边完整宽度与检测尺寸之比是 `10/8`。上例的 `0.5m` 外层完整图案约宽 `0.833m`，中心孔洞约宽 `0.167m`；`0.128m` 内层含白边约宽 `0.160m`，可以放入孔洞。配置尺寸必须按检测边界填写，不能直接填整张打印图的宽度。

## 5. 测摄像头

有桌面环境：

```bash
PYTHONPATH=src python3 tools/test_camera.py --config config/lubancat0n.json
```

无桌面环境：

```bash
PYTHONPATH=src python3 tools/test_camera.py --config config/lubancat0n.json --headless
```

成功后会保存 `camera_test.jpg`。

## 6. 测 AprilTag

有桌面环境时直接运行，会弹出实时画面，画面左上角显示 FPS，识别到 tag 后会画出绿色边框、tag ID、坐标和距离：

```bash
PYTHONPATH=src python3 tools/test_tags.py --config config/lubancat0n.json
```

默认预览窗口会按 `0.5` 缩小显示，但 AprilTag 仍使用配置里的采集分辨率做识别。可以手动调整：

```bash
PYTHONPATH=src python3 tools/test_tags.py --config config/lubancat0n.json --preview-scale 0.35
```

画面左上角还会显示 `detect`、`raw`、`ok`：

- `detect` 是每帧 AprilTag 检测耗时，持续很高说明 CPU 压力大
- `raw` 是底层检测到的原始 tag 数量
- `ok` 是通过 `hamming` 和 `decision_margin` 过滤后的数量

按 `Esc` 或 `q` 退出。

无桌面环境时使用：

```bash
PYTHONPATH=src python3 tools/test_tags.py --config config/lubancat0n.json --headless
```

看到类似输出就说明识别和位姿估计通了：

```text
id=0 size=0.500m px=312.5 expect_z=1.410 cam=(+0.012,-0.035,+1.420) body=(+0.035,+0.012,+1.420) q=(+0.998,+0.010,+0.020,+0.055) dist=1.421 margin=84.2
```

这里重点看：

- `size`：当前选中 tag 的真实边长，单位是米，必须和打印出来的实际尺寸一致
- `px`：tag 在画面中的像素边长
- `expect_z`：用 `fx * tag_size / px` 粗略估计出的距离
- `dist`：solvePnP 解算出的三维距离

正常情况下，`dist` 应该和 `expect_z` 大致接近。如果 `expect_z` 是几十厘米或一米，而 `dist` 只有几毫米，说明位姿解算或配置仍有问题。

默认坐标转换适用于：摄像头朝下，图像上方对应飞机前方。

如果验证方向不对，改 `config/lubancat0n.json` 里的 `transform.camera_to_body`。验证标准：

- Tag 放在飞机前方，`x_body` 应该为正
- Tag 放在飞机右侧，`y_body` 应该为正
- Tag 在摄像头下方，`z_body` 应该为正

## 7. 干运行 LANDING_TARGET

先不连接飞控，只打印即将发送的数据：

```bash
PYTHONPATH=src python3 tools/landing_target.py --config config/lubancat0n.json --dry-run
```

当前发送格式是 ArduPilot 精准降落文档里的 MAVLink2 `LANDING_TARGET` 位置/四元数字段：

```text
MAVLink: 2
message id: 149 LANDING_TARGET
frame: MAV_FRAME_BODY_FRD = 12
x/y/z: 目标在机体系下的位置，单位 m
q: 目标姿态四元数，顺序为 w/x/y/z
type: LANDING_TARGET_TYPE_VISION_FIDUCIAL
position_valid: 1
```

这和旧 OpenMV 示例里的 MAVLink1 角度模式不同。旧方式主要填：

```text
angle_x / angle_y / distance
```

现在这版会同时填角度字段和 MAVLink2 扩展字段：

```text
angle_x / angle_y / distance
x / y / z / q / type / position_valid
```

`angle_x` 和 `angle_y` 由机体系 `x/z`、`y/z` 反算得到，单位是弧度；`q` 不再固定为 `[1, 0, 0, 0]`，而是由 AprilTag 四个角点解算出的目标姿态转换得到。

可以不接飞控先检查打包结果：

```bash
PYTHONPATH=src python3 tools/inspect_landing_target_packet.py --config config/lubancat0n.json
```

如果看到：

```text
packet_magic: 0xfd expected: 0xfd
message_id: 149
frame: 12 MAV_FRAME_BODY_FRD
position_valid: 1
```

说明打包为 MAVLink2。

如果检查脚本无法运行，先升级 `pymavlink`：

```bash
python3 -m pip install --upgrade pymavlink
```

## 8. 连接飞控发送

确认串口连接、飞控 TELEM 参数、权限都正确后运行：

```bash
PYTHONPATH=src python3 tools/landing_target.py --config config/lubancat0n.json
```

飞控对应串口也要配置成 MAVLink2，例如 ArduPilot 常见配置：

```text
SERIALx_PROTOCOL = 2
SERIALx_BAUD = 115
```

如果串口权限不够：

```bash
sudo usermod -aG dialout $USER
```

重新登录后再运行。

## 9. 位置信息不对时的调试流程

如果 Mission Planner 里看到 `LANDING_TARGET` 的 `x/y/z/distance` 明显不对，例如距离只有几毫米，按下面顺序排查。

### 9.1 确认摄像头分辨率和内参一致

先确认程序实际打开到的分辨率：

```bash
PYTHONPATH=src python3 tools/test_camera.py --config config/lubancat0n.json --headless
```

输出里会显示实际 `width x height`，并保存 `camera_test.jpg`。`config/lubancat0n.json` 里的 `width/height/fx/fy/cx/cy` 必须和标定时的分辨率一致。

如果你配置的是 `1280x720`，但输出显示类似：

```text
camera opened: 3264x2160 fps=30.0
```

说明摄像头实际没有按配置分辨率输出。MIPI/RKISP 摄像头建议使用上面的 GStreamer 管线，让实际打开分辨率和标定分辨率一致。

### 9.2 先不接飞控，检查识别和位姿

运行：

```bash
PYTHONPATH=src python3 tools/test_tags.py --config config/lubancat0n.json --headless --print-every 0.5
```

看输出里的：

```text
id / size / px / expect_z / cam / body / dist / margin / raw
```

检查顺序：

- `id` 是否是你想用的 tag
- `size` 是否等于打印出来后实测的 tag 边长，单位是米
- `raw` 是否持续大于 0；如果一会儿有一会儿没有，说明识别不连续
- `margin` 是否太低；太低通常是 tag 太小、模糊、反光或距离太远
- `dist` 是否和 `expect_z` 接近；两者差很多时优先检查尺寸和内参

如果使用嵌套 tag，程序会在所有已接受标签中选择配置尺寸最大的一个。远距离测试时小 tag 很容易识别不稳定；如果你只想测试某一个码，可以先只保留那个 tag 的尺寸配置，例如只保留 `0`：

```json
"tag_sizes_m": {
  "0": 0.078
}
```

### 9.3 干运行检查即将发送的数据

确认 `test_tags.py` 的 `dist/body` 正常后，再运行：

```bash
PYTHONPATH=src python3 tools/landing_target.py --config config/lubancat0n.json --dry-run
```

这里打印出来的 `x/y/z/dist` 应该和 `test_tags.py` 接近。若这里正常，说明识别和打包前的数据正常。

### 9.4 检查 MAVLink2 打包格式

```bash
PYTHONPATH=src python3 tools/inspect_landing_target_packet.py --config config/lubancat0n.json
```

确认：

```text
packet_magic: 0xfd
message_id: 149
frame: 12 MAV_FRAME_BODY_FRD
position_valid: 1
```

### 9.5 最后再接飞控发送

只有前面都正常后，再运行正式发送：

```bash
PYTHONPATH=src python3 tools/landing_target.py --config config/lubancat0n.json
```

如果飞控一开始收到、过一会儿收不到，通常是识别中断。因为当前逻辑是识别不到 tag 就不发送旧数据，避免飞控继续追踪过期目标。

### 9.6 程序像死机一样卡住时

如果终端不再刷新，`Ctrl+C` 也没有反应，通常说明程序卡在 OpenCV/GStreamer/V4L2/串口这类底层阻塞调用里。可以加 watchdog 参数定位卡在哪一步：

```bash
PYTHONPATH=src python3 tools/test_tags.py --config config/lubancat0n.json --headless --print-every 0.5 --watchdog-timeout 10
```

正式发送时也可以用：

```bash
PYTHONPATH=src python3 tools/landing_target.py --config config/lubancat0n.json --watchdog-timeout 10
```

如果卡住超过 10 秒，终端会打印当前调用栈。常见判断：

- 卡在 `cap.read()`：摄像头取流/GStreamer/RKISP 阻塞，优先检查分辨率、GStreamer 管线和 CPU 压力
- 卡在 `tracker.detect()`：AprilTag 检测耗时异常，优先降低分辨率、增大 tag、或调大 `quad_decimate`
- 卡在 `sender.send()`：串口写入阻塞，优先检查飞控串口连接、波特率和权限

当前代码已经给摄像头读取加了超时保护：主循环不会直接卡死在 `cap.read()`，如果超过 2 秒没有收到新帧，会打印：

```text
camera read timeout after 2.0s; restarting camera
```

然后自动释放并重开摄像头。这个时间可以手动调整，例如：

```bash
PYTHONPATH=src python3 tools/landing_target.py --config config/lubancat0n.json --camera-read-timeout 3
```

如果仍然频繁重启，说明底层取流链路不稳定，优先改用 GStreamer 1280x720 管线、降低分辨率，或者检查 MIPI/RKISP 驱动状态。

注意：如果 `backend` 是 `gstreamer` 且 `device` 是一整条 GStreamer 管线，程序不会再用 OpenCV 的 GStreamer `cap.read()` 取流。代码会把配置里的 `appsink` 管线自动转换成 `fdsink fd=1`，启动 `gst-launch-1.0` 直接输出 BGR 原始帧，Python 只读取 stdout。这样复用 `gst-launch-1.0` 单独测试稳定的取流链路，同时绕开 OpenCV GStreamer 后端卡死的问题。

如果 `Ctrl+C` 无法结束，可以另开一个终端查看进程状态：

```bash
ps -o pid,stat,wchan,cmd -C python3
```

如果 `STAT` 里有 `D`，表示进程卡在内核不可中断 I/O，通常是摄像头驱动或底层取流阻塞，`kill -9` 也可能暂时无效，需要停止相关服务、重新插拔摄像头或重启板子。

## 重要注意

- 当前代码会在识别到的 tag 中按 `0 -> 1 -> 2` 选择目标；你的嵌套图案里这等价于优先发送最大码的位置。
- 没识别到 Tag 时程序不会发送旧数据，避免飞控继续追一个过期目标。
- 相机标定主要影响 `x/y/z` 位姿精度，不是识别连续性的主要原因。识别连续性更依赖 tag 在画面中的像素大小、清晰度、曝光、反光和 CPU 负载。
- 30 mm 小码距离远时像素太少，不可能稳定识别。估算公式是 `tag_pixels ~= fx * tag_size_m / distance_m`。
- 如果加了 `fourcc` 后摄像头打不开，先把 `config/lubancat0n.json` 里的 `"fourcc"` 改成空字符串 `""`。只有确认摄像头支持 MJPG 时再设置 `"MJPG"`。
- 上机测试前必须拆桨。

## Git 提交和推送流程

第一次把本地代码提交到 GitHub，一般是这几步：

```bash
git status
git add .
git commit -m "Initial AprilTag landing target implementation"
git push -u origin main
```

后续每次改完代码：

```bash
git status
git add 修改过的文件
git commit -m "说明这次改了什么"
git push
```

提交前建议看一下改动：

```bash
git diff
git status
```

## 如何包含 AprilTag 依赖

这个项目当前用 Python 开发，所以 AprilTag 依赖通过 `requirements.txt` 声明：

```text
pupil-apriltags>=1.0.4.post10
```

也就是说，仓库里提交的是“依赖清单”，不是把 `pupil-apriltags` 源码、虚拟环境 `venv/`、`site-packages/` 一起提交进去。别人克隆仓库后执行：

```bash
python3 -m pip install -r requirements.txt
```

就会自动安装 AprilTag 识别包。

如果以后要换成 C++ 官方 AprilTag 库，可以用 Git submodule 引入源码：

```bash
git submodule add https://github.com/AprilRobotics/apriltag.git third_party/apriltag
git commit -m "Add AprilTag as submodule"
```

但当前 Python 版本不需要这样做。
