# LubanCat 0N AprilTag Landing Target

鲁班猫 0N 读取摄像头画面，识别嵌套 `tag36h11` AprilTag，并向 ArduPilot 飞控发送 MAVLink `LANDING_TARGET` 消息。

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

如果 `NV12` 管线打不开，可以把 `format=NV12` 改成你用 `v4l2-ctl -d /dev/video0 --list-formats-ext` 查到的格式，例如 `UYVY`。

你的嵌套码尺寸在 `tag_sizes_m` 里：

```json
"tag_sizes_m": {
  "0": 0.5,
  "1": 0.128,
  "2": 0.03
}
```

这里单位是米，填的是 AprilTag 检测边界实际边长。程序会在当前画面识别到的 tag 里自动选择物理尺寸最小的一个，例如同时识别到 `0`、`1`、`2` 时，只使用 `2` 的位置。

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
id=0 size=0.500m px=312.5 expect_z=1.410 cam=(+0.012,-0.035,+1.420) body=(+0.035,+0.012,+1.420) dist=1.421 margin=84.2
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
q: [1, 0, 0, 0]
type: LANDING_TARGET_TYPE_VISION_FIDUCIAL
position_valid: 1
```

这和旧 OpenMV 示例里的 MAVLink1 角度模式不同。旧方式主要填：

```text
angle_x / angle_y / distance
```

现在这版会填 MAVLink2 扩展字段：

```text
x / y / z / q / type / position_valid
```

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

如果使用嵌套 tag，程序默认选择物理尺寸最小的 tag。远距离测试时小 tag 很容易识别不稳定，可以先只保留最大 tag 的尺寸配置，例如：

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

## 重要注意

- 当前代码会在识别到的 tag 中自动选择物理尺寸最小的目标，并且 `landing_target.py` 只发送这个最小目标的位置。
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
