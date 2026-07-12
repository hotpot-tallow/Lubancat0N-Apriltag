# Raspberry Pi 4B + Camera Module 2 迁移

这套配置使用 Raspberry Pi OS Lite 64-bit、Picamera2 和 Camera Module 2（IMX219）。
AprilTag、PnP 和 MAVLink 逻辑与鲁班猫版本共用，只有摄像头后端和设备配置不同。

## 1. 先验证官方摄像头链路

```bash
rpicam-hello --list-cameras
rpicam-still --nopreview --timeout 2000 --width 640 --height 480 --output ~/camera_test.jpg
```

第一条命令应列出 `imx219`。如果提示命令不存在：

```bash
sudo apt update
sudo apt install -y rpicam-apps-lite
```

## 2. 安装系统依赖

```bash
sudo apt update
sudo apt install -y \
  git python3-venv python3-pip python3-dev build-essential cmake pkg-config \
  python3-opencv python3-numpy python3-picamera2 v4l-utils
```

Picamera2 和 OpenCV 使用 Raspberry Pi OS 的 apt 包。虚拟环境必须带
`--system-site-packages`，否则看不到这些系统包。

## 3. 获取代码并安装 Python 依赖

首次安装：

```bash
cd ~
git clone --branch raspberry-pi-4b https://github.com/hotpot-tallow/Lubancat0N-Apriltag.git
cd Lubancat0N-Apriltag
python3 -m venv venv --system-site-packages
source venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
python -m pip install -r requirements.txt
```

已经克隆过仓库时：

```bash
cd ~/Lubancat0N-Apriltag
git switch raspberry-pi-4b
git pull
source venv/bin/activate
python -m pip install -r requirements.txt
```

## 4. 创建树莓派配置

```bash
cp config/pi4b.example.json config/pi4b.json
```

模板默认使用：

- Camera Module 2 的第 0 个相机。
- `1280x960@30` 的 YUV420 图像，只把灰度 Y 平面交给 AprilTag。
- `nthreads=3`，给取流、PnP 和系统保留一个 CPU 核心。
- `/dev/serial0@115200` 连接飞控。

模板里的 `fx=1060`、`fy=1060`、`cx=640`、`cy=480` 只是根据标称视场角得到的
启动近似值。它们只能用于检查识别方向和大致距离，实飞前必须用最终分辨率重新标定。

## 5. 分层测试

每次只运行一个程序，摄像头不能同时被两个进程占用。

```bash
cd ~/Lubancat0N-Apriltag
source venv/bin/activate

PYTHONPATH=src python tools/test_camera.py \
  --config config/pi4b.json --headless

PYTHONPATH=src python tools/test_tags.py \
  --config config/pi4b.json --headless --print-every 0.2

PYTHONPATH=src python tools/inspect_landing_target_packet.py \
  --config config/pi4b.json

PYTHONPATH=src python tools/landing_target.py \
  --config config/pi4b.json --dry-run
```

`test_camera.py` 会保存灰度的 `camera_test.jpg`。`test_tags.py` 应持续打印
`id`、`px`、`margin`、`detect_ms` 和相机/机体坐标。按 `Ctrl+C` 结束当前测试后，
再运行下一项。

## 6. 启用飞控串口

如果使用 Pi 4B GPIO UART，而不是 USB 转串口：

```bash
sudo raspi-config
```

在串口选项中关闭 login shell，启用 serial hardware。然后执行：

```bash
sudo usermod -aG dialout "$USER"
sudo reboot
```

重新登录后检查：

```bash
ls -l /dev/serial0
groups
```

接线必须是 Pi TX 接飞控 RX、Pi RX 接飞控 TX、GND 接 GND。两边都是 3.3V TTL，
不要把飞控 TELEM 口的 5V 电源线接到树莓派 5V。

## 7. 实际发送与自启动

先在桨叶拆除、飞控不解锁的状态下发送：

```bash
PYTHONPATH=src python tools/landing_target.py --config config/pi4b.json
```

确认飞控收到 `MAV_FRAME_BODY_FRD` 的 `LANDING_TARGET`，并且前后左右移动 Tag 时
`x/y` 符号正确，再安装自启动：

```bash
sudo bash scripts/install_autostart.sh \
  --config "$HOME/Lubancat0N-Apriltag/config/pi4b.json" \
  --service-name pi4b-apriltag
```

查看状态和日志：

```bash
systemctl status pi4b-apriltag.service
journalctl -u pi4b-apriltag.service -f
```

## 8. 实飞前必须完成

1. 在 `1280x960` 下标定 Camera Module 2 的内参。
2. 实测相机到机体的安装方向，确认 `camera_to_body` 矩阵和 `x/y/z` 符号。
3. 记录近、中、远距离的 `px`、`margin`、`detect_ms` 和丢失率。
4. 固定曝光和增益，减少飞行振动造成的运动模糊。
5. 先做手持移动、台架和 Precision Loiter，再进行 Precision Landing。
