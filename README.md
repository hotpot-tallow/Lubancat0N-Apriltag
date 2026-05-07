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
pip install --upgrade pip setuptools wheel
```

## 3. 安装 AprilTag 和 MAVLink 包

本项目使用 `pupil-apriltags` 做 AprilTag 识别和位姿估计，使用 `pymavlink` 发送 `LANDING_TARGET`。

推荐直接安装：

```bash
pip install pupil-apriltags pymavlink pyserial
```

或者使用仓库里的依赖文件：

```bash
pip install -r requirements.txt
```

安装完成后检查：

```bash
python -c "from pupil_apriltags import Detector; print('pupil-apriltags ok')"
python -c "from pymavlink import mavutil; print('pymavlink ok')"
```

如果 `pupil-apriltags` 安装失败，通常是缺少编译工具，先确认已经执行过：

```bash
sudo apt install -y python3-dev build-essential cmake pkg-config
```

然后重新运行：

```bash
pip install --no-cache-dir pupil-apriltags
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
  "fx": 600.0,
  "fy": 600.0,
  "cx": 320.0,
  "cy": 240.0
},
"mavlink": {
  "connection": "/dev/ttyS8",
  "baud": 115200
}
```

`fx/fy/cx/cy` 必须换成你自己的相机标定结果。没有内参时可以先用近似值跑通识别，但不要直接拿去飞。

你的嵌套码尺寸在 `tag_sizes_m` 里：

```json
"tag_sizes_m": {
  "0": 0.5,
  "1": 0.128,
  "2": 0.03
}
```

这里单位是米，填的是 AprilTag 检测边界实际边长。

## 5. 测摄像头

有桌面环境：

```bash
PYTHONPATH=src python tools/test_camera.py --config config/lubancat0n.json
```

无桌面环境：

```bash
PYTHONPATH=src python tools/test_camera.py --config config/lubancat0n.json --headless
```

成功后会保存 `camera_test.jpg`。

## 6. 测 AprilTag

有桌面环境时直接运行，会弹出实时画面，画面左上角显示 FPS，识别到 tag 后会画出绿色边框、tag ID、坐标和距离：

```bash
PYTHONPATH=src python tools/test_tags.py --config config/lubancat0n.json
```

按 `Esc` 或 `q` 退出。

无桌面环境时使用：

```bash
PYTHONPATH=src python tools/test_tags.py --config config/lubancat0n.json --headless
```

看到类似输出就说明识别和位姿估计通了：

```text
id=0 size=0.500m cam=(+0.012,-0.035,+1.420) body=(+0.035,+0.012,+1.420) dist=1.421 margin=84.2
```

默认坐标转换适用于：摄像头朝下，图像上方对应飞机前方。

如果验证方向不对，改 `config/lubancat0n.json` 里的 `transform.camera_to_body`。验证标准：

- Tag 放在飞机前方，`x_body` 应该为正
- Tag 放在飞机右侧，`y_body` 应该为正
- Tag 在摄像头下方，`z_body` 应该为正

## 7. 干运行 LANDING_TARGET

先不连接飞控，只打印即将发送的数据：

```bash
PYTHONPATH=src python tools/landing_target.py --config config/lubancat0n.json --dry-run
```

## 8. 连接飞控发送

确认串口连接、飞控 TELEM 参数、权限都正确后运行：

```bash
PYTHONPATH=src python tools/landing_target.py --config config/lubancat0n.json
```

如果串口权限不够：

```bash
sudo usermod -aG dialout $USER
```

重新登录后再运行。

## 重要注意

- 当前代码会按 `tag_priority` 选择目标，默认 `[2, 1, 0]`，也就是小码优先。想让大码优先就改成 `[0, 1, 2]`。
- 没识别到 Tag 时程序不会发送旧数据，避免飞控继续追一个过期目标。
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
pip install -r requirements.txt
```

就会自动安装 AprilTag 识别包。

如果以后要换成 C++ 官方 AprilTag 库，可以用 Git submodule 引入源码：

```bash
git submodule add https://github.com/AprilRobotics/apriltag.git third_party/apriltag
git commit -m "Add AprilTag as submodule"
```

但当前 Python 版本不需要这样做。
