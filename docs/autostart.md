# 开机自启动

这个项目的正式运行入口是：

```bash
PYTHONPATH=src python tools/landing_target.py --config config/lubancat0n.json
```

开机自启动使用 `systemd` 管理，服务会自动启动 AprilTag 识别程序，并通过配置里的 MAVLink 串口向飞控发送 `LANDING_TARGET`。

## 1. 准备配置

在鲁班猫上进入项目目录：

```bash
cd ~/Lubancat0N-Apriltag
cp config/example_config.json config/lubancat0n.json
```

编辑 `config/lubancat0n.json`，重点确认：

```json
"mavlink": {
  "connection": "/dev/ttyS8",
  "baud": 115200
}
```

同时确认相机参数、AprilTag 尺寸和坐标变换已经按实际安装方式改好。

## 2. 先手动测试

建议先不接飞控测试识别和打包：

```bash
PYTHONPATH=src python tools/landing_target.py --config config/lubancat0n.json --dry-run
```

确认能识别 Tag 后，再接飞控运行：

```bash
PYTHONPATH=src python tools/landing_target.py --config config/lubancat0n.json
```

## 3. 安装开机自启动

```bash
sudo bash scripts/install_autostart.sh
```

如果配置文件不在默认位置，可以指定路径：

```bash
sudo bash scripts/install_autostart.sh --config /home/cat/Lubancat0N-Apriltag/config/lubancat0n.json
```

安装脚本会创建并启动 `lubancat-apriltag.service`。服务使用当前登录用户运行，并附加 `video`、`dialout` 组权限，便于访问摄像头和串口。

## 4. 查看运行状态

```bash
systemctl status lubancat-apriltag.service
journalctl -u lubancat-apriltag.service -f
```

如果启动失败，优先检查日志里是否有：

- 配置文件不存在
- 摄像头打不开
- 串口路径错误
- 串口权限不足
- Python 依赖未安装

## 5. 停用或重启

```bash
sudo systemctl restart lubancat-apriltag.service
sudo systemctl disable --now lubancat-apriltag.service
```

修改 `config/lubancat0n.json` 后，重启服务即可生效。
