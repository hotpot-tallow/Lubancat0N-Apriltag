from __future__ import annotations

import faulthandler
import sys


def enable_watchdog(timeout_s: float) -> None:
    """启用卡死定位工具，超时后周期性打印当前 Python 调用栈。"""
    if timeout_s <= 0.0:
        return
    faulthandler.enable(file=sys.stderr, all_threads=True)
    faulthandler.dump_traceback_later(timeout_s, repeat=True, file=sys.stderr)
    print(f"watchdog enabled: dump traceback every {timeout_s:.1f}s if blocked")
