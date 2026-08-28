# -*- coding: utf-8 -*-
"""救援控制台网关（FastAPI + WebSocket）。

业务目的：
  在小车侧提供一层 HTTP + WebSocket 中间服务，把 ROS2 话题/动作封装为
  面向浏览器的 JSON 协议，客户端不依赖任何 ROS 库。桥接层固定使用
  RosBridge，直接读写目标机上的真实 ROS2 话题与 Nav2 动作。

启动（必须先 source ROS2 与 colcon 工作空间，否则 rclpy 导入失败）：
  source /opt/ros/humble/setup.bash
  source ~/mini_car_ws/install/setup.bash
  cd rescue_console/server
  python -m uvicorn app:app --host 0.0.0.0 --port 8000

接口一览：
  GET  /                     静态页面（../web/index.html）
  GET  /api/status           桥接模式与连接概况
  POST /api/cmd_vel          手动遥控 {vx, vy, wz}，米/秒、弧度/秒
  POST /api/nav_goal         导航目标 {x, y}，地图坐标（米）
  POST /api/cancel_nav       取消导航并停车
  WS   /ws/telemetry         遥测推送：telemetry 5Hz + 地图约 1Hz（有更新时）
  GET  /video/stream         MJPEG 彩色画面（/camera/color/image_raw，限流 10fps）

安全约束（与 AGENTS.md 一致）：
  - 手动遥控带 0.5 秒看门狗，客户端断开即自动停车；
  - 切换到 ROS2 桥接并连接实车前，必须车轮悬空或场地清空。
"""

from __future__ import annotations

import asyncio
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from pydantic import BaseModel, Field

from bridge import BaseBridge, create_bridge

WEB_DIR = Path(__file__).resolve().parent.parent / "web"

bridge: BaseBridge = create_bridge()

# 连接管理：所有 WebSocket 客户端共享同一份广播
_clients: set[WebSocket] = set()
_started_at = time.time()


@asynccontextmanager
async def lifespan(_app: FastAPI):
    """启动时创建桥接主循环任务，退出时停止任务并释放桥接资源。"""
    worker = asyncio.create_task(_tick_loop())
    yield
    worker.cancel()
    close = getattr(bridge, "close", None)
    if close:
        close()


app = FastAPI(title="救援控制台网关", version="0.1.0", lifespan=lifespan)


class CmdVelBody(BaseModel):
    """手动遥控指令体。限幅由桥接层负责，这里只做类型校验。"""

    vx: float = Field(default=0.0, ge=-2.0, le=2.0)
    vy: float = Field(default=0.0, ge=-2.0, le=2.0)
    wz: float = Field(default=0.0, ge=-3.0, le=3.0)


class NavGoalBody(BaseModel):
    x: float = Field(ge=-100.0, le=100.0)
    y: float = Field(ge=-100.0, le=100.0)


async def _tick_loop() -> None:
    """主循环：10Hz 推进桥接状态，5Hz 广播遥测，地图按需推送。"""
    period = 0.1
    counter = 0
    last_map_sent = 0.0
    while True:
        started = time.monotonic()
        try:
            bridge.tick(period)
            counter += 1
            # 每 2 个周期（5Hz）广播一次遥测
            if counter % 2 == 0 and _clients:
                msg = bridge.telemetry()
                dead = []
                for ws in list(_clients):
                    try:
                        await ws.send_json(msg)
                    except Exception:
                        dead.append(ws)
                for ws in dead:
                    _clients.discard(ws)
            # 地图约 1Hz 且仅在栅格有更新时推送
            if time.monotonic() - last_map_sent > 1.0 and _clients:
                map_msg = bridge.map_message()
                if map_msg is not None:
                    last_map_sent = time.monotonic()
                    dead = []
                    for ws in list(_clients):
                        try:
                            await ws.send_json(map_msg)
                        except Exception:
                            dead.append(ws)
                    for ws in dead:
                        _clients.discard(ws)
        except Exception as exc:  # 模拟循环不允许静默崩溃
            print(f"[tick_loop] 桥接循环异常: {exc!r}")
        # 补偿耗时，保持周期稳定
        elapsed = time.monotonic() - started
        await asyncio.sleep(max(0.0, period - elapsed))


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(WEB_DIR / "index.html")


@app.get("/api/status")
async def status() -> dict:
    return {
        "bridge": bridge.name,
        "clients": len(_clients),
        "uptime_s": round(time.time() - _started_at, 1),
        "note": "ROS2 桥接（真实话题）",
        "video": bridge.video_status(),
        # 遥操指令的实际发布话题，用于排查“发了指令但小车不动”：
        # 部署 twist_mux 后底盘只接收仲裁输出，直连 /cmd_vel 会失效。
        "cmd_vel_topic": getattr(bridge, "cmd_vel_topic", "/cmd_vel"),
    }


async def _mjpeg_chunks():
    """MJPEG 分帧字节块异步生成器（multipart/x-mixed-replace）。

    按帧序号去重：同一帧只推一次，所有客户端复用同一份 JPEG 缓存，
    因此 N 个浏览器只付一次编码成本。无新帧时不发送任何数据，避免
    用重复帧空耗带宽。

    必须是异步生成器：无新帧时 await 让出事件循环，否则同步 while True
    会变成忙循环吃满一个 CPU 核。
    """
    last_seq = -1
    while True:
        frame = bridge.video_frame()
        if frame is not None and frame[0] != last_seq:
            seq, jpeg = frame
            last_seq = seq
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n"
                   b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n"
                   b"\r\n" + jpeg + b"\r\n")
        else:
            # 20ms 轮询，短于编码周期（1/_VIDEO_FPS），保证新帧及时取到
            await asyncio.sleep(0.02)


@app.get("/video/stream")
async def video_stream():
    """彩色实时画面（MJPEG）。前端用 <img src="/video/stream"> 直接播放。"""
    return StreamingResponse(
        _mjpeg_chunks(),
        media_type="multipart/x-mixed-replace;boundary=frame",
    )


@app.post("/api/cmd_vel")
async def cmd_vel(body: CmdVelBody) -> dict:
    bridge.cmd_vel(body.vx, body.vy, body.wz)
    return {"ok": True}


@app.post("/api/nav_goal")
async def nav_goal(body: NavGoalBody) -> dict:
    bridge.nav_goal(body.x, body.y)
    return {"ok": True}


@app.post("/api/cancel_nav")
async def cancel_nav() -> dict:
    bridge.cancel_nav()
    return {"ok": True}


@app.websocket("/ws/telemetry")
async def telemetry_ws(ws: WebSocket) -> None:
    await ws.accept()
    _clients.add(ws)
    # 连接建立后立即补发一帧遥测与当前地图，缩短首屏等待
    try:
        await ws.send_json(bridge.telemetry())
        # force=True：新客户端立即补发缓存地图，不等待下一帧更新
        map_msg = bridge.map_message(force=True)
        if map_msg is not None:
            await ws.send_json(map_msg)
        while True:
            # 客户端可在 WS 上发 {"type":"ping"} 保活，不做其他处理
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        _clients.discard(ws)
