"""aiortc WebRTC — ROS 카메라 프레임을 브라우저로 송출.

시그널링은 기존 WebSocket 채널 재활용(app.py). 전용망(LAN)이라 non-trickle ICE,
STUN/TURN 없음. 아래 3개가 핵심:
  - FrameSource      : ROS 스레드가 최신 프레임을 넣고, 트랙이 꺼내가는 핸드오프
  - RosImageTrack    : VideoStreamTrack, recv()에서 av.VideoFrame 반환
  - WebRTCManager    : 클라이언트별 RTCPeerConnection 생성/정리 + offer→answer
"""
from __future__ import annotations

import asyncio
import fractions
import threading
import time
from typing import Optional

import numpy as np
from av import VideoFrame
from aiortc import (RTCConfiguration, RTCIceServer, RTCPeerConnection,
                    RTCSessionDescription)
from aiortc.mediastreams import MediaStreamTrack

# localhost/LAN 이어도 브라우저 ICE가 STUN을 요구하는 경우가 있어 하나 지정
ICE_SERVERS = [RTCIceServer(urls="stun:stun.l.google.com:19302")]

VIDEO_CLOCK_RATE = 90000
VIDEO_FPS = 15  # 송출 목표 fps (전시 데모엔 15~20이면 충분)


class FrameSource:
    """rclpy 스레드(쓰기)와 asyncio 트랙(읽기) 사이의 스레드 안전 최신-프레임 홀더.

    큐가 아니라 '최신 1장'만 유지 → 늦은 소비자가 밀린 프레임 따라잡느라
    지연 누적되는 걸 방지 (실시간 영상엔 최신만 중요).
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None  # HxWx3, RGB

    def put(self, frame_rgb: np.ndarray) -> None:
        with self._lock:
            self._frame = frame_rgb

    def get(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._frame


class RosImageTrack(MediaStreamTrack):
    """FrameSource의 최신 프레임을 VIDEO_FPS로 송출하는 트랙."""

    kind = "video"

    def __init__(self, source: FrameSource) -> None:
        super().__init__()
        self._source = source
        self._start = time.time()
        self._count = 0

    async def recv(self) -> VideoFrame:
        # 목표 fps에 맞춰 페이싱 (다음 프레임 시각까지 sleep)
        self._count += 1
        target = self._start + self._count / VIDEO_FPS
        wait = target - time.time()
        if wait > 0:
            await asyncio.sleep(wait)

        arr = self._source.get()
        if arr is None:
            # 아직 카메라 프레임 없음 → 검은 화면 (RGB)
            arr = np.zeros((480, 640, 3), dtype=np.uint8)

        # TODO: ROS Image의 encoding이 bgr8이면 여기서 arr = arr[:, :, ::-1] 로 뒤집기
        frame = VideoFrame.from_ndarray(arr, format="rgb24")
        frame.pts = self._count
        frame.time_base = fractions.Fraction(1, VIDEO_FPS)
        return frame


async def _wait_ice_complete(pc: RTCPeerConnection) -> None:
    """non-trickle: ICE gathering이 끝나야 candidate가 SDP에 다 박힘."""
    if pc.iceGatheringState == "complete":
        return
    done = asyncio.Event()

    @pc.on("icegatheringstatechange")
    def _on_change() -> None:
        if pc.iceGatheringState == "complete":
            done.set()

    try:                       # STUN 지연 대비: 오래 안 끝나면 모은 후보로 진행
        await asyncio.wait_for(done.wait(), timeout=3.0)
    except asyncio.TimeoutError:
        pass


class WebRTCManager:
    """WS 시그널링 메시지를 받아 peer connection 수명주기를 관리."""

    def __init__(self, source: FrameSource) -> None:
        self._source = source
        # client_id(=WS 연결) 당 하나의 pc
        self._pcs: dict[str, RTCPeerConnection] = {}

    async def handle_offer(self, client_id: str, sdp: str, sdp_type: str) -> dict:
        """{type:"webrtc/offer"} 처리 → {type:"webrtc/answer"} 페이로드 반환."""
        await self.close(client_id)  # 재협상 시 기존 것 정리

        pc = RTCPeerConnection(RTCConfiguration(iceServers=ICE_SERVERS))
        self._pcs[client_id] = pc

        @pc.on("connectionstatechange")
        async def _on_state() -> None:
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await self.close(client_id)

        # ⑤ 영상 트랙 부착 (sendonly)
        pc.addTrack(RosImageTrack(self._source))

        await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=sdp_type))
        await pc.setLocalDescription(await pc.createAnswer())
        await _wait_ice_complete(pc)

        return {
            "type": "webrtc/answer",
            "sdp": pc.localDescription.sdp,
            "sdpType": pc.localDescription.type,
        }

    async def close(self, client_id: str) -> None:
        """{type:"webrtc/bye"} 또는 WS 종료 시 호출. 안 하면 인코딩 스레드 누수."""
        pc = self._pcs.pop(client_id, None)
        if pc is not None:
            await pc.close()

    async def close_all(self) -> None:
        await asyncio.gather(*(self.close(cid) for cid in list(self._pcs)))
