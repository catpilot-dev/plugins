"""WebRTCSession — portable aiortc-native WebRTC session.

This is the canonical implementation of a webrtcd session that runs on
pure aiortc with no teleoprtc dependency.  It lives in the plugins repo so
it can be installed on any openpilot fork that exposes the
webrtc.session_factory hook, not just catpilot.

Drop-in replacement for catpilot's default StreamSession:
  same constructor signature, same public interface.
"""

import asyncio
import uuid
import logging
from typing import Optional, TYPE_CHECKING

from openpilot.system.webrtc.cereal_bridge import (
  CerealOutgoingMessageProxy, CerealIncomingMessageProxy,
  CerealProxyRunner, DynamicPubMaster,
)
from openpilot.system.webrtc.sdp import strip_to_h264, parse_offer_info
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.plugins.hooks import hooks
from cereal import messaging

if TYPE_CHECKING:
  from aiortc.rtcdatachannel import RTCDataChannel


class WebRTCSession:
  """Minimal WebRTC session — pure aiortc, zero teleoprtc dependency.

  Manages one RTCPeerConnection:
  - Sends H264 camera streams from the cereal encode pipeline
  - Optionally sends microphone audio
  - Optionally receives and plays back client audio
  - Bridges a WebRTC data channel to/from cereal pub/sub services

  The session lifecycle is:
    session = WebRTCSession(offer_sdp, cameras, ...)
    answer  = await session.get_answer()   # SDP exchange
    session.start()                        # begin async run loop
    # ... peer connects, runs, eventually disconnects ...
    # run loop exits, session_ended hook fires
  """

  shared_pub_master = DynamicPubMaster([])

  def __init__(self, sdp: str, cameras: list[str],
               incoming_services: list[str], outgoing_services: list[str],
               debug_mode: bool = False):
    import aiortc
    from aiortc.contrib.media import MediaRelay, MediaBlackhole
    from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack
    from openpilot.system.webrtc.device.audio import AudioInputStreamTrack, AudioOutputSpeaker

    n_video, wants_audio_out, has_audio_in, has_datachannel = parse_offer_info(sdp)
    assert len(cameras) == n_video, \
      f"Offer has {n_video} video track(s) but {len(cameras)} camera(s) requested"

    self._pc    = aiortc.RTCPeerConnection()
    self._relay = MediaRelay()
    self._offer = sdp
    self.identifier = str(uuid.uuid4())

    # ── Outgoing tracks (device → client) ────────────────────────────────────
    self._video_tracks = [
      LiveStreamVideoStreamTrack(c) if not debug_mode else aiortc.mediastreams.VideoStreamTrack()
      for c in cameras
    ]
    self._audio_out: Optional[aiortc.MediaStreamTrack] = (
      (AudioInputStreamTrack() if not debug_mode else aiortc.mediastreams.AudioStreamTrack())
      if wants_audio_out else None
    )

    # ── Incoming audio (client → device speaker) ──────────────────────────────
    self._has_audio_in    = has_audio_in
    self._audio_output_cls = AudioOutputSpeaker if not debug_mode else MediaBlackhole
    self._audio_output    = None
    self._incoming_audio_track = None

    # ── Data channel — the client creates it, we receive it ───────────────────
    self._messaging_channel: Optional[RTCDataChannel] = None

    # ── Lifecycle events ──────────────────────────────────────────────────────
    self._connected    = asyncio.Event()
    self._failed       = asyncio.Event()
    self._disconnected = asyncio.Event()
    self._channel_open = asyncio.Event()

    # Fire _incoming_ready once every expected incoming item has arrived.
    # Expected = data channel (if offer contains one) + incoming audio (if any).
    _expected = int(has_datachannel) + int(has_audio_in)
    self._incoming_ready  = asyncio.Event()
    self._incoming_count  = 0
    self._expected_incoming = _expected
    if _expected == 0:
      self._incoming_ready.set()

    # ── RTCPeerConnection callbacks ───────────────────────────────────────────
    @self._pc.on("connectionstatechange")
    async def _on_state():
      s = self._pc.connectionState
      self.logger.debug("session %s: connectionState=%s", self.identifier, s)
      if s == "connected":
        self._connected.set()
      if s == "failed":
        self._failed.set()
      if s in ("disconnected", "closed", "failed"):
        self._disconnected.set()

    @self._pc.on("track")
    def _on_track(track):
      if track.kind == "audio" and self._has_audio_in:
        self._incoming_audio_track = self._relay.subscribe(track, buffered=False)
        self._incoming_count += 1
        if self._incoming_count >= self._expected_incoming:
          self._incoming_ready.set()

    @self._pc.on("datachannel")
    def _on_datachannel(channel):
      if channel.label != "data":
        return
      self._messaging_channel = channel

      def _opened():
        self._channel_open.set()
        self._incoming_count += 1
        if self._incoming_count >= self._expected_incoming:
          self._incoming_ready.set()

      if channel.readyState == "open":
        _opened()
      else:
        channel.on("open", _opened)

    # ── Cereal bridges ────────────────────────────────────────────────────────
    self._incoming_bridge   = CerealIncomingMessageProxy(self.shared_pub_master) if incoming_services else None
    self._incoming_services = incoming_services
    self._outgoing_bridge: Optional[CerealOutgoingMessageProxy] = None
    self._outgoing_runner:  Optional[CerealProxyRunner]         = None
    if outgoing_services:
      self._outgoing_bridge = CerealOutgoingMessageProxy(messaging.SubMaster(outgoing_services))
      self._outgoing_runner = CerealProxyRunner(self._outgoing_bridge)

    self.run_task: Optional[asyncio.Task] = None
    self.logger = logging.getLogger("webrtcd")
    self.logger.info(
      "new session (%s) cameras=%s audio_out=%s audio_in=%s in=%s out=%s",
      self.identifier, cameras, wants_audio_out, has_audio_in,
      incoming_services, outgoing_services,
    )
    cloudlog.info(f"webrtcd: new session {self.identifier} cameras={cameras}")

  async def get_answer(self):
    """Perform SDP offer/answer and return the local answer description."""
    import aiortc

    # Strip offer SDP to H264-only: our tracks deliver raw H264 from cereal
    patched = strip_to_h264(self._offer)
    await self._pc.setRemoteDescription(
      aiortc.RTCSessionDescription(sdp=patched, type="offer")
    )

    # Add outgoing video tracks; lock each transceiver to H264 as belt-and-suspenders
    for track in self._video_tracks:
      sender = self._pc.addTrack(track)
      pref = getattr(track, "codec_preference", lambda: None)()
      if pref:
        transceiver = next(
          (t for t in self._pc.getTransceivers() if t.sender == sender), None
        )
        if transceiver:
          caps = [
            c for c in aiortc.RTCRtpSender.getCapabilities("video").codecs
            if c.mimeType.upper() == f"VIDEO/{pref.upper()}"
          ]
          if caps:
            transceiver.setCodecPreferences(caps)

    if self._audio_out:
      self._pc.addTrack(self._audio_out)

    answer = await self._pc.createAnswer()
    await self._pc.setLocalDescription(answer)
    return self._pc.localDescription

  def get_messaging_channel(self) -> Optional['RTCDataChannel']:
    return self._messaging_channel

  async def _on_message(self, message: bytes):
    assert self._incoming_bridge is not None
    try:
      self._incoming_bridge.send(message)
    except Exception:
      self.logger.exception("session %s: cereal incoming error", self.identifier)

  def start(self):
    self.run_task = asyncio.create_task(self.run())

  def stop(self):
    if self.run_task and not self.run_task.done():
      self.run_task.cancel()

  async def run(self):
    try:
      # Wait for ICE to settle
      connected_task = asyncio.create_task(self._connected.wait())
      failed_task    = asyncio.create_task(self._failed.wait())
      done, pending  = await asyncio.wait(
        {connected_task, failed_task},
        return_when=asyncio.FIRST_COMPLETED,
      )
      for t in pending:
        t.cancel()

      if self._failed.is_set() and not self._connected.is_set():
        raise ConnectionError("ICE negotiation failed")

      # Wait for all expected incoming media to be ready
      await self._incoming_ready.wait()

      # Wire up cereal ↔ data-channel bridge
      if self._messaging_channel is not None:
        if self._incoming_bridge is not None:
          await self.shared_pub_master.add_services_if_needed(self._incoming_services)
          self._messaging_channel.on("message", self._on_message)
        if self._outgoing_runner is not None:
          self._outgoing_bridge.add_channel(self._messaging_channel)
          self._outgoing_runner.start()

      # Wire up incoming audio → speaker
      if self._incoming_audio_track is not None:
        self._audio_output = self._audio_output_cls()
        self._audio_output.addTrack(self._incoming_audio_track)
        self._audio_output.start()

      self.logger.info("session (%s) connected", self.identifier)
      cloudlog.info(f"webrtcd: session {self.identifier} connected")
      hooks.run('webrtc.session_started', None, self.identifier)

      await self._disconnected.wait()
      await self._cleanup()

      self.logger.info("session (%s) ended", self.identifier)
      cloudlog.info(f"webrtcd: session {self.identifier} ended")
      hooks.run('webrtc.session_ended', None, self.identifier)

    except asyncio.CancelledError:
      await self._cleanup()
      raise
    except Exception as e:
      self.logger.exception("session (%s) error", self.identifier)
      cloudlog.error(f"webrtcd: session {self.identifier} failed: {e}")
      await self._cleanup()

  async def _cleanup(self):
    await self._pc.close()
    if self._outgoing_runner:
      self._outgoing_runner.stop()
    if self._audio_output:
      self._audio_output.stop()
