import argparse
import asyncio
import json
import logging
import os
import uuid
import cv2
from aiohttp import web
import aiohttp_cors
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaRelay
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import threading

class WebRTCNode(Node):
    def __init__(self):
        super().__init__('webrtc_node')
        self.get_logger().info("Initializing WebRTCNode")
        self.publisher = None
        self.active_connections = 0
        self.relay = MediaRelay()
        self.pcs = set()
        
        # Initialize web server
        self.app = web.Application()
        self._setup_routes()
        self._setup_cors()
        
        # ROS2 parameters
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('verbose', False)
        
        # Create event loop in a separate thread
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_event_loop, daemon=True)
        self.thread.start()
        
        # Start web server in the event loop thread
        asyncio.run_coroutine_threadsafe(self._start_web_server(), self.loop)

    def _run_event_loop(self):
        self.get_logger().debug("Starting event loop thread")
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
        self.get_logger().debug("Event loop thread exited")

    def _setup_routes(self):
        self.app.router.add_get('/', self.index)
        self.app.router.add_post('/offer', self.offer)
        self.app.on_shutdown.append(self.on_shutdown)

    def _setup_cors(self):
        cors = aiohttp_cors.setup(self.app, defaults={
            "*": aiohttp_cors.ResourceOptions(
                allow_credentials=True,
                expose_headers="*",
                allow_headers="*",
                allow_methods="*"
            )
        })
        for route in list(self.app.router.routes()):
            cors.add(route)

    async def _start_web_server(self):
        self.get_logger().debug(f"Starting web server on {self.get_parameter('host').value}:{self.get_parameter('port').value}")
        runner = web.AppRunner(self.app)
        await runner.setup()
        site = web.TCPSite(runner, 
                          host=self.get_parameter('host').value,
                          port=self.get_parameter('port').value)
        await site.start()
        self.get_logger().info("WebRTC server started")

    async def index(self, request):
        return web.Response(text="WebRTC ROS2 Node", content_type="text/html")

    async def offer(self, request):
        self.get_logger().debug("Received WebRTC offer request")
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        pc = RTCPeerConnection()
        self.pcs.add(pc)
        
        @pc.on("connectionstatechange")
        async def _on_connectionstatechange():
            self.get_logger().debug(f"Connection state changed to {pc.connectionState}")
            if pc.connectionState == "connected":
                self._handle_new_connection()
            elif pc.connectionState in ["closed", "failed", "disconnected"]:
                if pc in self.pcs:  # Prevent multiple handling
                    await pc.close()
                    self.pcs.discard(pc)
                    self._handle_connection_end()

        @pc.on("track")
        def _on_track(track):
            self.get_logger().debug(f"Received track of type {track.kind}")
            if track.kind == "video":
                self._process_video_track(track, pc, params)

        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        
        self.get_logger().debug(f"Created peer connection {id(pc)}")
        
        return web.Response(
            content_type="application/json",
            text=json.dumps({
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type
            }),
        )

    def _handle_new_connection(self):
        self.get_logger().info(f"New connection, total: {self.active_connections + 1}")
        self.active_connections += 1
        if self.active_connections == 1:
            self.publisher = self.create_publisher(CompressedImage, 'camera/image/compressed', 10)
            self.get_logger().info("Started compressed video publisher")

    def _handle_connection_end(self):
        self.active_connections = max(0, self.active_connections - 1)
        self.get_logger().info(f"Connections remaining: {self.active_connections}")
        if self.active_connections == 0 and self.publisher is not None:
            self.destroy_publisher(self.publisher)
            self.publisher = None
            self.get_logger().info("Stopped video publisher")

    def _process_video_track(self, track, pc, params):
        self.get_logger().debug("Starting video track processing")
        video_track = self.relay.subscribe(track)
        
        @track.on("ended")
        async def _on_ended():
            self.get_logger().info("Video track ended")
            await pc.close()

        async def _process_frames():
            self.get_logger().debug("Starting frame processing loop")
            try:
                while track.readyState == "live":
                    try:
                        frame = await video_track.recv()
                        self.get_logger().debug("Received video frame", throttle_duration_sec=1)
                        img = frame.to_ndarray(format="bgr24")
                        if self.publisher is not None:
                            self._publish_image(img)
                    except Exception as e:
                        self.get_logger().error(f"Frame processing error: {str(e)}")
                        break
            finally:
                # Ensure connection cleanup even if loop breaks
                if pc.connectionState not in ["closed", "disconnected"]:
                    await pc.close()
                self._handle_connection_end()

        asyncio.create_task(_process_frames())

    def _publish_image(self, img):
        # Compress image to JPEG
        _, compressed_img = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = compressed_img.tobytes()
        self.publisher.publish(msg)

    async def on_shutdown(self, app):
        coros = [pc.close() for pc in self.pcs]
        await asyncio.gather(*coros)
        self.pcs.clear()

    def destroy_node(self):
        self.get_logger().info("Shutting down WebRTC node")
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join()
        self.get_logger().debug("Event loop thread joined")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebRTCNode()
    
    # Configure logging from parameter
    if node.get_parameter('verbose').value:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 