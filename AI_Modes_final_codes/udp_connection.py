import asyncio
import socket
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class UDPServerProtocol:
    def __init__(self, queue):
        self.queue = queue

    def connection_made(self, transport):
        self.transport = transport
        logger.info(f"UDP server listening on port 5006")

    def datagram_received(self, data, addr):
        try:
            decoded_text = data.decode('utf-8', errors='ignore')
            parsed_data = json.loads(decoded_text)
            asyncio.create_task(self.queue.put(parsed_data))  # Asynchronously put data in queue
            logger.info(f"Received from {addr}: {parsed_data}")
        except json.JSONDecodeError:
            logger.error(f"Failed to decode JSON from {addr}: {data}")

    def error_received(self, exc):
        logger.error(f"UDP error: {exc}")

async def fetch_data_from_udp(queue):
    """Receive UDP data asynchronously and put it into the queue."""
    loop = asyncio.get_running_loop()
    try:
        transport, protocol = await loop.create_datagram_endpoint(
            lambda: UDPServerProtocol(queue),
            local_addr=("0.0.0.0", 5006)  # Explicitly bind to all interfaces
        )
        logger.info(f"UDP server started on 0.0.0.0:5006")
        try:
            await asyncio.Future()  # Keep the UDP server running
        finally:
            transport.close()
            logger.info("UDP server closed")
    except socket.gaierror as e:
        logger.error(f"Failed to create UDP endpoint: {e}")
        raise
    except Exception as e:
        logger.error(f"Unexpected error in UDP server: {e}")
        raise        
