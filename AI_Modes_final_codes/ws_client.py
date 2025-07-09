# ws_client.py
import asyncio
import websockets
import json
import logging
import random
import socket
from udp_connection import UDPServerProtocol
from udp_connection import fetch_data_from_udp
from asyncio import Queue
from access_token import get_access_token

# Configure server
hostname = socket.gethostname()
local_ip = socket.gethostbyname(hostname)
port = 8081

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

async def connect_to_server():
    # WebSocket server details
    server_url = f"ws://{local_ip}:{port}"  # Replace with actual server IP and port
    data_queue = Queue()
    access_token = None

    # Delay connection by 5 seconds until it gets access token
    while not access_token:
        logger.info("No access_token yet, waiting 5 seconds before getting access token again...")
        await asyncio.sleep(5)
        access_token = get_access_token()

    logger.info("Initiating connection to server")

    # Start data producer
    asyncio.create_task(fetch_data_from_udp(data_queue))

    try:
        async with websockets.connect(server_url) as websocket:
            # Send authentication token
            auth_message = json.dumps({"accessToken": access_token})
            await websocket.send(auth_message)
            logger.info("Sent authentication token")

            # Wait for authentication response
            auth_response = await websocket.recv()
            auth_response = json.loads(auth_response)
            logger.info(f"Authentication response: {auth_response}")

            if auth_response.get("status") == "success":
                # Continuously process and send data from queue
                while True:
                    health_data = await data_queue.get()
                    await websocket.send(json.dumps(health_data))
                    logger.info(f"Sent health data: {health_data}")

                    # Wait for server acknowledgment
                    acknowledgment = await websocket.recv()
                    acknowledgment = json.loads(acknowledgment)
                    logger.info(f"Server acknowledgment: {acknowledgment}")
                    data_queue.task_done()
            else:
                logger.error("Authentication failed")

    except websockets.exceptions.ConnectionClosed:
        logger.error("Connection to server closed")
    except Exception as e:
        logger.error(f"An error occurred: {str(e)}")

# Run the WebSocket client
if __name__ == "__main__":
    asyncio.run(connect_to_server())
