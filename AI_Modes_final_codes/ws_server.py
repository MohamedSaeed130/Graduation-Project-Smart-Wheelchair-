import os
import sys

# Add project_scripts to sys.path
project_scripts_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'project_scripts'))
if project_scripts_path not in sys.path:
    sys.path.append(project_scripts_path)

import asyncio
import websockets
import socket
import subprocess
import json
import logging
import transmitter
from udp_connection import UDPServerProtocol
from udp_connection import fetch_data_from_udp
from access_token import set_access_token

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Get local IP address
hostname = socket.gethostname()
local_ip = socket.gethostbyname(hostname)
port = 8082

client_processes = {}  # Dictionary to store processes per client
client_data_tasks = {}  # Dictionary to store data streaming tasks per client

async def send_real_time_data(websocket, queue):
    """Send real-time health data to the client."""
    try:
        while True:
            data = await queue.get()
            await websocket.send(json.dumps(data))
            logger.info(f"Sent data to {websocket.remote_address[0]}: {data}")
            queue.task_done()
    except (websockets.exceptions.ConnectionClosed, asyncio.CancelledError):
        logger.info(f"Stopped sending real-time data to {websocket.remote_address[0]}")
        raise

async def wss(websocket):
    client_ip = websocket.remote_address[0]
    logger.info(f"Client {client_ip} connected")
    client_processes[websocket] = None  # Initialize process for this client
    client_data_tasks[websocket] = None  # Initialize data task for this client
    data_queue = asyncio.Queue()  # Create a queue for this client's data

    try:
        async for message in websocket:
            logger.info(f"Received from {client_ip}: {message}")

            if message == "reset":
                transmitter.send_command("STOP|REMOTE")  # Stop movement
                if client_processes.get(websocket):
                    client_processes[websocket].terminate()
                    client_processes.pop(websocket)
                logger.info("Mode selection reset")
            elif message == "send_real_time":
                if not client_data_tasks[websocket]:
                    # Start the data producer if not already running for this client
                    asyncio.create_task(fetch_data_from_udp(data_queue))
                    # Start sending data to client
                    client_data_tasks[websocket] = asyncio.create_task(
                        send_real_time_data(websocket, data_queue)
                    )
                    logger.info(f"Started sending real-time data to {client_ip}")
                else:
                    logger.info(f"Real-time data already streaming to {client_ip}")
            elif message == "stop_real_time":
                if client_data_tasks[websocket]:
                    client_data_tasks[websocket].cancel()
                    client_data_tasks[websocket] = None
                    logger.info(f"Stopped sending real-time data to {client_ip}")
                else:
                    logger.info(f"No real-time data streaming to stop for {client_ip}")
            else:
                parsed_message = message.split(":")

                if parsed_message[0] == "select_mode" and not client_processes.get(websocket):  # Mode selection
                    mode = parsed_message[1]
                    if mode in ["face", "eye", "voice", "hand"]:
                        script = f"./project_scripts/{mode}.py"
                        process = subprocess.Popen(["python", script])
                        logger.info(f"Executing {mode} mode")
                        client_processes[websocket] = process
                    else:
                        logger.warning(f"Unknown mode: {mode}")
                elif parsed_message[0] == "remote":  # Remote control
                    movement_command = parsed_message[1].upper()
                    transmitter.send_command(f"{movement_command}|REMOTE")
                    logger.info(f"Sent {movement_command}|REMOTE to ESP32")
                elif parsed_message[0] == "access_token":  # Access token
                    set_access_token(parsed_message[1])
                else:
                    logger.warning(f"Unknown command: {message}")
    except websockets.exceptions.ConnectionClosed:
        logger.info(f"Client {client_ip} disconnected")
    finally:
        # Cleanup on client disconnection
        if client_processes.get(websocket):
            client_processes[websocket].terminate()
            client_processes.pop(websocket)
        if client_data_tasks.get(websocket):
            client_data_tasks[websocket].cancel()
            client_data_tasks.pop(websocket)

async def main():
    logger.info(f"WebSocket server is running on ws://{local_ip}:{port}")
    async with websockets.serve(wss, local_ip, port):
        await asyncio.Future()  # Keeps the server running

if __name__ == "__main__":
    asyncio.run(main())