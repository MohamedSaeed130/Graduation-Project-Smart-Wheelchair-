import asyncio
import logging
from ws_client import connect_to_server
from ws_server import main as start_ws_server



# Configure logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)



async def run_both_scripts():
  """Run both scripts concurrently."""
  try:

    # Run both scripts concurrently using gather

    await asyncio.gather(

    connect_to_server(), # From websocket_health_client.py

    start_ws_server(), # From other_script.py (adjust name as needed)

    return_exceptions=True

    )

  except Exception as e:
    logger.error(f"Error running scripts: {str(e)}")



if __name__ == "__main__":
  logger.info("Starting both ws_client and ws_server scripts")
  asyncio.run(run_both_scripts())
