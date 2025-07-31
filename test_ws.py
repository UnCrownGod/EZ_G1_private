# test_ws.py
import asyncio
import websockets

async def main():
    uri = "ws://127.0.0.1:8000/ws/devices/1/metrics?sample_frames=30"
    async with websockets.connect(uri) as ws:
        for _ in range(5):
            msg = await ws.recv()
            print(msg)
            await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main()) 
