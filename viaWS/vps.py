import asyncio
import websockets

CLIENTS = set()
PI_WS = None  # WebSocket dari Pi

async def broadcast_to_clients(message):
    if not CLIENTS:
        return
    tasks = [c.send(message) for c in list(CLIENTS)]
    if tasks:
        await asyncio.gather(*tasks, return_exceptions=True)


async def handle_pi(websocket): #PI CONNECTED
    global PI_WS
    PI_WS = websocket
    print("Pi connected")
    try:
        async for message in websocket:
            # Teruskan apa adanya ke semua client
            await broadcast_to_clients(message)
    finally:
        print("Pi disconnected")
        PI_WS = None
 
async def handle_client(websocket, first_msg=None): #WEB CONNECTED
    CLIENTS.add(websocket)
    try:
        print("Client connected")
        # Proses pesan pertama jika sudah terlanjur terbaca
        if first_msg is not None:
            print(f"Received from client: {first_msg}")
            if PI_WS is not None:
                await PI_WS.send(first_msg)

        async for message in websocket:
            print(f"Received from client: {message}")
            # Teruskan ke Pi bila tersedia
            if PI_WS is not None:
                await PI_WS.send(message)
    finally:
        CLIENTS.remove(websocket)
        print("Client disconnected")





















async def handler(websocket):
    try:
        # Identifikasi awal: Pi akan kirim "pi", browser biasanya tidak kirim apa-apa
        first_msg = await asyncio.wait_for(websocket.recv(), timeout=1)
        if first_msg == "pi":
            await handle_pi(websocket)
        else:
            await handle_client(websocket, first_msg)
    except asyncio.TimeoutError:
        # Tidak ada pesan awal -> anggap client biasa
        await handle_client(websocket)
    except websockets.ConnectionClosed:
        pass

async def main():
    server = await websockets.serve(handler, "0.0.0.0", 8766)
    print("WebSocket server started on ws://0.0.0.0:8766")
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())