import asyncio
import json
import websockets

PI_WS = None
CLIENTS = {}

async def handler(ws):
    global PI_WS
    try:
        async for message in ws:
            try:
                data = json.loads(message)
            except Exception:
                continue
            
            mtype = data.get("type")

            if mtype == "register":
                role = data.get("role")
                if role == "pi":
                    PI_WS = ws
                    await ws.send(json.dumps({"type": "registered", "role": "pi"}))
                elif role == "client":
                    cid = data.get("client_id")
                    if not cid:
                        continue
                    CLIENTS[cid] = ws
                    await ws.send(json.dumps({"type": "registered", "role": "client"}))
                continue

            if mtype == "offer" and PI_WS is not None:
                await PI_WS.send(json.dumps(data))
                continue

            if mtype == "answer":
                cid = data.get("client_id")
                target = CLIENTS.get(cid)
                if target is not None:
                    await target.send(json.dumps(data))
                continue

            if mtype == "candidate":
                if ws is not PI_WS and PI_WS is not None:
                    await PI_WS.send(json.dumps({
                        "type": "candidate",
                        "client_id": data.get("client_id"),
                        "candidate": data.get("candidate"),
                    }))
                elif ws is PI_WS:
                    cid = data.get("client_id")
                    target = CLIENTS.get(cid)
                    if target is not None:
                        await target.send(json.dumps({
                            "type": "candidate",
                            "candidate": data.get("candidate"),
                        }))
                continue

            # Relay control: client -> PI
            if mtype == "control":
                if ws is not PI_WS and PI_WS is not None:
                    # teruskan pesan control apa adanya (PI akan memfilter key PINS)
                    await PI_WS.send(json.dumps(data))
                # jika PI mengirim control, abaikan
                continue
            if mtype == "ping":
                if ws is not PI_WS and PI_WS is not None:
                    # teruskan pesan control apa adanya (PI akan memfilter key PINS)
                    await PI_WS.send(json.dumps(data))
                # jika PI mengirim control, abaikan
                continue
            if mtype == "exposure":
                if ws is not PI_WS and PI_WS is not None:
                    # teruskan pesan control apa adanya (PI akan memfilter key PINS)
                    await PI_WS.send(json.dumps(data))
                # jika PI mengirim control, abaikan
                continue

        

    finally:
        if ws is PI_WS:
            PI_WS = None
        for k, v in list(CLIENTS.items()):
            if v is ws:
                CLIENTS.pop(k, None)

async def main():
    server = await websockets.serve(handler, "0.0.0.0", 8766)
    print("Signaling server started on ws://0.0.0.0:8766")
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())