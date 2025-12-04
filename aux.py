import asyncio
import struct

HOST = "127.0.0.1"     # same as IMAGE_SERVER_HOST
PORT = 9000            # same as IMAGE_SERVER_PORT

async def send_images():
    reader, writer = await asyncio.open_connection(HOST, PORT)
    print("Connected to server")

    try:
        while True:
            # Minimal fake "image" data
            img_bytes = b"0123456789"  # 10 bytes

            # 4-byte header: image size in big-endian
            header = struct.pack(">I", len(img_bytes))

            # Send
            writer.write(header + img_bytes)
            await writer.drain()

            print("Sent fake image:", img_bytes)

            await asyncio.sleep(1)

    except KeyboardInterrupt:
        print("Stopped by user")

    finally:
        writer.close()
        await writer.wait_closed()

asyncio.run(send_images())