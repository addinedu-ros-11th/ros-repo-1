import argparse
import socket
import struct
import sys
import time


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Minimal UDP receiver for robot video stream (prints header fields)."
    )
    parser.add_argument("--bind", default="0.0.0.0", help="Bind IP (default: 0.0.0.0)")
    parser.add_argument(
        "--port", type=int, default=54321, help="UDP port (default: 54321)"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind, args.port))

    count = 0
    last_print = time.time()
    print(f"Listening on {args.bind}:{args.port} (UDP)")

    while True:
        data, _addr = sock.recvfrom(65535)
        if len(data) < 16:
            continue
        frame_id, ts, idx, total = struct.unpack(">IQHH", data[:16])
        count += 1
        now = time.time()
        if count % 100 == 0 or (now - last_print) > 5.0:
            print(
                f"packets={count} frame_id={frame_id} idx/total={idx}/{total} ts={ts}"
            )
            last_print = now


if __name__ == "__main__":
    sys.exit(main())
