#!/usr/bin/env python3
"""
Простой тестовый TCP-сервер, эмулирующий ESP32: отправляет строки телеметрии с заданной частотой
и печатает полученные команды от клиента.

Usage:
    python3 tcp_telemetry_server.py --host 0.0.0.0 --port 3333 --hz 50
"""
import socket
import threading
import time
import argparse


def handle_client(conn, addr, hz):
    print(f"Client connected: {addr}")
    conn.settimeout(1.0)
    count = 0
    try:
        while True:
            # Отправляем строку телеметрии
            line = f"L={count},R={count*2}\n"
            try:
                conn.sendall(line.encode('utf-8'))
            except BrokenPipeError:
                print('Client disconnected')
                break

            # Попытаться прочитать входящие команды (non-blocking-ish)
            try:
                data = conn.recv(1024)
                if data:
                    print(f"<- Received from client: {data.decode().strip()}")
            except socket.timeout:
                pass
            except Exception as e:
                print('Recv error:', e)
                break

            count += 1
            time.sleep(1.0 / hz)
    finally:
        try:
            conn.close()
        except:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=3333)
    parser.add_argument('--hz', type=float, default=50.0, help='Telemetry lines per second')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.host, args.port))
    sock.listen(1)
    print(f"Telemetry server listening on {args.host}:{args.port} (hz={args.hz})")
    try:
        while True:
            conn, addr = sock.accept()
            t = threading.Thread(target=handle_client, args=(conn, addr, args.hz), daemon=True)
            t.start()
    except KeyboardInterrupt:
        print('Shutting down')
    finally:
        sock.close()

if __name__ == '__main__':
    main()
