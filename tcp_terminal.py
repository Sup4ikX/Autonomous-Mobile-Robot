#!/usr/bin/env python3
import socket
import threading
import sys
import os

HOST = "192.168.0.23"
PORT = 3333

def receive_thread(sock):
    """Поток для приёма данных от ESP32 и вывода на экран."""
    while True:
        try:
            data = sock.recv(1024)
            if not data:
                print("\nСоединение закрыто ESP32")
                os._exit(0)
            # Выводим полученные данные (может быть несколько строк)
            print(data.decode('utf-8', errors='ignore').strip())
        except (ConnectionResetError, BrokenPipeError):
            print("\nСоединение потеряно")
            os._exit(0)
        except Exception as e:
            print(f"\nОшибка приёма: {e}")
            os._exit(0)

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((HOST, PORT))
        print(f"Подключено к {HOST}:{PORT}. Вводите команды (Ctrl+C для выхода):")
    except Exception as e:
        print(f"Не удалось подключиться: {e}")
        sys.exit(1)

    # Запускаем поток для чтения из сокета
    recv_thread = threading.Thread(target=receive_thread, args=(sock,), daemon=True)
    recv_thread.start()

    try:
        while True:
            # Читаем строку из stdin
            cmd = sys.stdin.readline()
            if not cmd:  # EOF (Ctrl+D)
                break
            cmd = cmd.strip()
            if cmd:
                sock.sendall((cmd + '\n').encode('utf-8'))
    except KeyboardInterrupt:
        print("\nЗавершение по Ctrl+C")
    finally:
        sock.close()
        print("Соединение закрыто")

if __name__ == "__main__":
    main()