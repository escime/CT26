import threading
import socket
import time
from typing import List

# RobotPy HAL components
import hal


class AMDiagnosticsServer:
    DEFAULT_PORT = 12345

    # Shared state
    _burst_status = -1  # -1: waiting, 0: fail, 1: success
    _last_timestamp = 0
    _confirmation_monitor = threading.Condition()
    _stream_handle = None
    _server_started = False
    _lock = threading.Lock()

    @classmethod
    def ensure_server_running(cls, port: int = DEFAULT_PORT):
        with cls._lock:
            if not cls._server_started:
                cls._server_started = True
                t = threading.Thread(target=cls._server_thread_main, args=(port,), name="AMDiag-ServerThread")
                t.daemon = True
                t.start()

    @classmethod
    def _open_can_stream_all(cls) -> bool:
        # Filter is AndyMark manufacturer code (0x0F), mask is manufacturer bits
        filter_id = 0x0F << 16
        mask = 0xFF << 16
        max_msgs = 256

        try:
            # hal.openCANStreamSession returns a handle
            cls._stream_handle = hal.openCANStreamSession(filter_id, mask, max_msgs)
            return cls._stream_handle != 0
        except Exception as e:
            print(f"[AMDiag] openCANStreamSession failed: {e}")
            return False

    @classmethod
    def _close_can_stream(cls):
        if cls._stream_handle is not None:
            try:
                hal.closeCANStreamSession(cls._stream_handle)
            except Exception as e:
                print(f"[AMDiag] closeCANStreamSession failed: {e}")
            cls._stream_handle = None

    @classmethod
    def _listen_for_can_messages(cls, conn: socket.socket, running_event: threading.Event):
        if not cls._open_can_stream_all():
            print("[AMDiag] Open CAN stream failed")
            return

        try:
            while running_event.is_set():
                try:
                    # hal.readCANStreamSession returns a list of CANStreamMessage objects
                    msgs = hal.readCANStreamSession(cls._stream_handle, 128)
                except Exception:
                    time.sleep(0.003)
                    continue

                if not msgs:
                    time.sleep(0.004)
                    continue

                for m in msgs:
                    # API code is bits 6-15
                    api_code = (m.messageID & 0x0000FFC0) >> 6

                    if api_code != 0x05:
                        # Format: "CAN_MSG <id> d0 d1 ... d7\n"
                        data_str = " ".join(str(b) for b in m.data)
                        # Padding if length < 8
                        if len(m.data) < 8:
                            data_str += " " + " ".join("0" for _ in range(8 - len(m.data)))

                        out_msg = f"CAN_MSG {m.messageID} {data_str}\n"
                        conn.sendall(out_msg.encode('ascii'))
                    else:
                        # ESP32 confirmation (API 0x05)
                        if len(m.data) >= 2:
                            success_flag = m.data[0]
                            timestamp = m.data[1]

                            with cls._confirmation_monitor:
                                cls._last_timestamp = timestamp
                                cls._burst_status = success_flag
                                cls._confirmation_monitor.notify_all()

                time.sleep(0.002)
        except (socket.error, Exception) as e:
            print(f"[AMDiag] Socket or Stream error: {e}")
        finally:
            cls._close_can_stream()

    @classmethod
    def _handle_connection(cls, client_socket: socket.socket):
        print("[AMDiag] Connection established")
        running_event = threading.Event()
        running_event.set()

        # Start CAN listener thread
        can_thread = threading.Thread(
            target=cls._listen_for_can_messages,
            args=(client_socket, running_event),
            name="AMDiag-CANListener"
        )
        can_thread.daemon = True
        can_thread.start()

        # Connection-specific state
        program_data = bytearray()
        expected_len = 0
        program_mode = False

        try:
            while True:
                data = client_socket.recv(32768)
                if not data:
                    break

                lines = data.decode('ascii').splitlines()
                for line in lines:
                    tokens = line.strip().split()
                    if not tokens:
                        continue

                    header = tokens[0]

                    if header == "FILE_START":
                        # logic for file handling
                        pass
                    # Add remaining protocol logic (SEND_CAN, etc.) here

        except Exception as e:
            print(f"[AMDiag] Connection handler error: {e}")
        finally:
            running_event.clear()
            client_socket.close()
            print("[AMDiag] Connection closed")

    @classmethod
    def _server_thread_main(cls, port: int):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            server_sock.bind(('', port))
            server_sock.listen(1)
            print(f"[AMDiag] Server listening on port {port}")

            while True:
                client, addr = server_sock.accept()
                cls._handle_connection(client)
        except Exception as e:
            print(f"[AMDiag] Server main thread error: {e}")
        finally:
            server_sock.close()
