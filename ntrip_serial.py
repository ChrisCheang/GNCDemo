import base64
import serial
import socket
import sys
import time

# --- CONFIGURATION PARAMETERS ---
SERIAL_PORT = "COM7"
BAUD_RATE = 115200

NTRIP_CASTER = "www.geodetic.gov.hk"
NTRIP_PORT = 2101
MOUNTPOINT = "VRS32G"
USERNAME = "chrischeanghk"
PASSWORD = "hdbd6n"  # <--- INSERT YOUR EXACT SATREF PASSWORD HERE

# Default fallback GGA position (Hong Kong baseline) until Teensy returns a live fix
CURRENT_GGA = "$GNGGA,133733.70,2218.478535,N,11415.591232,E,1,14,2.19,60.1,M,-0.7,M,,*5E\r\n"


def create_ntrip_request(caster, port, mountpoint, username, password):
    """Builds the HTTP NTRIP v2 GET request with strict ASCII Basic Auth."""
    auth_str = f"{username}:{password}"
    # Use ASCII encoding to prevent auth header byte-string formatting bugs
    auth_bytes = base64.b64encode(auth_str.encode("ascii")).decode("ascii")

    request = (
        f"GET /{mountpoint} HTTP/1.1\r\n"
        f"Host: {caster}:{port}\r\n"
        f"User-Agent: NTRIP PythonClient/1.0\r\n"
        f"Authorization: Basic {auth_bytes}\r\n"
        f"Ntrip-Version: Ntrip/2.0\r\n"
        f"Connection: close\r\n\r\n"
    )
    return request.encode("ascii")


def main():
    global CURRENT_GGA

    # 1. Open Serial Connection to Teensy 4.1 on COM7
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"[SERIAL] Successfully opened {SERIAL_PORT} at {BAUD_RATE} baud.")
    except Exception as e:
        print(f"[ERROR] Failed to open serial port {SERIAL_PORT}: {e}")
        sys.exit(1)

    # 2. Connect to SatRef NTRIP Caster
    print(f"[NTRIP] Connecting to {NTRIP_CASTER}:{NTRIP_PORT}/{MOUNTPOINT}...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((NTRIP_CASTER, NTRIP_PORT))
        sock.settimeout(5.0)

        # Send GET Request
        ntrip_req = create_ntrip_request(
            NTRIP_CASTER, NTRIP_PORT, MOUNTPOINT, USERNAME, PASSWORD
        )
        sock.sendall(ntrip_req)

        # Read Caster Response Header
        response = sock.recv(1024).decode("utf-8", errors="ignore")
        print(
            f"[NTRIP] Server Response Header:\n{response.strip()}\n"
            + "-" * 50
        )

        if "200 OK" not in response and "ICY 200 OK" not in response:
            print(
                "[ERROR] Failed to authorize or connect to SatRef NTRIP server. Check your username/password."
            )
            sock.close()
            ser.close()
            sys.exit(1)

        print(
            "[NTRIP] Connected successfully! Streaming RTCM3 corrections to Teensy...\n"
        )

    except Exception as e:
        print(f"[ERROR] Socket error connecting to NTRIP Caster: {e}")
        ser.close()
        sys.exit(1)

    # 3. Main Dual-Direction Transfer Loop
    last_gga_sent_time = 0

    try:
        while True:
            # A. Read incoming RTCM3 bytes from Caster -> Write to Teensy USB Serial
            try:
                data = sock.recv(2048)
                if data:
                    ser.write(data)
                    print(
                        f"[NTRIP -> TEENSY] Forwarded {len(data)} RTCM3 bytes",
                        end="\r",
                    )
            except socket.timeout:
                pass

            # B. Read incoming lines from Teensy USB Serial -> Capture GNGGA & Print
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    if "$GNGGA" in line or "$GPGGA" in line:
                        CURRENT_GGA = line + "\r\n"
                        print(
                            f"\n[TEENSY -> PYTHON GNGGA] Captured live fix: {line}"
                        )

            # C. Periodically send NMEA GGA position to SatRef Caster (every 5 seconds)
            current_time = time.time()
            if current_time - last_gga_sent_time >= 5.0:
                sock.sendall(CURRENT_GGA.encode("ascii"))
                print(
                    f"[TEENSY -> NTRIP CASTER] Sent GGA: {CURRENT_GGA.strip()}"
                )
                last_gga_sent_time = current_time

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[INFO] Exiting script cleanly...")
    finally:
        sock.close()
        ser.close()
        print("[INFO] Serial port and socket closed.")


if __name__ == "__main__":
    main()
