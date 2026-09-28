

import argparse
import json
import socket
import struct
import time
import threading


class WaypointRailClient:
    def __init__(self, tcp_host="192.168.7.6", tcp_port=5002, udp_port=5003, counts_per_mm=26214.4):
        self.tcp_host = tcp_host
        self.tcp_port = tcp_port
        self.udp_port = udp_port
        self.counts_per_mm = counts_per_mm
        
        self.tcp_sock = None
        self.udp_sock = None
        self.lock = threading.Lock()
        
        # Latest telemetry state from UDP
        self.current_counts = 0
        self.current_vel = 0.0
        self.running = False
        self.listener_thread = None

    def connect(self):
        # Setup TCP command socket
        self.tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_sock.connect((self.tcp_host, self.tcp_port))
        
        # Setup UDP feedback listener socket (~100Hz stream)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_sock.bind(("0.0.0.0", self.udp_port))
        
        # Send subscribe command bytes as noted in your protocol doc
        subscribe_packet = b"\x00\x00\x00\x01\x00\x00\x00\x0a"
        self.udp_sock.sendto(subscribe_packet, (self.tcp_host, self.udp_port))
        
        # Start background feedback listener
        self.running = True
        self.listener_thread = threading.Thread(target=self._udp_listen_loop, daemon=True)
        self.listener_thread.start()

    def close(self):
        self.running = False
        if self.udp_sock:
            # Unsubscribe bytes
            try:
                self.udp_sock.sendto(b"\x00\x00\x00\x00\x00\x00\x00\x00", (self.tcp_host, self.udp_port))
            except Exception:
                pass
            self.udp_sock.close()
        if self.tcp_sock:
            self.tcp_sock.close()

    def _udp_listen_loop(self):
        while self.running:
            try:
                self.udp_sock.settimeout(1.0)
                data, _ = self.udp_sock.recvfrom(320)  # Expected 80 32-bit words (320 bytes)
                if len(data) >= 320:
                    # Word 0: Actual position (int32) at offset 0
                    pos_counts = struct.unpack_from(">i", data, 0)[0]
                    # Word 8: Current jog/axis velocity (float32) at offset 32
                    vel = struct.unpack_from(">f", data, 32)[0]
                    
                    with self.lock:
                        self.current_counts = pos_counts
                        self.current_vel = vel
            except socket.timeout:
                continue
            except Exception:
                if not self.running:
                    break

    def send_tcp_command(self, cmd: str):
        with self.lock:
            full_cmd = f"{cmd}\r".encode("ascii")
            self.tcp_sock.sendall(full_cmd)
            # Read reply/prompt if needed (Acroloop returns SYS>)
            time.sleep(0.05) 
        # Non-blocking or short-timeout read to capture controller response/echo/prompt
        self.tcp_sock.settimeout(0.2)
        response_bytes = b""
        try:
            while True:
                chunk = self.tcp_sock.recv(1024)
                if not chunk:
                    break
                response_bytes += chunk
                if b"SYS>" in response_bytes or len(chunk) < 1024:
                    break
        except socket.timeout:
            pass
        
        response_str = response_bytes.decode("ascii", errors="ignore")
        print(f"TX: {cmd} | RX: {response_str.strip()}")
        return response_str

    def move_abs(self, position_mm: float, speed_mm_s: float = None, tolerance_mm: float = 0.2, timeout: float = 10.0):
        """
        Moves the rail to an absolute position (mm) using native controller commands.
        Optionally sets axis speed if supported by your firmware parameters, 
        then blocks until position is reached within tolerance.
        """
        target_counts = position_mm * self.counts_per_mm
        
        # Optional: Set speed parameter if desired before moving, e.g., AXIS0 MOVE_SPEED
        if speed_mm_s is not None:
            # Note: Verify parameter/command format for speed in your Acroloop manual if needed,
            # or rely on pre-configured acceleration/speed profiles on the controller.
            pass

        print(f"Moving to {position_mm} mm ({target_counts} counts)...")
        
        # Send native absolute move command
        # self.send_tcp_command(f"AXIS0 JOG {position_mm:.3f}")
        self.send_tcp_command(f"AXIS0 JOG ABS {position_mm:.3f}")

        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.lock:
                current_mm = self.current_counts / self.counts_per_mm
                current_v = self.current_vel

            # Check if we are within position tolerance and motion has settled (velocity ~ 0)
            if abs(current_mm - position_mm) <= tolerance_mm and abs(current_v) < 0.05:
                print(f"Reached waypoint {position_mm} mm successfully.")
                return True

            time.sleep(0.05)

        raise TimeoutError(f"Motion to {position_mm} mm timed out.")

    def follow_trajectory(self, waypoints_mm, tolerance_mm=0.2):
        """
        Iterates through a list of absolute position waypoints.
        """
        try:
            for wp in waypoints_mm:
                self.move_abs(wp, tolerance_mm=tolerance_mm)
                # Move to the next waypoint immediately after settling.
        except KeyboardInterrupt:
            print("Trajectory interrupted by user. Stopping axis...")
            self.send_tcp_command("AXIS0 JOG OFF")


def load_rail_waypoints(plan_file, num_waypoints=100):
    with open(plan_file, "r") as f:
        plan = json.load(f)

    q_path = plan["q_path"]

    rail_waypoints_mm = []

    for configuration in q_path[:num_waypoints]:
        rail_m = configuration[0]
        rail_mm = rail_m * 1000.0
        rail_waypoints_mm.append(rail_mm)

    return rail_waypoints_mm


# --- Example Usage ---
if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("plan_file", help="Path to best_plan.json")
    args = parser.parse_args()

    # Load first 100 rail waypoints from q_path
    waypoint_list = load_rail_waypoints(
        args.plan_file,
        num_waypoints=100
    )

    print("Rail waypoints loaded from plan:")
    for i, waypoint in enumerate(waypoint_list):
        print(f"  {i + 1}: {waypoint:.3f} mm")

    client = WaypointRailClient()

    try:
        client.connect()
        print("Connected to rail. Starting waypoint sequence...")

        client.follow_trajectory(waypoint_list)

    finally:
        client.close()
        print("Connection closed.")
