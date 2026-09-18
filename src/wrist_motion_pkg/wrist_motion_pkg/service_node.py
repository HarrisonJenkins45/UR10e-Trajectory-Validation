#Send velocity commands
# import rclpy
# from rclpy.node import Node
# from std_srvs.srv import Trigger
# import socket
# import textwrap

# class WristVelocityService(Node):
#     def __init__(self):
#         super().__init__('wrist_velocity_service')
#         self.srv = self.create_service(
#             Trigger, 
#             'trigger_wrist_motion', 
#             self.handle_trigger_motion
#         )
#         self.robot_ip = "192.168.7.8"
#         self.port = 30002
#         self.get_logger().info("Wrist Velocity Service is ready.")

#     def handle_trigger_motion(self, request, response):
#         # textwrap.dedent strips leading indentation from the multiline string
#         script_str = textwrap.dedent("""
#             def wrist_velocity_test():
#                 target_speed = [0.0, 0.0, 0.0, 0.0, 0.3, 0.3]
#                 speedj(target_speed, a=1.5, t=3.0)
#                 stopj(1.5)
#             end
#             wrist_velocity_test()
#         """).strip()
        
#         urscript_code = script_str.encode('utf-8')

#         try:
#             sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             sock.settimeout(3.0)
#             sock.connect((self.robot_ip, self.port))
#             sock.sendall(urscript_code)
#             sock.close()
            
#             response.success = True
#             response.message = "Successfully streamed wrist velocity command!"
#             self.get_logger().info(response.message)
#         except Exception as e:
#             response.success = False
#             response.message = f"Failed to connect: {e}"
#             self.get_logger().error(response.message)
            
#         return response

# def main(args=None):
#     rclpy.init(args=args)
#     node = WristVelocityService()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



#Send joint angle commands
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import socket
import textwrap

class WristVelocityService(Node):
    def __init__(self):
        super().__init__('wrist_velocity_service')
        self.srv = self.create_service(
            Trigger, 
            'trigger_wrist_motion', 
            self.handle_trigger_motion
        )
        self.robot_ip = "192.168.7.8"
        self.port = 30002
        self.get_logger().info("Relative Waypoint Service is ready.")
        # q_init = get_actual_joint_positions()
        # self.get_logger().info(f"Starting waypoint is {q_init}")



    def handle_trigger_motion(self, request, response):
        # URScript natively queries the current configuration, 
        # avoiding complex binary parsing over port 30003 in Python.
        script_str = textwrap.dedent("""
            def relative_waypoint_path():
                # Query current joint positions [base, shoulder, elbow, w1, w2, w3]
                q_init = get_actual_joint_positions()
                
                # Create a target by adding a 0.2 rad offset specifically to Wrist 3 (index 5)
                q_target = [q_init[0], q_init[1], q_init[2], q_init[3], q_init[4]+0.2, q_init[5] + 0.2]
                
                # Move smoothly to the relative target
                movej(q_target, a=1.2, v=0.8)
            end
            relative_waypoint_path()
        """).strip()
        
        urscript_code = script_str.encode('utf-8')

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((self.robot_ip, self.port))
            sock.sendall(urscript_code)
            sock.close()
            
            response.success = True
            response.message = "Successfully executed relative waypoint movement!"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to connect: {e}"
            self.get_logger().error(response.message)
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = WristVelocityService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()