import asyncio
import websockets
import json
import rospy
from geometry_msgs.msg import Twist
import signal

class RobotController:
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('websocket_server_node', anonymous=True)
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

        # Dictionary to mimic switch-case for commands
        self.commands = {
            'forward': self.move_north,
            'backward': self.move_south,
            'west': self.move_west,
            'east': self.move_east,
            'northwest': self.move_northwest,
            'northeast': self.move_northeast,
            'southwest': self.move_southwest,
            'southeast': self.move_southeast
        }

    def move_north(self):
        self.twist.linear.x += 1.0

    def move_south(self):
        self.twist.linear.x -= 1.0

    def move_west(self):
        self.twist.linear.y += 1.0

    def move_east(self):
        self.twist.linear.y -= 1.0

    def move_northwest(self):
        self.twist.linear.x += 1.0
        self.twist.linear.y += 1.0

    def move_northeast(self):
        self.twist.linear.x += 1.0
        self.twist.linear.y -= 1.0

    def move_southwest(self):
        self.twist.linear.x -= 1.0
        self.twist.linear.y += 1.0

    def move_southeast(self):
        self.twist.linear.x -= 1.0
        self.twist.linear.y -= 1.0

    def process_command(self, command):
        rospy.loginfo(f"Received command: {command}")
        if command in self.commands:
            self.commands[command]()
        else:
            rospy.logwarn(f"Unknown command: {command}")

        self.pub.publish(self.twist)
        return self.twist

async def handle_connection(websocket, path, controller):
    async for message in websocket:
        try:
            command = message.strip().lower()
            twist = controller.process_command(command)
            response = {
                "linear": {
                    "x": twist.linear.xc,
                    "y": twist.linear.y,
                    "z": twist.linear.z
                },
                "angular": {
                    "x": twist.angular.x,
                    "y": twist.angular.y,
                    "z": twist.angular.z
                }
            }
            await websocket.send(json.dumps(response))
        except json.JSONDecodeError as e:
            rospy.logwarn(f"Failed to decode JSON message: {e}")

async def main():
    controller = RobotController()
    server = await websockets.serve(lambda ws, path: handle_connection(ws, path, controller), "localhost", 8765)
    
    loop = asyncio.get_running_loop()
    stop = asyncio.Future()

    def signal_handler():
        stop.set_result(None)

    loop.add_signal_handler(signal.SIGINT, signal_handler)
    loop.add_signal_handler(signal.SIGTERM, signal_handler)

    await stop
    server.close()
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())
