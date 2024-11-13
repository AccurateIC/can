from flask import Flask, request, jsonify
from flask_cors import CORS
import rospy
from std_msgs.msg import Int64
import threading
import time

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})

rospy.init_node('flask_ros_bridge', anonymous=True)

prop_cmd_pub = rospy.Publisher('/prop_cmd', Int64, queue_size=10)
rud_cmd_pub = rospy.Publisher('/rud_cmd', Int64, queue_size=10)

last_prop_value = 0
lock = threading.Lock()

def publish_prop_cmd():
    global last_prop_value
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        with lock:
            prop_cmd_pub.publish(last_prop_value)
        rate.sleep()

threading.Thread(target=publish_prop_cmd, daemon=True).start()

@app.route('/button_pressed', methods=['POST'])
def button_pressed():
    global last_prop_value
    data = request.json
    button_name = data.get('buttonName')
    value = data.get('value')
    
    valid_buttons = ['Right', 'Left', 'Forward', 'Backward']
    if button_name not in valid_buttons or not isinstance(value, int):
        return jsonify({"error": "Invalid button name or value"}), 400

    with lock:
        if button_name in ['Forward', 'Right', 'Left']:
            last_prop_value = value
        elif button_name == 'Backward':
            last_prop_value = -value

    if button_name in ['Right', 'Left']:
        rud_value = 1 if button_name == 'Right' else -1
        rud_cmd_pub.publish(rud_value)
    
    return jsonify({"message": f"Button {button_name} pressed with value {value}"}), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
