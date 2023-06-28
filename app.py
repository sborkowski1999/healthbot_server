from flask import Flask, render_template
from flask_socketio import SocketIO
from flask_cors import CORS
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import cv2 # below are used for converting map to image
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app)
map_image = 'map_image.png'
goal_publisher = rospy.Publisher('goal_topic', Point, queue_size=10)

bridge = CvBridge()

goal_reached = 0;

def occupancy_grid_to_image(map_grid):
    # Convert the OccupancyGrid message to an image using OpenCV
    # You can modify this implementation based on your specific requirements
    grid_width = map_grid.info.width
    grid_height = map_grid.info.height
    grid_data = map_grid.data

    # Create an empty image with the same dimensions as the grid
    image = np.zeros((grid_height, grid_width), dtype=np.uint8)

    # Map the occupancy grid values to image pixel intensities
    for i in range(grid_height):
        for j in range(grid_width):
            index = i * grid_width + j
            if grid_data[index] == 100:  # Occupied cell
                image[i, j] = 255
            elif grid_data[index] == 0:  # Free cell
                image[i, j] = 0
            else:  # Unknown cell
                image[i, j] = 127

    # Convert the image to png format
    _, png_image = cv2.imencode('.png', image)

    # Convert the png image data to bytes
    png_bytes = png_image.tobytes()

    return png_bytes

def map_callback(map_grid):
    global map_image
    # make the map into an image HERdE
    map_image = occupancy_grid_to_image(map_grid)
    #emit map image to Websocket clients
    socketio.emit('map_update', map_image)

    if goal_reached==1:
        socketio.emit('prompt_patient_load')
        goal_reached = 0;

@socketio.on('patient_done_loading')
def handle_patient_done_loading():
    # Handle the patient's completion of loading laundry
    socketio.emit('robot_start_navigation')

@app.route('/') 
def index():
    return render_template('index.html') # check in the template folder

@socketio.on('connect')
def handle_connect():
    print('User connected!')

@socketio.on('marker_coordinates')
def handle_marker_coordinates(data):
    x = data.get('x')
    y = data.get('y')
    print("Coords: (", x, ",", y,")")
    
    # Create Point message here for goal
    goal_msg = Point()
    goal_msg.x = x # need to map points from image to goal
    goal_msg.y = y
    goal_msg.z = 0.0
    goal_publisher.publish(goal_msg)
    goal_reached=1

    if goal_reached==1:
        socketio.emit('prompt_patient_load')
        goal_reached = 0;

    


@app.route('/favicon.ico') # gets rid of get favicon.ico error
def favicon():
    return app.send_static_file('favicon.ico')

rospy.init_node('websocket_node', anonymous=True)
rospy.Subscriber('/map_topic', OccupancyGrid, map_callback)

if __name__ == '__main__':
    app.debug = True
    socketio.run(app, host='0.0.0.0', port=5000)
    rospy.spin()
