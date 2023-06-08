from flask import Flask, render_template
from flask_socketio import SocketIO
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import cv2 # below are used for converting map to image
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

app = Flask(__name__)
socketio = SocketIO(app)
map_image = 'map_image.jpg'
goal_publisher = rospy.Publisher('goal_topic', Point, queue_size=10)

bridge = CvBridge()

def occupancy_grid_to_image(map_data):
    # Convert the OccupancyGrid message to an image using OpenCV
    # You can modify this implementation based on your specific requirements
    grid_width = map_data.info.width
    grid_height = map_data.info.height
    grid_data = map_data.data

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

    # Convert the image to JPEG format
    _, jpeg_image = cv2.imencode('.jpg', image)

    # Convert the JPEG image data to bytes
    jpeg_bytes = jpeg_image.tobytes()

    return jpeg_bytes

def map_callback(map_data):
    global map_image
    # make the map into an image HERE
    map_image = occupancy_grid_to_image(map_data)
    #emit map image to Websocket clients
    socketio.emit('map_update', map_image)

@app.route('/') 
def index():
    return render_template('index.html') # check in the template folder

@socketio.on('connect')
def handle_connect():
    print('User connected!')
    map_callback(None)


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


@app.route('/favicon.ico') # gets rid of get favicon.ico error
def favicon():
    return app.send_static_file('favicon.ico')

rospy.init_node('websocket_node', anonymous=True)
rospy.Subscriber('/map_topic', OccupancyGrid, map_callback)

if __name__ == '__main__':
    app.debug = True
    socketio.run(app, host='0.0.0.0', port=5000)
    rospy.spin()
