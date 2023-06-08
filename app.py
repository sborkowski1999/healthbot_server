from flask import Flask, render_template
from flask_socketio import SocketIO
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

app = Flask(__name__)
socketio = SocketIO(app)
map_image = 'map_image.jpg'
point_publisher = rospy.Publisher('goal_topic', Point, queue_size=10)

def map_callback(map_data):
    global map_image
    # make the map into an image HERE
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
    point_publisher.publish(goal_msg)

@app.route('/favicon.ico') # gets rid of get favicon.ico error
def favicon():
    return app.send_static_file('favicon.ico')


if __name__ == '__main__':
    rospy.init_node('websocket_node', anonymous=True)
    rospy.Subscriber('/map_topic', String, map_callback) #change map_topic and String
    app.debug = True
    socketio.run(app, host='0.0.0.0', port=5000)
