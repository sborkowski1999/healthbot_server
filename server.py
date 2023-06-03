from flask import Flask, render_template
from flask_socketio import SocketIO
import rospy
from std_msgs.msg import Strings

app = Flask(__name__)

socketio = SocketIO(app)

@app.route('/') 
def index():
    return render_template('index.html') # check in the template folder

@socketio.on('connect')
def handle_connect(): # this function is for subscribing to map, and sending data to clients
    def map_callback(data):
        #process map data and initialize map_image variable

        #emit map image to Websocket clients
        socketio.emit('map_update', map_image)
    rospy.init_node('websocket_node')
    rospy.Subscriber('/map_topic', String, map_callback) #change map_topic and String

if __name__ == '__main__':
    socketio.run(app)
