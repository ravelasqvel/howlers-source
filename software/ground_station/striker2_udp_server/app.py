from flask import Flask, render_template, request
import socket

app = Flask(__name__)

# UDP Configuration
# Replace with your actual UDP server IP and port
UDP_IP = "10.0.0.107"  #192.168.20.7
UDP_PORT = 12345           

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/send', methods=['POST'])
def send():
    command = request.form['command']
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(command.encode(), (UDP_IP, UDP_PORT))
        return f"Command sent: {command}"
    except Exception as e:
        return f"Error sending command: {str(e)}"
if __name__ == '__main__':
    print("Starting Flask server...")
    app.run(host='0.0.0.0', port=5000, debug=True)
    print("Flask server started at http://localhost:5000/")