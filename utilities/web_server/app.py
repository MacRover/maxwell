from flask import Flask, render_template, jsonify
import threading, serial, time

app = Flask(__name__)


latest = {'data': []}  

def reader():
    ser = serial.Serial('/dev/cu.usbmodem169898601', 115200, timeout=1)
    while True:
        line = ser.readline().decode(errors='ignore').rstrip('\r\n')
        if line:
           
            latest['data'].append(line)
            if len(latest['data']) > 10:
               
                latest['data'].pop(0)

@app.route('/')
def index():
    
    joined = "\n".join(latest['data'])
    return render_template('index.html', data=joined)

@app.route('/data')
def data():
    
    return jsonify(latest)

if __name__ == '__main__':
    t = threading.Thread(target=reader, daemon=True)
    t.start()
    app.run(host='0.0.0.0', port=5000)
