
import subprocess, cv2, glob, os, time, threading, json
from ultralytics import YOLO
from flask import Flask, Response, jsonify
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

MODEL_PATH = "/home/design2/best.pt"
CONF_THRESHOLD = 0.45

# Desired object per waypoint
WAYPOINT_TARGETS = {
    'B': 'plastic_bottle',
    'C': 'clock',
    'D': 'perfume'
}

# Camera angles
CAM_ANGLES = [0, 30, 45, 60]

env = os.environ.copy()
env['LIBCAMERA_IPA_MODULE_PATH'] = '/usr/local/lib/aarch64-linux-gnu/libcamera/ipa'
env['LD_LIBRARY_PATH'] = '/usr/local/lib/aarch64-linux-gnu'
env['HOME'] = '/home/design2'

CLASS_COLORS = {
    0: (255, 100,  50),
    1: ( 50, 230,  50),
    2: ( 50, 150, 255),
    3: (  0, 255, 255),
}

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10)

app = Flask(__name__)
raw_frame = None
latest_detections = []
detection_active = False
last_trigger_angle = None
raw_lock = threading.Lock()
detection_lock = threading.Lock()

print("Loading YOLO model...")
model = YOLO(MODEL_PATH)
print(f"Model loaded | Classes: {model.names}")

# ROS2 node reference
ros_node = None

def is_file_complete(filepath):
    try:
        s1 = os.path.getsize(filepath)
        time.sleep(0.01)
        s2 = os.path.getsize(filepath)
        return s1 == s2 and s1 > 0
    except:
        return False

def camera_loop():
    global raw_frame
    for f in glob.glob('/tmp/vis*'): os.remove(f)
    proc = subprocess.Popen([
        '/usr/local/bin/cam', '-c', '1', '--capture=10000',
        '--file=/tmp/vis#.ppm', '-s', 'width=640,height=480'
    ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    print("Camera started!")
    while proc.poll() is None:
        files = sorted(glob.glob('/tmp/viscam*'))
        if len(files) >= 2:
            target = files[-2]
            if is_file_complete(target):
                frame = cv2.imread(target)
                if frame is not None:
                    with raw_lock:
                        raw_frame = frame.copy()
            for f in files[:-1]:
                try: os.remove(f)
                except: pass
        time.sleep(0.01)
    proc.terminate()

def run_detection(angle, desired_object=None):
    global latest_detections, detection_active
    print(f"Running detection at angle {angle} degrees, looking for: {desired_object}")

    start_time = time.time()
    found = False
    best = None
    final_frame = None

    while time.time() - start_time < 5.0:
        with raw_lock:
            frame = raw_frame.copy() if raw_frame is not None else None
        if frame is None:
            time.sleep(0.2)
            continue

        results = model(frame, conf=CONF_THRESHOLD, imgsz=320, verbose=False)[0]
        detections = []
        for box in results.boxes:
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            class_name = model.names[cls_id]
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            detections.append({
                'class': class_name,
                'class_id': cls_id,
                'confidence': round(conf, 3),
                'box': [x1, y1, x2, y2],
                'center_x': cx,
                'center_y': cy,
                'angle': angle
            })
            print(f"  Detected: {class_name} conf={conf:.2f} at angle {angle}")

        with detection_lock:
            latest_detections = detections

        if desired_object:
            matches = [d for d in detections if d['class'] == desired_object]
            if matches:
                best = max(matches, key=lambda d: d['confidence'])
                final_frame = frame.copy()
                found = True
                print(f"  Found desired object: {desired_object} conf={best['confidence']:.2f}")
                break
        elif detections:
            best = max(detections, key=lambda d: d['confidence'])
            final_frame = frame.copy()
            found = True
            break

        time.sleep(0.2)

    os.makedirs('/tmp/detections', exist_ok=True)
    timestamp = str(int(time.time()))
    wp_label = ros_node.current_wp_label if ros_node else 'X'
    img_filename = f"wp{wp_label}angle{angle}{timestamp}.jpg"
    img_path = f'/tmp/detections/{img_filename}'
    img_url = f'http://172.20.10.5:5000/images/{img_filename}'

    if found and best and final_frame is not None:
        annotated = final_frame.copy()
        x1, y1, x2, y2 = best['box']
        color = CLASS_COLORS.get(best['class_id'], (200,200,200))
        cv2.rectangle(annotated, (x1,y1), (x2,y2), color, 2)
        label = f"{best['class']} {best['confidence']:.2f}"
        cv2.putText(annotated, label, (x1,y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.imwrite(img_path, annotated)

        if ros_node:
            result = {
                'waypoint': wp_label,
                'angle': angle,
                'best_object': best['class'],
                'best_conf': best['confidence'],
                'image_path': img_url,
                'timestamp': timestamp,
                'found': True
            }
            msg = String()
            msg.data = json.dumps(result)
            ros_node.pub_result.publish(msg)
            print(f"Published detection result: {best['class']} at WP {wp_label}")
    else:
        print(f"  Desired object {desired_object} not found at angle {angle} after 30s")
        if ros_node:
            result = {
                'waypoint': wp_label,
                'angle': angle,
                'best_object': 'not_found',
                'best_conf': 0.0,
                'image_path': '',
                'timestamp': timestamp,
                'found': False
            }
            msg = String()
            msg.data = json.dumps(result)
            ros_node.pub_result.publish(msg)

    detection_active = False
    def clear_boxes():
        time.sleep(3)
        with detection_lock:
            latest_detections.clear()
    threading.Thread(target=clear_boxes, daemon=True).start()
    publish_vision_done(found=found)


def publish_vision_done(found=False):
    if ros_node:
        msg = Bool()
        msg.data = found
        ros_node.pub_vision_done.publish(msg)
        print(f"Published /vision_done found={found}")

def generate():
    while True:
        with raw_lock:
            frame = raw_frame.copy() if raw_frame is not None else None
        with detection_lock:
            current_detections = latest_detections.copy()

        if frame is not None:
            for d in current_detections:
                x1, y1, x2, y2 = d['box']
                color = CLASS_COLORS.get(d['class_id'], (200, 200, 200))
                label = f"{d['class']} {d['confidence']:.2f} @{d.get('angle','?')}°"
                cv2.rectangle(frame, (x1,y1), (x2,y2), color, 2)
                cv2.putText(frame, label, (x1, y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.03)

@app.route('/images/<filename>')
def serve_image(filename):
    from flask import send_from_directory
    return send_from_directory('/tmp/detections', filename)

@app.route('/video')
def video():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/detections')
def detections():
    with detection_lock:
        return jsonify(latest_detections)

@app.route('/')
def index():
    return '''
<html>
<head>
    <title>Vision Server (Triggered)</title>
    <style>
        body{background:#1a1a2e;color:white;font-family:Arial;margin:0;padding:20px}
        h1{text-align:center;color:#00d4ff}
        .container{display:flex;gap:20px;justify-content:center;align-items:flex-start}
        img{border:3px solid #00d4ff;border-radius:8px}
        .panel{background:#16213e;padding:20px;border-radius:10px;width:320px}
        .panel h2{color:#00d4ff;margin-top:0}
        .textbox{background:#0f3460;border:2px solid #00d4ff;border-radius:8px;padding:15px;margin-bottom:15px;text-align:center}
        .textbox-label{color:#aaa;font-size:12px;margin-bottom:5px}
        .textbox-value{color:#a8ff78;font-size:24px;font-weight:bold}
        .status{background:#0f3460;padding:10px;border-radius:8px;text-align:center;margin-bottom:15px}
        .det-item{background:#0f3460;margin:8px 0;padding:10px;border-radius:8px;border-left:4px solid #00d4ff}
    </style>
    <script>
        function update(){
            fetch('/detections').then(r=>r.json()).then(data=>{
                const status=document.getElementById('status');
                const bestObj=document.getElementById('best-object');
                const bestConf=document.getElementById('best-conf');
                const div=document.getElementById('det-list');
                if(data.length===0){
                    status.textContent='Waiting for trigger...';
                    status.style.color='#888';
                    bestObj.textContent='None';
                    bestConf.textContent='-';
                    div.innerHTML='<p style="color:#888;text-align:center">No detections yet</p>';
                }else{
                    status.textContent=data.length+' object(s) detected!';
                    status.style.color='#a8ff78';
                    const best=data.reduce((a,b)=>a.confidence>b.confidence?a:b);
                    bestObj.textContent=best.class;
                    bestConf.textContent=(best.confidence*100).toFixed(1)+'%';
                    div.innerHTML=data.map(d=>`
                        <div class="det-item">
                            <b style="color:#00d4ff">${d.class}</b>
                            <div style="color:#a8ff78">Conf: ${(d.confidence*100).toFixed(1)}%</div>
                            <div style="color:#aaa">Angle: ${d.angle}° | Center: (${d.center_x},${d.center_y})</div>
                        </div>`).join('');
                }
            });
        }
        setInterval(update, 500);
    </script>
</head>
<body>
    <h1>🤖 Vision Server (Triggered Mode)</h1>
    <div class="container">
        <img src="/video" width="640" height="480">
        <div class="panel">
            <h2>Detection Status</h2>
            <div id="status" class="status">Waiting for trigger...</div>
            <div class="textbox">
                <div class="textbox-label">BEST DETECTED OBJECT</div>
                <div class="textbox-value" id="best-object">-</div>
                <div class="textbox-label" style="margin-top:8px">CONFIDENCE</div>
                <div class="textbox-value" id="best-conf">-</div>
            </div>
            <h2>All Detections</h2>
            <div id="det-list"><p style="color:#888;text-align:center">No detections yet</p></div>
        </div>
    </div>
</body>
</html>'''

class VisionRosNode(Node):
    def __init__(self):
        super().__init__('vision_server_node')
        self.current_wp_label = 'X'
        self.pub_vision_done = self.create_publisher(Bool, '/vision_done', RELIABLE_QOS)
        self.pub_result = self.create_publisher(String, '/detection_result', 10)
        self.create_subscription(Int32, '/cam_ready_trigger', self.trigger_cb, RELIABLE_QOS)
        self.create_subscription(String, '/mission_state', self.state_cb, 10)
        self.get_logger().info('Vision ROS node ready — waiting for /cam_ready_trigger')

    def state_cb(self, msg):
        try:
            state = json.loads(msg.data)
            self.current_wp_label = state.get('current_wp', 'X')
        except:
            pass

    def trigger_cb(self, msg):
        global detection_active, last_trigger_angle
        angle = msg.data
        if detection_active:
            self.get_logger().warn(f'Already processing, ignoring trigger at {angle}°')
            return
        detection_active = True
        last_trigger_angle = angle
        desired = WAYPOINT_TARGETS.get(self.current_wp_label, None)
        self.get_logger().info(f'Triggered at angle {angle}°, looking for: {desired}')
        threading.Thread(target=run_detection, args=(angle, desired), daemon=True).start()

def ros_spin_loop():
    rclpy.init()
    global ros_node
    ros_node = VisionRosNode()
    rclpy.spin(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    threading.Thread(target=camera_loop, daemon=True).start()
    time.sleep(4)
    threading.Thread(target=ros_spin_loop, daemon=True).start()
    time.sleep(1)
    print("Vision server (triggered) at http://172.20.10.5:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)
