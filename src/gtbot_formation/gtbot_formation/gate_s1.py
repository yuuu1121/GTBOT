import json, time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from .mixer import yaw_of

class GateS1(Node):
    def __init__(self, robot='gtbot'):
        super().__init__('gate_s1')
        self.v = None
        self.create_subscription(Odometry, f'/{robot}/odometry', self.on_odom, 10)
        self.pub = self.create_publisher(Float64MultiArray, f'/{robot}/accel_cmd', 10)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = yaw_of(q.x, q.y, q.z, q.w)
        tw = msg.twist.twist.linear
        c, s = np.cos(yaw), np.sin(yaw)
        self.v = np.array([c * tw.x - s * tw.y, s * tw.x + c * tw.y])

    def run_axis(self, a_vec, v_expect):
        t0 = time.time()
        samples = []
        while time.time() - t0 < 8.0:
            self.pub.publish(Float64MultiArray(data=list(a_vec)))
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > 7.0 and self.v is not None:
                samples.append(self.v.copy())
        self.pub.publish(Float64MultiArray(data=[0.0, 0.0]))
        v_meas = np.mean(samples, axis=0)
        err = float(np.linalg.norm(v_meas - v_expect) / np.linalg.norm(v_expect))
        return {'v_meas': v_meas.tolist(), 'v_expect': list(v_expect), 'rel_err': err}

def main():
    rclpy.init()
    g = GateS1()
    while g.v is None:
        rclpy.spin_once(g, timeout_sec=0.2)
    rx = g.run_axis([0.2, 0.0], [0.5, 0.0])
    time.sleep(5)                              # 감속·안정화
    ry = g.run_axis([0.0, 0.2], [0.0, 0.5])
    out = {'x': rx, 'y': ry,
           'gate_s1_pass': bool(rx['rel_err'] < 0.1 and ry['rel_err'] < 0.1)}
    with open('results/s1_stonefish.json', 'w') as f:
        json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
