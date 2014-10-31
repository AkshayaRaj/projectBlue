#!/usr/bin/env python
import rospy, roslib
from nix_msgs.msg import dvl_vel
from nav_msgs.msg import Odometry
import numpy as np

#roslib.load_manifest('explorer_dvl')

class Kalman(object):
    def __init__(self):
        self.Xk_prev = np.array([[0], [0]])
        #self.Xk = np.array([[0], [0]])
        self.sigma_model = 1
        self.P = np.array([[self.sigma_model * self.sigma_model, 0],
                           [0, self.sigma_model * self.sigma_model]])
        self.Q = np.array([[0, 0], [0, 0]])
        self.H = np.array([[1, 0]])

        self.meas_cov_model = 0.2
        self.R = self.meas_cov_model * self.meas_cov_model

    def kalman1d(self, dt, pos, vel):
        A = np.array([[1, dt], [0, 1]])
        Q = np.array([[(dt * dt * dt * dt) / 4.0, (dt * dt * dt) / 3.0],
                     [(dt * dt * dt * dt) / 3.0, (dt * dt) / 2.0]])
        Q = Q * (0.26 * 0.26)
        meas = np.array([[pos], [vel]])
        Xk_pred = np.dot(A, self.Xk_prev)
        P_pred = A.dot(self.P).dot(np.transpose(A)) + Q
        s = np.dot(self.H.dot(P_pred),(np.transpose(self.H))) + self.R
        K = P_pred.dot(np.transpose(self.H)).dot(1/s)
        Xk = Xk_pred + K.dot(self.H.dot(meas) - (self.H.dot(Xk_pred)))
        self.P = (np.eye(2) - K.dot(self.H)).dot(P_pred)
        self.Xk_prev = Xk
        return Xk

class Filter(object):
    def __init__(self):
        self.data = None
        self.old_data = None
        rospy.Subscriber('/explorer_vel', dvl_vel, self._velcb)
        self.odo_pub = rospy.Publisher('/explorer_vel_filtered', Odometry)

        self.measured_x = 0
        self.measured_y = 0
        self.measured_z = 0

        self.kalman_x = Kalman()
        self.kalman_y = Kalman()
        self.kalman_z = Kalman()

        self.start = 0

    def _velcb(self, msg):
        msg.velocity.x = 0 if np.isnan(msg.velocity.x) else msg.velocity.x
        msg.velocity.y = 0 if np.isnan(msg.velocity.y) else msg.velocity.y
        msg.velocity.z = 0 if np.isnan(msg.velocity.z) else msg.velocity.z
        if self.old_data is None:
            self.old_data = msg
            return
        delta = msg.header.stamp.to_sec() - self.old_data.header.stamp.to_sec()

        self.measured_x += (self.old_data.velocity.x +
                            msg.velocity.x) * delta * 0.5;
        self.measured_y += (self.old_data.velocity.y +
                            msg.velocity.y) * delta * 0.5;
        self.measured_z += (self.old_data.velocity.z +
                            msg.velocity.z) * delta * 0.5;

        Xk = self.kalman_x.kalman1d(delta, self.measured_x, msg.velocity.x)
        Yk = self.kalman_y.kalman1d(delta, self.measured_y, msg.velocity.y)
        Zk = self.kalman_z.kalman1d(delta, self.measured_z, msg.velocity.z)

        self.start += delta
        print "%f,%f,%f" % (self.start, msg.velocity.x, Xk[1])

        o = Odometry()
        o.pose.pose.position.x = Xk[0]
        o.pose.pose.position.y = Yk[0]
        o.pose.pose.position.z = Zk[0]
        o.twist.twist.linear.x = Xk[1]
        o.twist.twist.linear.y = Yk[1]
        o.twist.twist.linear.z = Zk[1]

        o.header.stamp = msg.header.stamp
        o.header.frame_id = "odometry_filtered"
        self.odo_pub.publish(o)

        self.old_data = msg


rospy.init_node('linear_kalman', anonymous=False)
k = Filter()
rospy.spin()
