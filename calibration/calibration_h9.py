import rospy
import math
from sg_sensor_msgs.msg import Imu
from sg_vehicle_msgs.msg import ThrottleCmd
from sg_vehicle_msgs.msg import BrakeCmd
from sg_vehicle_msgs.msg import Speed
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import message_filters
import os


class Data_collector(object):

    # def callback_throttle_report(throttlereport_msg):
    #     # if throttlereport_msg.percent_command > 0:
    #     print(
    #         f"throttle:{throttlereport_msg.percent_actual},time:{throttlereport_msg.header.stamp}")

    # def callback_brake_report(brake_msg):
    #     print(f"brake:{brake_msg.percent_actual}")
    # def callback_accel(self, accel_msg):
    #     s_t = accel_msg.header.stamp.secs
    #     n_t = accel_msg.header.stamp.nsecs
    #     n_t1 = accel_msg.header.stamp.nsecs/1000000000
    #     #t = accel_msg.header.stamp.to_sec()
    #     now = rospy.get_rostime()
    #     #t = rospy.Time.from_sec(123456.789)
    #     print(s_t)
    #     print(n_t1)
    #     print(now)
    # print(f"accel:{accel_msg.accel_x},time:{accel_msg.header.stamp}")

    # def callback_speed(speed_msg):
    #     print(f"speed:{speed_msg.mps},time:{speed_msg.header.stamp}")

    def multi_callback(self, throttle_sub, brake_sub, acc_sub, speed_sub):
        # print(f"thro:{throttle_sub.percent_actual},time:{throttle_sub.header.stamp}")
        # print(f"acc:{acc_sub.accel_x},time:{acc_sub.header.stamp}")

        thro = throttle_sub.command
        brake = brake_sub.command
        vel = speed_sub.mps
        acc = acc_sub.accel.y
        time = acc_sub.header.stamp.secs + acc_sub.header.stamp.nsecs/1000000000

        if thro >= brake:
            brake = 0
        if thro < brake:
            thro = 0
        if vel > 0:
            self.file.write(
                "%s,%s,%s,%s,%s\n" % (
                    time, thro, brake, vel, acc
                )
            )

    def calib(self):
        rospy.init_node('calib', anonymous=True)
        # rospy.Subscriber("/vehicle/throttle/report",
        #                  ThrottleReport, callback_throttle_report)
        # rospy.Subscriber("/vehicle/brake/report",
        #                  BrakeReport, callback_brake_report)
        #rospy.Subscriber("/sensor/ins/rawimu", Imu, self.callback_accel)
        # rospy.Subscriber("/vehicle/chassis/vehicle_speed",
        #                  Speed, callback_speed)

        i = 0
        out = "h9_calib_test_"
        outfile = out + str(i) + '_recorded.csv'
        while os.path.exists(outfile):
            i += 1
            outfile = out + str(i) + '_recorded.csv'
        self.file = open(outfile, 'w')
        self.file.write(
            "time_stamp,throttle_percentage,brake_percentage,speed,acc\n"
        )

        throttle_sub = message_filters.Subscriber("/vehicle/throttle/command",
                                                  ThrottleCmd)
        brake_sub = message_filters.Subscriber("/vehicle/brake/command",
                                               BrakeCmd)
        acc_sub = message_filters.Subscriber("/sensor/ins/rawimu", Imu)
        speed_sub = message_filters.Subscriber("/vehicle/chassis/vehicle_speed",
                                               Speed)

        ts = message_filters.ApproximateTimeSynchronizer(
            [throttle_sub, brake_sub, acc_sub, speed_sub], 10, 0.02, allow_headerless=True)
        # 对齐时间辍，将获取消息的频率统一为0.02
        ts.registerCallback(self.multi_callback)

        rospy.spin()


if __name__ == '__main__':
    data_collector = Data_collector()
    try:
        data_collector.calib()
        # ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=10,
        #                               repeat=False, init_func=init)

        # plt.show()

    except rospy.ROSInterruptException:
        pass
