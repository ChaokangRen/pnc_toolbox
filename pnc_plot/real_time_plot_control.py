import rospy
import math
from sg_debug_msgs.msg import DebugInfo
from sg_planning_msgs.msg import Trajectory
from sg_vehicle_msgs.msg import SteeringReport
import matplotlib.pyplot as plt
import matplotlib.animation as animation


steer_r_list = [0]
steer_e_list = [0]
acc_r_list = [0]
acc_e_list = [0]
vel_r_list = [0]
vel_e_list = [0]
jerk_list = [0]
dsteer_list = [0]
serror_list = [0]
verror_list = [0]
ey_list = [0]


xdata_steer_r, ydata_steer_r = [], []
xdata_steer_e, ydata_steer_e = [], []
xdata_acc_r, ydata_acc_r = [], []
xdata_acc_e, ydata_acc_e = [], []
xdata_vel_r, ydata_vel_r = [], []
xdata_vel_e, ydata_vel_e = [], []
xdata_jerk, ydata_jerk = [], []
xdata_dsteer, ydata_dsteer = [], []
xdata_serror, ydata_serror = [], []
xdata_verror, ydata_verror = [], []
xdata_ey, ydata_ey = [], []

t_now = 0
t_pre = 0
is_init = 0

t_now_debug = 0
t_pre_debug = 0


fig = plt.figure('real-time control plot')

# ax_pos = fig.add_subplot(2, 4, 1)
ax_steer = fig.add_subplot(2, 4, 1)
ax_acc = fig.add_subplot(2, 4, 2)
ax_vel = fig.add_subplot(2, 4, 3)
ax_jerk = fig.add_subplot(2, 4, 4)
ax_dsteer = fig.add_subplot(2, 4, 5)
ax_serror = fig.add_subplot(2, 4, 6)
ax_verror = fig.add_subplot(2, 4, 7)
ax_ey = fig.add_subplot(2, 4, 8)

# ax_theta = fig.add_subplot(2, 4, 8)
# ax_r = fig.add_subplot(2, 4, 8)

line_steer_r, = ax_steer.plot([], [], lw=2, color='blue', label='real')
line_steer_e, = ax_steer.plot([], [], lw=2, color='red', label='expected')
line_acc_r, = ax_acc.plot([], [], lw=2, color='blue', label='real')
line_acc_e, = ax_acc.plot([], [], lw=2, color='red', label='expected')
line_vel_r, = ax_vel.plot([], [], lw=2, color='blue', label='real')
line_vel_e, = ax_vel.plot([], [], lw=2, color='red', label='expected')
line_jerk, = ax_jerk.plot([], [], lw=2)
line_dsteer, = ax_dsteer.plot([], [], lw=2)
line_serror, = ax_serror.plot([], [], lw=2)
line_verror, = ax_verror.plot([], [], lw=2)
line_ey, = ax_ey.plot([], [], lw=2)

# line_theta, = ax_theta.plot([], [], lw=2)
# line_pos, = ax_pos.plot([], [], lw=2)
# line_r, = ax_r.plot([], [], lw=2)


def callback_debuginfo(debuginfo_msg):

    info = debuginfo_msg.string_data
    if(info.count("steer_real=")):
        steer_r = float(info.split("steer_real=")[1].split(";")[0])
        steer_r_list.append(steer_r)
    if(info.count("steer_cmd=")):
        steer_e = float(info.split("steer_cmd=")[1].split(";")[0])
        steer_e_list.append(steer_e)
    if(info.count("acc_real=")):
        acc_r = float(info.split("acc_real=")[1].split(";")[0])
        acc_r_list.append(acc_r)
    if(info.count("acc_cmd=")):
        acc_e = float(info.split("acc_cmd=")[1].split(";")[0])
        acc_e_list.append(acc_e)
    if(info.count("vel_real=")):
        vel_r = float(info.split("vel_real=")[1].split(";")[0])
        vel_r_list.append(vel_r)
    if(info.count("vel_ref=")):
        vel_e = float(info.split("vel_ref=")[1].split(";")[0])
        vel_e_list.append(vel_e)
    if(info.count("jerk=")):
        jerk = float(info.split("jerk=")[1].split(";")[0])
        jerk_list.append(jerk)
    if(info.count("dsteer_cmd=")):
        dsteer = float(info.split("dsteer_cmd=")[1].split(";")[0])
        dsteer_list.append(dsteer)
    if(info.count("station_error=")):
        serror = float(info.split("station_error=")[1].split(";")[0])
        serror_list.append(serror)
    if(info.count("speed_error=")):
        verror = float(info.split("speed_error=")[1].split(";")[0])
        verror_list.append(verror)
    if(info.count("ey=")):
        ey = float(info.split("ey=")[1].split(";")[0])
        ey_list.append(ey)

    global t_now
    global t_pre
    global is_init

    t_now = rospy.Time.now().to_sec()
    if (is_init == 0):
        t_pre = t_now
        is_init = 1
    global t_now_debug
    t_now_debug = rospy.Time.now().to_sec()


def data_gen(t=0.0):
    global t_now
    global t_pre
    global t_now_debug
    global t_pre_debug

    while True:
        t += (t_now - t_pre)
        t_pre = t_now
        # dt_debug = t_now_debug - t_pre_debug
        # t_pre_debug = t_now_debug
        yield t, steer_r_list[-1], steer_e_list[-1], acc_r_list[-1], acc_e_list[-1], vel_r_list[-1],\
            vel_e_list[-1], jerk_list[-1], dsteer_list[-1], serror_list[-1], verror_list[-1], ey_list[-1]


def init():
    # ax_pos.grid()
    # ax_pos.set_ylim(0, 20)
    # ax_pos.set_xlim(0, 20)
    # ax_pos.set_title("position")
    # ax_pos.set_ylabel("m")
    # ax_pos.set_xlabel("m")
    # del xdata_pos[:]
    # del ydata_pos[:]
    # line_pos.set_data(xdata_pos, ydata_pos)

    ax_steer.grid()
    ax_steer.set_ylim(-2, 2)
    ax_steer.set_xlim(0, 10)
    ax_steer.set_title("steer")
    ax_steer.set_ylabel("rad")
    ax_steer.set_xlabel("s")
    ax_steer.legend(loc='upper right')
    del xdata_steer_r[:]
    del ydata_steer_r[:]
    del xdata_steer_e[:]
    del ydata_steer_e[:]
    line_steer_r.set_data(xdata_steer_r, ydata_steer_r)
    line_steer_e.set_data(xdata_steer_e, ydata_steer_e)

    ax_acc.grid()
    ax_acc.set_ylim(-3, 3)
    ax_acc.set_xlim(0, 10)
    ax_acc.set_title("acc")
    ax_acc.set_ylabel("m/s^2")
    ax_acc.set_xlabel("s")
    ax_acc.legend(loc='upper right')
    del xdata_acc_r[:]
    del ydata_acc_r[:]
    del xdata_acc_e[:]
    del ydata_acc_e[:]
    line_acc_r.set_data(xdata_acc_r, ydata_acc_r)
    line_acc_e.set_data(xdata_acc_e, ydata_acc_e)

    ax_vel.grid()
    ax_vel.set_ylim(0, 18)
    ax_vel.set_xlim(0, 10)
    ax_vel.set_title("vel")
    ax_vel.set_ylabel("m/s")
    ax_vel.set_xlabel("s")
    ax_vel.legend(loc='upper right')
    del xdata_vel_r[:]
    del ydata_vel_r[:]
    del xdata_vel_e[:]
    del ydata_vel_e[:]
    line_vel_r.set_data(xdata_vel_r, ydata_vel_r)
    line_vel_e.set_data(xdata_vel_e, ydata_vel_e)

    ax_jerk.grid()
    ax_jerk.set_ylim(-5, 5)
    ax_jerk.set_xlim(0, 10)
    ax_jerk.set_title("jerk")
    ax_jerk.set_ylabel("m/s^3")
    ax_jerk.set_xlabel("s")
    del xdata_jerk[:]
    del ydata_jerk[:]
    line_jerk.set_data(xdata_jerk, ydata_jerk)

    ax_dsteer.grid()
    ax_dsteer.set_ylim(-1, 1)
    ax_dsteer.set_xlim(0, 10)
    ax_dsteer.set_title("dsteer")
    ax_dsteer.set_ylabel("rad/s")
    ax_dsteer.set_xlabel("s")
    del xdata_dsteer[:]
    del ydata_dsteer[:]
    line_dsteer.set_data(xdata_dsteer, ydata_dsteer)

    ax_serror.grid()
    ax_serror.set_ylim(-1, 1)
    ax_serror.set_xlim(0, 10)
    ax_serror.set_title("serror")
    ax_serror.set_ylabel("m")
    ax_serror.set_xlabel("s")
    del xdata_serror[:]
    del ydata_serror[:]
    line_serror.set_data(xdata_serror, ydata_serror)

    ax_verror.grid()
    ax_verror.set_ylim(-3, 3)
    ax_verror.set_xlim(0, 10)
    ax_verror.set_title("verror")
    ax_verror.set_ylabel("m/s")
    ax_verror.set_xlabel("s")
    del xdata_verror[:]
    del ydata_verror[:]
    line_verror.set_data(xdata_verror, ydata_verror)

    ax_ey.grid()
    ax_ey.set_ylim(-1, 1)
    ax_ey.set_xlim(0, 10)
    ax_ey.set_title("ey")
    ax_ey.set_ylabel("m")
    ax_ey.set_xlabel("s")
    del xdata_ey[:]
    del ydata_ey[:]
    line_ey.set_data(xdata_ey, ydata_ey)

    return line_steer_r, line_steer_e, line_acc_r, line_acc_e, line_vel_r, line_vel_e, line_jerk, \
        line_dsteer, line_serror, line_verror, line_ey


def run(data):

    t, steer_r, steer_e, acc_r, acc_e, vel_r, vel_e, jerk, dsteer, serror, verror, ey = data

    xdata_steer_r.append(t)
    ydata_steer_r.append(steer_r)
    xdata_steer_e.append(t)
    ydata_steer_e.append(steer_e)
    xmin_steer, xmax_steer = ax_steer.get_xlim()
    ax_steer.set_xlim(t - 9, t+1)

    xdata_acc_r.append(t)
    ydata_acc_r.append(acc_r)
    xdata_acc_e.append(t)
    ydata_acc_e.append(acc_e)
    xmin_acc, xmax_acc = ax_acc.get_xlim()
    ax_acc.set_xlim(t - 9, t+1)

    xdata_vel_r.append(t)
    ydata_vel_r.append(vel_r)
    xdata_vel_e.append(t)
    ydata_vel_e.append(vel_e)
    xmin_vel, xmax_vel = ax_vel.get_xlim()
    ax_vel.set_xlim(t - 9, t+1)

    xdata_jerk.append(t)
    ydata_jerk.append(jerk)
    xmin_jerk, xmax_jerk = ax_jerk.get_xlim()
    ax_jerk.set_xlim(t - 9, t+1)

    xdata_dsteer.append(t)
    ydata_dsteer.append(dsteer)
    xmin_dsteer, xmax_dsteer = ax_dsteer.get_xlim()
    ax_dsteer.set_xlim(t - 9, t+1)

    xdata_serror.append(t)
    ydata_serror.append(serror)
    xmin_serror, xmax_serror = ax_serror.get_xlim()
    ax_serror.set_xlim(t - 9, t+1)

    xdata_verror.append(t)
    ydata_verror.append(verror)
    xmin_verror, xmax_verror = ax_verror.get_xlim()
    ax_verror.set_xlim(t - 9, t+1)

    xdata_ey.append(t)
    ydata_ey.append(ey)
    xmin_ey, xmax_ey = ax_ey.get_xlim()
    ax_ey.set_xlim(t - 9, t+1)

    # xdata_pos.append(x_pos)
    # ydata_pos.append(y_pos)
    # xmin, xmax = ax_pos.get_xlim()
    # ax_pos.set_ylim(y[-1]-10, y[-1]+10)
    # ax_pos.set_xlim(x[-1]-10, x[-1]+10)

    if t >= xmax_steer:
        ax_steer.set_xlim(xmin_steer, 2*xmax_steer)
        ax_steer.figure.canvas.draw()
    line_steer_r.set_data(xdata_steer_r, ydata_steer_r)
    line_steer_e.set_data(xdata_steer_e, ydata_steer_e)

    if t >= xmax_acc:
        ax_acc.set_xlim(xmin_acc, 2*xmax_acc)
        ax_acc.figure.canvas.draw()
    line_acc_r.set_data(xdata_acc_r, ydata_acc_r)
    line_acc_e.set_data(xdata_acc_e, ydata_acc_e)

    if t >= xmax_vel:
        ax_vel.set_xlim(xmin_vel, 2*xmax_vel)
        ax_vel.figure.canvas.draw()
    line_vel_r.set_data(xdata_vel_r, ydata_vel_r)
    line_vel_e.set_data(xdata_vel_e, ydata_vel_e)

    if t >= xmax_jerk:
        ax_jerk.set_xlim(xmin_jerk, 2*xmax_jerk)
        ax_jerk.figure.canvas.draw()
    line_jerk.set_data(xdata_jerk, ydata_jerk)

    if t >= xmax_dsteer:
        ax_dsteer.set_xlim(xmin_dsteer, 2*xmax_dsteer)
        ax_dsteer.figure.canvas.draw()
    line_dsteer.set_data(xdata_dsteer, ydata_dsteer)

    if t >= xmax_serror:
        ax_serror.set_xlim(xmin_serror, 2*xmax_serror)
        ax_serror.figure.canvas.draw()
    line_serror.set_data(xdata_serror, ydata_serror)

    if t >= xmax_verror:
        ax_verror.set_xlim(xmin_verror, 2*xmax_verror)
        ax_verror.figure.canvas.draw()
    line_verror.set_data(xdata_verror, ydata_verror)

    if t >= xmax_ey:
        ax_ey.set_xlim(xmin_ey, 2*xmax_ey)
        ax_ey.figure.canvas.draw()
    line_ey.set_data(xdata_ey, ydata_ey)

    return line_steer_r, line_steer_e, line_acc_r, line_acc_e, line_vel_r, line_vel_e, line_jerk, \
        line_dsteer, line_serror, line_verror, line_ey


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/control/debuginfo", DebugInfo, callback_debuginfo)


if __name__ == '__main__':
    try:
        listener()
        ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=10,
                                      repeat=False, init_func=init)

        plt.show()

    except rospy.ROSInterruptException:
        pass
