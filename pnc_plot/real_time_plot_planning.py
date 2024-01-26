import rospy
from sg_planning_msgs.msg import Trajectory
from sg_debug_msgs.msg import DebugInfo
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import copy
import math

sls_list = []
sll_list = []
sts_list = []
stt_list = []

dpsts_list = []
dpstt_list = []

obs_x_list_list = []
obs_y_list_list = []
obs_sls_list_list = []
obs_sll_list_list = []
bound_s_list_list = []
bound_t_list_list = []

x_point = []
y_point = []
s_point = []
v_point = []
ax_point = []
ay_point = []
t_point = []

t_now = 0
t_pre = 0

fig = plt.figure('real-time planning plot')

ax_xy = fig.add_subplot(2, 3, 1)
ax_sax = fig.add_subplot(2, 3, 2)
ax_say = fig.add_subplot(2, 3, 3)
ax_sv = fig.add_subplot(2, 3, 4)
ax_sl = fig.add_subplot(2, 3, 5)
ax_st = fig.add_subplot(2, 3, 6)

# line_xy, = ax_xy.plot([], [], lw=2)
# line_sax, = ax_sax.plot([], [], lw=2)
# line_say, = ax_say.plot([], [], lw=2)
# line_sv, = ax_sv.plot([], [], lw=2)
# line_sl, = ax_sl.plot([], [], lw=2)
# line_st, = ax_st.plot([], [], lw=2)
# line_enu, = ax_xy.plot([], [], lw=2)
# line_obs, = ax_sl.plot([], [], lw=2)


demo_point_mx = []
demo_point_my = []

with open("demo_qidi_0214_mid.json", "r") as f:
    for line in f.readlines():
        point_data = line.strip('\n')
        if(point_data.count(",") != 0):
            demo_mx = float(point_data.split("[")[1].split(",")[0])
            demo_my = float(point_data.split(",")[1].split(",")[0])
            demo_point_mx.append(demo_mx)
            demo_point_my.append(demo_my)


demo_point_lx = []
demo_point_ly = []

with open("demo_qidi_0214_left.json", "r") as f:
    for line in f.readlines():
        point_data = line.strip('\n')
        if(point_data.count(",") != 0):
            demo_lx = float(point_data.split("[")[1].split(",")[0])
            demo_ly = float(point_data.split(",")[1].split(",")[0])
            demo_point_lx.append(demo_lx)
            demo_point_ly.append(demo_ly)


demo_point_rx = []
demo_point_ry = []

with open("demo_qidi_0214_right.json", "r") as f:
    for line in f.readlines():
        point_data = line.strip('\n')
        if(point_data.count(",") != 0):
            demo_rx = float(point_data.split("[")[1].split(",")[0])
            demo_ry = float(point_data.split(",")[1].split(",")[0])
            demo_point_rx.append(demo_rx)
            demo_point_ry.append(demo_ry)


def callback_debuginfo(debuginfo_msg):
    bound_s_list_list.clear()
    bound_t_list_list.clear()
    sls_list.clear()
    sll_list.clear()
    sts_list.clear()
    stt_list.clear()
    dpsts_list.clear()
    dpstt_list.clear()
    obs_x_list_list.clear()
    obs_y_list_list.clear()
    obs_sls_list_list.clear()
    obs_sll_list_list.clear()

    i1 = 0
    i2 = 0
    i3 = 0
    i4 = 0
    i5 = 0
    i6 = 0
    i7 = 0
    i8 = 0
    info = debuginfo_msg.string_data
    #print(info)

    has_st_bound_msg = info.count("st_bound:")
    has_obstacles_msg = info.count("obstacles:")
    has_sl_line_msg = info.count("sl_line:")
    has_st_line_msg = info.count("st_line:")
    has_dpst_line_msg = info.count("stdp_line:")
    has_obs_sl_line_msg = info.count("obs_sl_line:")

    if(has_st_bound_msg):
        st_bound_msg = info.split("{st_bound:")[1].split("st_bound_end")[0]

        count_bound = st_bound_msg.count(">,")

        for i1 in range(count_bound):

            bound_info = st_bound_msg.split(">,")[i1].split("<")[1]
            if(bound_info.count(",@,") != 0):

                bound_down = bound_info.split(",@,")[0]
                bound_up = bound_info.split(",@,")[1]
                count_point_down = bound_down.count(")")
                bound_s_list = []
                bound_t_list = []
                for i2 in range(count_point_down):

                    point_down = bound_down.split(")")[i2].split("(")[1]
                    point_down_s = float(point_down.split(",")[0])
                    point_down_t = float(point_down.split(",")[1])
                    bound_s_list.append(point_down_s)
                    bound_t_list.append(point_down_t)

                count_point_up = bound_up.count(")")
                for i3 in range(count_point_up):

                    point_up = bound_up.split(
                        ")")[count_point_up-1-i3].split("(")[1]
                    point_up_s = float(point_up.split(",")[0])
                    point_up_t = float(point_up.split(",")[1])
                    bound_s_list.append(point_up_s)
                    bound_t_list.append(point_up_t)

                s_begin = bound_s_list[0]
                t_begin = bound_t_list[0]
                bound_s_list.append(s_begin)
                bound_t_list.append(t_begin)

                bound_s_list_list.append(bound_s_list)
                bound_t_list_list.append(bound_t_list)

    if(has_obstacles_msg):
        obstacles_msg = info.split("{obstacles:")[1].split("obstacles_end")[0]
        count_obs = obstacles_msg.count(")")
        for i4 in range(count_obs):
            obs_x_list = []
            obs_y_list = []
            obs_info = obstacles_msg.split(")")[i4].split("(")[1]
            obs_x = float(obs_info.split(":")[1].split(",")[0])
            obs_y = float(obs_info.split(":")[2].split(",")[0])
            obs_yaw = float(obs_info.split(":")[3].split(",")[0])
            obs_len = float(obs_info.split(":")[4].split(",")[0])
            obs_wid = float(obs_info.split(":")[5])
            delta = math.atan2(obs_wid, obs_len)
            l = math.sqrt((obs_wid/2)*(obs_wid/2)+(obs_len/2)*(obs_len/2))
            x1 = obs_x + l * math.cos(obs_yaw+delta)
            x2 = obs_x + l * math.cos(obs_yaw-delta)
            x3 = obs_x - l * math.cos(obs_yaw+delta)
            x4 = obs_x - l * math.cos(obs_yaw-delta)
            y1 = obs_y + l * math.sin(obs_yaw+delta)
            y2 = obs_y + l * math.sin(obs_yaw-delta)
            y3 = obs_y - l * math.sin(obs_yaw+delta)
            y4 = obs_y - l * math.sin(obs_yaw-delta)
            obs_x_list.append(x1)
            obs_x_list.append(x2)
            obs_x_list.append(x3)
            obs_x_list.append(x4)
            obs_x_list.append(x1)
            obs_y_list.append(y1)
            obs_y_list.append(y2)
            obs_y_list.append(y3)
            obs_y_list.append(y4)
            obs_y_list.append(y1)
            obs_x_list_list.append(obs_x_list)
            obs_y_list_list.append(obs_y_list)

    if(has_sl_line_msg):
        sl_line_msg = info.split("{sl_line:")[1].split("sl_line_end")[0]
        count_slpoint = sl_line_msg.count(")")

        for i5 in range(count_slpoint):
            sl_point = sl_line_msg.split(")")[i5].split("(")[1]
            point_sls = float(sl_point.split(",")[0])
            point_sll = float(sl_point.split(",")[1])
            sls_list.append(point_sls)
            sll_list.append(point_sll)

    if(has_st_line_msg):
        st_line_msg = info.split("{st_line:")[1].split("st_line_end")[0]
        count_stpoint = st_line_msg.count(")")
        for i6 in range(count_stpoint):
            st_point = st_line_msg.split(")")[i6].split("(")[1]
            point_sts = float(st_point.split(",")[0])
            point_stt = float(st_point.split(",")[1])
            sts_list.append(point_sts)
            stt_list.append(point_stt)

    if(has_dpst_line_msg):
        dpst_line_msg = info.split("{stdp_line:")[1].split("stdp_line_end")[0]
        count_dpstpoint = dpst_line_msg.count(")")
        for i8 in range(count_dpstpoint):
            st_point = dpst_line_msg.split(")")[i8].split("(")[1]
            point_sts = float(st_point.split(",")[0])
            point_stt = float(st_point.split(",")[1])
            dpsts_list.append(point_sts)
            dpstt_list.append(point_stt)

    if(has_obs_sl_line_msg):
        obs_sl_line_msg = info.split("{obs_sl_line:")[
            1].split("obs_sl_line_end")[0]
        count_obs = obs_sl_line_msg.count(">")
        for i7 in range(count_obs):
            obs_sls_list = []
            obs_sll_list = []
            obsl_info = obs_sl_line_msg.split(">")[i7]
            point1_info = obsl_info.split("(")[1]
            point1s = float(point1_info.split(",")[0])
            point1l = float(point1_info.split(",")[1].split(")")[0])
            point2_info = obsl_info.split("(")[2]
            point2s = float(point2_info.split(",")[0])
            point2l = float(point2_info.split(",")[1].split(")")[0])
            point3_info = obsl_info.split("(")[4]
            point3s = float(point3_info.split(",")[0])
            point3l = float(point3_info.split(",")[1].split(")")[0])
            point4_info = obsl_info.split("(")[3]
            point4s = float(point4_info.split(",")[0])
            point4l = float(point4_info.split(",")[1].split(")")[0])

            obs_sls_list.append(point1s)
            obs_sls_list.append(point2s)
            obs_sls_list.append(point3s)
            obs_sls_list.append(point4s)
            obs_sls_list.append(point1s)
            obs_sll_list.append(point1l)
            obs_sll_list.append(point2l)
            obs_sll_list.append(point3l)
            obs_sll_list.append(point4l)
            obs_sll_list.append(point1l)
            obs_sls_list_list.append(obs_sls_list)
            obs_sll_list_list.append(obs_sll_list)


def callback_trajectory(trajectory_msg):
    x_point.clear()
    y_point.clear()
    s_point.clear()
    v_point.clear()
    ax_point.clear()
    ay_point.clear()
    t_point.clear()

    for i in range(0, len(trajectory_msg.trajectory_points)-1):
        x = trajectory_msg.trajectory_points[i].position_m.x
        y = trajectory_msg.trajectory_points[i].position_m.y
        s = trajectory_msg.trajectory_points[i].s_m
        v = trajectory_msg.trajectory_points[i].velocity_mps
        ax = trajectory_msg.trajectory_points[i].acceleration_mps2
        k = trajectory_msg.trajectory_points[i].kappa
        ay = v*v*k
        t = trajectory_msg.trajectory_points[i].relative_time.secs

        x_point.append(x)
        y_point.append(y)
        s_point.append(s)
        v_point.append(v)
        ax_point.append(ax)
        ay_point.append(ay)
        t_point.append(t)
    global t_now
    t_now = rospy.Time.now().to_sec()


def data_gen(t=0.0):
    while True:
        yield x_point, y_point, s_point, v_point, ax_point, ay_point, bound_s_list_list,\
            bound_t_list_list, sls_list, sll_list, sts_list, stt_list, obs_x_list_list,\
            obs_y_list_list, obs_sls_list_list, obs_sll_list_list,dpsts_list, dpstt_list, 


def init():

    ax_xy.grid()
    ax_xy.set_ylim(0, 60)
    ax_xy.set_xlim(0, 60)
    ax_xy.set_title("trajectory")
    ax_xy.set_ylabel("m")
    ax_xy.set_xlabel("m")

    ax_sv.grid()
    ax_sv.set_ylim(0, 15)
    ax_sv.set_xlim(0, 40)
    ax_sv.set_title("s-v")
    ax_sv.set_ylabel("m/s")
    ax_sv.set_xlabel("m")

    ax_sax.grid()
    ax_sax.set_ylim(-5, 5)
    ax_sax.set_xlim(0, 40)
    ax_sax.set_title("s-ax")
    ax_sax.set_ylabel("m/s^2")
    ax_sax.set_xlabel("m")

    ax_say.grid()
    ax_say.set_ylim(-5, 5)
    ax_say.set_xlim(0, 40)
    ax_say.set_title("s-ay")
    ax_say.set_ylabel("m/s^2")
    ax_say.set_xlabel("m")

    ax_sl.grid()
    ax_sl.set_ylim(-10, 10)
    ax_sl.set_xlim(0, 80)
    ax_sl.set_title("s-l")
    ax_sl.set_ylabel("m")
    ax_sl.set_xlabel("m")

    ax_st.grid()
    ax_st.set_ylim(-10, 80)
    ax_st.set_xlim(0, 8)
    ax_st.set_title("s-t")
    ax_st.set_ylabel("m")
    ax_st.set_xlabel("s")
    return


def run(data):
    xpoint, ypoint, spoint, vpoint, axpoint, aypoint, bounds, boundt,\
        sls, sll, sts, stt, obsx, obsy, obss, obsl, dpsts, dpstt = data

    ax_xy.clear()
    ax_xy.grid()
    ax_xy.set_title("trajectory")
    ax_xy.set_ylabel("m")
    ax_xy.set_xlabel("m")

    ax_sv.clear()
    ax_sv.grid()
    ax_sv.set_ylim(0, 15)
    ax_sv.set_xlim(0, 40)
    ax_sv.set_title("s-v")
    ax_sv.set_ylabel("m/s")
    ax_sv.set_xlabel("m")

    ax_sax.clear()
    ax_sax.grid()
    ax_sax.set_ylim(-5, 5)
    ax_sax.set_xlim(0, 40)
    ax_sax.set_title("s-ax")
    ax_sax.set_ylabel("m/s^2")
    ax_sax.set_xlabel("m")

    ax_say.clear()
    ax_say.grid()
    ax_say.set_ylim(-5, 5)
    ax_say.set_xlim(0, 40)
    ax_say.set_title("s-ay")
    ax_say.set_ylabel("m/s^2")
    ax_say.set_xlabel("m")

    ax_sl.clear()
    ax_sl.grid()
    ax_sl.set_ylim(-10, 10)
    ax_sl.set_xlim(0, 80)
    ax_sl.set_title("s-l")
    ax_sl.set_ylabel("m")
    ax_sl.set_xlabel("m")

    ax_st.clear()
    ax_st.grid()
    ax_st.set_ylim(-10, 80)
    ax_st.set_xlim(0, 8)
    ax_st.set_title("s-t")
    ax_st.set_ylabel("m")
    ax_st.set_xlabel("s")

    xdata_xy = xpoint
    ydata_xy = ypoint
    if(xdata_xy):
        ax_xy.set_xlim(xdata_xy[0]-50, xdata_xy[0]+50)
        ax_xy.set_ylim(ydata_xy[0]-50, ydata_xy[0]+50)
    # else:
    #     ax_xy.set_xlim(-30, 30)
    #     ax_xy.set_ylim(-30, 30)
    line_xy, = ax_xy.plot([], [], lw=2)
    line_xy.set_data(xdata_xy, ydata_xy)
    i8 = 0
    for i8 in range(len(obsx)):
        try:
            line_xy, = ax_xy.plot([], [], lw=1, color='red')
            line_st.set_data(obsx[i8], obsy[i8])
        except:
            print("error_obs_xy")

    line_demom, = ax_xy.plot(demo_point_mx, demo_point_my, lw=0.5, color='red')
    line_demol, = ax_xy.plot(
        demo_point_lx, demo_point_ly, lw=0.5, color='orange')
    line_demor, = ax_xy.plot(
        demo_point_rx, demo_point_ry, lw=0.5, color='green')

    xdata_sax = spoint
    ydata_sax = axpoint
    line_sax, = ax_sax.plot([], [], lw=2)
    line_sax.set_data(xdata_sax, ydata_sax)

    xdata_say = spoint
    ydata_say = aypoint
    line_say, = ax_say.plot([], [], lw=2)
    line_say.set_data(xdata_say, ydata_say)

    xdata_sv = spoint
    ydata_sv = vpoint
    line_sv, = ax_sv.plot([], [], lw=2)
    line_sv.set_data(xdata_sv, ydata_sv)

    xdata_sl = sls
    ydata_sl = sll
    line_sl, = ax_sl.plot([], [], lw=2)
    line_sl.set_data(xdata_sl, ydata_sl)

    i9 = 0
    for i9 in range(len(obss)):
        try:
            line_sl, = ax_sl.plot([], [], lw=1, color='red')
            line_sl.set_data(obss[i9], obsl[i9])
        except:
            print("error_obs_sl")

    xdata_st = stt
    ydata_st = sts
    try:
        #print(ydata_st)
        line_st, = ax_st.plot([], [], lw=2)
        line_st.set_data(xdata_st, ydata_st)
    except:
        print("error st final")
    try:
        line_st, = ax_st.plot([], [], lw=2,color='orange')
        line_st.set_data(dpstt, dpsts)
    except:
        print("error dp st")
    i10 = 0
    for i10 in range(len(bounds)):
        try:
            line_st, = ax_st.plot([], [], lw=1, color='red')
            line_st.set_data(boundt[i10], bounds[i10])
        except:
            print("error_bound_st")
    return


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/planning/trajectory", Trajectory, callback_trajectory)
    rospy.Subscriber("/planning/debuginfo", DebugInfo, callback_debuginfo)


if __name__ == '__main__':
    try:
        listener()
        ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=10,
                                      repeat=False, init_func=init)
        plt.show()
    except rospy.ROSInterruptException:
        pass
