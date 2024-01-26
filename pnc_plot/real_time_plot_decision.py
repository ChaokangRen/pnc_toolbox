import rospy
from sg_planning_msgs.msg import Trajectory
from sg_debug_msgs.msg import DebugInfo
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import copy
import math

dps_list = []
dpl_list = []
dpls_list = []
dpll_list = []

l1s_list = []
l1l_list = []
l2s_list = []
l2l_list = []
l3s_list = []
l3l_list = []

sls_list = []
sll_list = []
obs_sls_list_list = []
obs_sll_list_list = []

target_line_id_list = [0]
decide_status_list = ["LANE_FOLLOW"]

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

ax_xy = fig.add_subplot(2, 1, 1)
ax_dp = fig.add_subplot(2, 1, 2)

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
    dps_list.clear()
    dpl_list.clear()
    dpls_list.clear()
    dpll_list.clear()

    l1s_list.clear()
    l1l_list.clear()
    l2s_list.clear()
    l2l_list.clear()
    l3s_list.clear()
    l3l_list.clear()

    sls_list.clear()
    sll_list.clear()

    obs_sls_list_list.clear()
    obs_sll_list_list.clear()

    i1 = 0
    i2 = 0
    info = debuginfo_msg.string_data

    has_dp_map_msg = info.count("dp_map:")
    has_dp_line_msg = info.count("dp_line:")
    has_refline_msg = info.count("refline:")
    has_decide_status_msg = info.count("decide_status:")
    has_sl_line_msg = info.count("sl_line:")
    has_obs_sl_line_msg = info.count("obs_sl_line:")

    if(has_sl_line_msg):
        sl_line_msg = info.split("sl_line:")[1].split("sl_line_end")[0]

    count_slpoint = sl_line_msg.count(")")
    for i5 in range(count_slpoint):
        sl_point = sl_line_msg.split(")")[i5].split("(")[1]
        point_sls = float(sl_point.split(",")[0])
        point_sll = float(sl_point.split(",")[1])
        sls_list.append(point_sls)
        sll_list.append(point_sll)

    if(has_dp_map_msg):
        dp_map_msg = info.split("dp_map:")[1].split("dp_map_end")[0]
        count_dppoint = dp_map_msg.count("),")
        for i1 in range(count_dppoint):
            dp_point = dp_map_msg.split("),")[i1].split("(")[1]
            point_dps = float(dp_point.split(",")[0])
            point_dpl = float(dp_point.split(",")[1])
            dps_list.append(point_dps)
            dpl_list.append(point_dpl)

    if(has_dp_line_msg):
        dp_line_msg = info.split("dp_line:")[1].split("dp_line_end")[0]
        count_dppoint = dp_line_msg.count("),")
        for i2 in range(count_dppoint):
            dp_point = dp_line_msg.split("),")[i2].split("(")[1]
            point_dps = float(dp_point.split(",")[0])
            point_dpl = float(dp_point.split(",")[1])
            dpls_list.append(point_dps)
            dpll_list.append(point_dpl)

    if(has_refline_msg):
        has_target_line_msg = info.count("target_line:")
        has_line1_msg = info.count("line0:")
        has_line2_msg = info.count("line1:")
        has_line3_msg = info.count("line2:")

        i3 = 0
        i4 = 0
        i5 = 0

        if(has_target_line_msg):
            target_line_msg = info.split("target_line:")[1].split("]")[0]
            target_line_id = int(target_line_msg)
            target_line_id_list.append(target_line_id)

        if(has_line1_msg):
            line1_msg = info.split("line0:")[1].split("line0_end")[0]
            count_l1point = line1_msg.count("),")
            for i3 in range(count_l1point):
                dp_point = line1_msg.split("),")[i3].split("(")[1]
                point_l1s = float(dp_point.split(",")[0])
                point_l1l = float(dp_point.split(",")[1])
                l1s_list.append(point_l1s)
                l1l_list.append(point_l1l)

        if(has_line2_msg):
            line2_msg = info.split("line1:")[1].split("line1_end")[0]
            count_l2point = line2_msg.count("),")
            for i4 in range(count_l2point):
                dp_point = line2_msg.split("),")[i4].split("(")[1]
                point_l2s = float(dp_point.split(",")[0])
                point_l2l = float(dp_point.split(",")[1])
                l2s_list.append(point_l2s)
                l2l_list.append(point_l2l)

        if(has_line3_msg):
            line3_msg = info.split("line2:")[1].split("line2_end")[0]
            count_l3point = line3_msg.count("),")
            for i5 in range(count_l3point):
                dp_point = line3_msg.split("),")[i5].split("(")[1]
                point_l3s = float(dp_point.split(",")[0])
                point_l3l = float(dp_point.split(",")[1])
                l3s_list.append(point_l3s)
                l3l_list.append(point_l3l)

    if(has_decide_status_msg):
        decide_status_msg = info.split("decide_status:")[
            1].split("decide_status_end")[0]
        decide_status_list.append(decide_status_msg)

    if(has_obs_sl_line_msg):
        obs_sl_line_msg = info.split("obs_sl_line:")[
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
        t = trajectory_msg.trajectory_points[i].relative_time.secs

        x_point.append(x)
        y_point.append(y)
        t_point.append(t)
    global t_now
    t_now = rospy.Time.now().to_sec()


def data_gen(t=0.0):
    while True:
        yield x_point, y_point, dps_list, dpl_list, dpls_list, dpll_list, \
            l1s_list, l1l_list, l2s_list, l2l_list, l3s_list, l3l_list, \
            target_line_id_list[-1], decide_status_list[-1], sls_list, sll_list, \
            obs_sls_list_list, obs_sll_list_list


def init():

    ax_xy.grid()
    ax_xy.set_ylim(0, 60)
    ax_xy.set_xlim(0, 60)
    ax_xy.set_title("trajectory")
    ax_xy.set_ylabel("m")
    ax_xy.set_xlabel("m")

    ax_dp.grid()
    ax_dp.set_ylim(-5, 5)
    ax_dp.set_xlim(0, 80)
    ax_dp.set_title("dp in frenet")
    ax_dp.set_ylabel("l:m")
    ax_dp.set_xlabel("s:m")

    return


def run(data):
    xpoint, ypoint, dps, dpl, dpls, dpll, l1s, l1l, l2s, \
    l2l, l3s, l3l, tar_id, decide_status, sls, sll, obss, obsl = data

    ax_xy.clear()
    ax_xy.grid()
    ax_xy.set_title("trajectory")
    ax_xy.set_ylabel("m")
    ax_xy.set_xlabel("m")

    ax_dp.clear()
    ax_dp.grid()
    ax_dp.set_ylim(-5, 5)
    ax_dp.set_xlim(0, 80)
    ax_dp.set_title("dp in frenet")
    ax_dp.set_ylabel("l:m")
    ax_dp.set_xlabel("s:m")

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

    line_demom, = ax_xy.plot(demo_point_mx, demo_point_my, lw=0.5, color='red')
    line_demol, = ax_xy.plot(
        demo_point_lx, demo_point_ly, lw=0.5, color='orange')
    line_demor, = ax_xy.plot(
        demo_point_rx, demo_point_ry, lw=0.5, color='green')

    line_dp = ax_dp.scatter(dps, dpl, s=1, marker='x', lw=5)
    line_dpl, = ax_dp.plot([], [], lw=2, color='blue')
    line_dpl.set_data(dpls, dpll)

    if tar_id == 0:
        line_rl1, = ax_dp.plot([], [], lw=1.5, color='black')
    else:
        line_rl1, = ax_dp.plot([], [], lw=1.5, color='grey')
    line_rl1.set_data(l1s, l1l)

    if tar_id == 1:
        line_rl2, = ax_dp.plot([], [], lw=1.5, color='black')
    else:
        line_rl2, = ax_dp.plot([], [], lw=1.5, color='grey')
    line_rl2.set_data(l2s, l2l)

    if tar_id == 2:
        line_rl3, = ax_dp.plot([], [], lw=1.5, color='black')
    else:
        line_rl3, = ax_dp.plot([], [], lw=1.5, color='grey')
    line_rl3.set_data(l3s, l3l)

    xdata_sl = sls
    ydata_sl = sll
    line_sl, = ax_dp.plot([], [], lw=2, color='red')
    line_sl.set_data(xdata_sl, ydata_sl)

    i9 = 0
    for i9 in range(len(obss)):
        try:
            line_sl, = ax_dp.plot([], [], lw=0.5, color='red')
            line_sl.set_data(obss[i9], obsl[i9])
        except:
            print("error_obs_sl")

    decision_text = "target_line:" + str(tar_id) + "\n" + str(decide_status)
    ax_dp.text(80, 5, decision_text, fontsize=12, color="black",
               verticalalignment='top', horizontalalignment='right')

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
