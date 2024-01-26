import rospy
from sg_debug_msgs.msg import DebugInfo
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math


lane_x_list_list = []
lane_y_list_list = []

obs_x_list_list = []
obs_y_list_list = []

point_x_list_list = []
point_y_list_list = []

solidlane_x_list_list = []
solidlane_y_list_list = []
brokenlane_x_list_list = []
brokenlane_y_list_list = []

stop_x_list_list = []
stop_y_list_list = []

xdata_lane, ydata_lane = [], []
xdata_obs, ydata_obs = [], []

xdata_echo = [1, -1, -1, 1, 1]
ydata_echo = [2.4, 2.4, -2.4, -2.4, 2.4]

t_now = 0
t_pre = 0

fig = plt.figure('real-time prediction plot')

ax_xy = fig.add_subplot(111, aspect=1)


def callback_debuginfo(debuginfo_msg):

    lane_x_list_list.clear()
    lane_y_list_list.clear()
    obs_x_list_list.clear()
    obs_y_list_list.clear()
    point_x_list_list.clear()
    point_y_list_list.clear()
    solidlane_x_list_list.clear()
    solidlane_y_list_list.clear()
    brokenlane_x_list_list.clear()
    brokenlane_y_list_list.clear()
    stop_x_list_list.clear()
    stop_y_list_list.clear()

    i1 = 0

    info = debuginfo_msg.string_data

    has_lane_msg = info.count("{lane:")
    has_obstacles_msg = info.count("{obstacles:")
    has_indicator_msg = info.count("{indicator:")
    has_section_msg = info.count("{section:")

    if(has_lane_msg):
        lane_info = info.split("{lane:")[1].split("lane_end}")[0]
        count_lane = lane_info.count("[")
        for i1 in range(count_lane):
            lane = lane_info.split("]")[i1]
            i2 = 0
            count_lane_point = lane.count(")")
            lane_x_list = []
            lane_y_list = []
            for i2 in range(count_lane_point):
                lane_point = lane.split(")")[i2]
                lane_x = float(lane_point.split(",")[0].split("(")[1])
                lane_y = float(lane_point.split(",")[1])
                lane_x_list.append(lane_x)
                lane_y_list.append(lane_y)
            lane_x_list_list.append(lane_x_list)
            lane_y_list_list.append(lane_y_list)

    if(has_obstacles_msg):
        obs_info = info.split("{obstacles:")[1].split("obstacles_end}")[0]
        count_obs = obs_info.count("state:")
        print(count_obs)

        i3 = 0

        for i3 in range(count_obs):
            obs_x_list = []
            obs_y_list = []

            obs = obs_info.split("state:")[i3 + 1]

            obs_state = obs.split("trajectory:")[0]
            obs_traj = obs.split("trajectory:")[1]

            obs_x = float(obs_state.split("[")[1].split(",")[0])
            obs_y = float(obs_state.split("[")[1].split(",")[1])
            obs_yaw = float(obs_state.split("[")[1].split(",")[2])
            obs_width = float(obs_state.split("[")[1].split(",")[3])
            obs_length = float(obs_state.split(
                "[")[1].split(",")[4].split("]")[0])

            x0 = obs_length / 2
            y0 = obs_width / 2

            x1 = -obs_length / 2
            y1 = obs_width / 2

            x2 = -obs_length / 2
            y2 = -obs_width / 2

            x3 = obs_length / 2
            y3 = -obs_width / 2

            rotation = math.pi / 2 - obs_yaw

            x0r = x0 * math.cos(rotation) + y0 * math.sin(rotation) + obs_x
            y0r = y0 * math.cos(rotation) - x0 * math.sin(rotation) + obs_y

            x1r = x1 * math.cos(rotation) + y1 * math.sin(rotation) + obs_x
            y1r = y1 * math.cos(rotation) - x1 * math.sin(rotation) + obs_y

            x2r = x2 * math.cos(rotation) + y2 * math.sin(rotation) + obs_x
            y2r = y2 * math.cos(rotation) - x2 * math.sin(rotation) + obs_y

            x3r = x3 * math.cos(rotation) + y3 * math.sin(rotation) + obs_x
            y3r = y3 * math.cos(rotation) - x3 * math.sin(rotation) + obs_y

            obs_x_list .append(x0r)
            obs_x_list .append(x1r)
            obs_x_list .append(x2r)
            obs_x_list .append(x3r)
            obs_x_list .append(x0r)

            obs_y_list .append(y0r)
            obs_y_list .append(y1r)
            obs_y_list .append(y2r)
            obs_y_list .append(y3r)
            obs_y_list .append(y0r)

            obs_x_list_list .append(obs_x_list)
            obs_y_list_list .append(obs_y_list)

            count_traj_point = obs_traj.count(")")

            i6 = 0

            point_x_list = []
            point_y_list = []
            for i6 in range(count_traj_point):
                point_xy = obs_traj.split(")")[i6].split("(")[1]
                point_x = float(point_xy.split(",")[0])
                point_y = float(point_xy.split(",")[1])
                point_x_list.append(point_x)
                point_y_list.append(point_y)

            point_x_list_list.append(point_x_list)
            point_y_list_list.append(point_y_list)

    if(has_section_msg):
        section_info = info.split("{section:")[1].split("section_end}")[0]
        count_newlane = section_info.count("[")
        i8 = 0
        for i8 in range(count_newlane):
            newlane = section_info.split("]")[i8]
            point_begin = newlane.split(")")[0].split("(")[1]
            point_begin_x = float(point_begin.split(",")[0])
            point_begin_y = float(point_begin.split(",")[1])
            point_end = newlane.split(")")[1].split("(")[1]
            point_end_x = float(point_end.split(",")[0])
            point_end_y = float(point_end.split(",")[1])
            type = newlane.split(",")[4]
            color = newlane.split(",")[5]
            stop_x_list = []
            stop_y_list = []
            if(newlane.count(")") == 4):
                stop_begin = newlane.split(")")[2].split("(")[1]
                stop_begin_x = float(stop_begin.split(",")[0])
                stop_begin_y = float(stop_begin.split(",")[1])
                stop_end = newlane.split(")")[3].split("(")[1]
                stop_end_x = float(stop_end.split(",")[0])
                stop_end_y = float(stop_end.split(",")[1])
                stop_x_list.append(stop_begin_x)
                stop_x_list.append(stop_end_x)
                stop_y_list.append(stop_begin_y)
                stop_y_list.append(stop_end_y)
                stop_x_list_list.append(stop_x_list)
                stop_y_list_list.append(stop_y_list)

            solidlane_x_list = []
            solidlane_y_list = []

            brokenlane_x_list = []
            brokenlane_y_list = []

            if (type == "solid"):
                solidlane_x_list.append(point_begin_x)
                solidlane_x_list.append(point_end_x)
                solidlane_y_list.append(point_begin_y)
                solidlane_y_list.append(point_end_y)
                solidlane_x_list_list.append(solidlane_x_list)
                solidlane_y_list_list.append(solidlane_y_list)

            if (type == "broken"):
                brokenlane_x_list.append(point_begin_x)
                brokenlane_x_list.append(point_end_x)
                brokenlane_y_list.append(point_begin_y)
                brokenlane_y_list.append(point_end_y)
                brokenlane_x_list_list.append(brokenlane_x_list)
                brokenlane_y_list_list.append(brokenlane_y_list)


def data_gen(t=0.0):
    while True:

        yield t, lane_x_list_list, lane_y_list_list, obs_x_list_list, obs_y_list_list, point_x_list_list, \
            point_y_list_list, solidlane_x_list_list, solidlane_y_list_list, brokenlane_x_list_list, \
            brokenlane_y_list_list, stop_x_list_list, stop_y_list_list


def init():
    ax_xy.set_ylim(0, 80)
    ax_xy.set_xlim(-20, 20)
    ax_xy.set_title("center_lane")
    ax_xy.set_ylabel("m")
    ax_xy.set_xlabel("m")

    del xdata_lane[:]
    del ydata_lane[:]
    del xdata_obs[:]
    del ydata_obs[:]

    # line_lane.set_data(xdata_lane, ydata_lane)
    # line_obs.set_data(xdata_obs, ydata_obs)

    # return line_lane, line_obs
    return


def run(data):
    time, lane_xs, lane_ys, obs_xs, obs_ys, traj_xs, traj_ys, \
        solidlane_xs, solidlane_ys, brokenlane_xs, brokenlane_ys, \
        stopline_xs, stopline_ys = data

    ax_xy.clear()
    ax_xy.set_ylim(0, 80)
    ax_xy.set_xlim(-20, 20)
    ax_xy.set_title("center_lane")
    ax_xy.set_ylabel("m")
    ax_xy.set_xlabel("m")

    xdata_lane = lane_xs
    ydata_lane = lane_ys

    # print("lanexx=")
    # print(len(xdata_lane))
    # print("laneyy=")
    # print(len(ydata_lane))
    # for i in range(len(xdata_lane)):
    #     print(xdata_lane[i])
    # i4 = 0
    # for i4 in range(len(xdata_lane)):
    #     # print("lanex=")
    #     # print(len(xdata_lane[i4]))
    #     # print("laney=")
    #     # print(len(ydata_lane[i4]))
    #     # print(i4)
    #     try:
    #         line_lane, = ax_xy.plot([], [], lw=2, color='black')
    #         line_lane.set_data(xdata_lane[i4], ydata_lane[i4])
    #     except:
    #         print("errori4")

    xdata_obs = obs_xs
    ydata_obs = obs_ys

    i5 = 0

    for i5 in range(len(obs_xs)):

        try:
            line_obs, = ax_xy.plot([], [], lw=2, color='blue')
            line_obs.set_data(xdata_obs[i5], ydata_obs[i5])

        except:
            print("errori5")

    xdata_traj = traj_xs
    ydata_traj = traj_ys

    i7 = 0
    for i7 in range(len(traj_xs)):
        try:
            line_traj, = ax_xy.plot([], [], lw=3, color='green')
            line_traj.set_data(xdata_traj[i7], ydata_traj[i7])
        except:
            print("errori7")

    line_echo, = ax_xy.plot([], [], lw=1, color='blue')
    line_echo.set_data(xdata_echo, ydata_echo)

    xdata_solidlane = solidlane_xs
    ydata_solidlane = solidlane_ys
    xdata_brokenlane = brokenlane_xs
    ydata_brokenlane = brokenlane_ys
    i9 = 0
    for i9 in range(len(xdata_solidlane)):

        try:
            line_lane, = ax_xy.plot([], [], lw=2, color='black')
            line_lane.set_data(xdata_solidlane[i9], ydata_solidlane[i9])
        except:
            print("errori9")

    i10 = 0
    for i10 in range(len(xdata_brokenlane)):

        try:
            line_lane, = ax_xy.plot([], [], lw=2, color='black', linestyle='-')
            line_lane.set_data(xdata_brokenlane[i10], ydata_brokenlane[i10])
            line_lane.set_dashes((4, 2))
        except:
            print("errori10")

    xdata_stopline = stopline_xs
    ydata_stopline = stopline_ys

    i11 = 0
    for i11 in range(len(xdata_stopline)):

        try:
            line_lane, = ax_xy.plot([], [], lw=2, color='red')
            line_lane.set_data(xdata_stopline[i11], ydata_stopline[i11])
        except:
            print("errori11")

    return


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/planning/prediction/debuginfo",
                     DebugInfo, callback_debuginfo)


if __name__ == '__main__':
    try:
        listener()
        ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=10,
                                      repeat=False, init_func=init)
        plt.show()
    except rospy.ROSInterruptException:
        pass
