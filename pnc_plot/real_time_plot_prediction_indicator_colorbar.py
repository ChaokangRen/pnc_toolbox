import rospy
from sg_debug_msgs.msg import DebugInfo
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import math

count_indi = 0

demo_point_x = []
demo_point_y = []

indicator0_x = []
indicator0_y = []
indicator0 = []


indicator1_x = []
indicator1_y = []
indicator1 = []

lane_x_list_list = []
lane_y_list_list = []

obs_x_list_list = []
obs_y_list_list = []

point_x_list_list = []
point_y_list_list = []


xdata_lane, ydata_lane = [], []
xdata_obs, ydata_obs = [], []

xdata_echo = [1, -1, -1, 1, 1]
ydata_echo = [2.4, 2.4, -2.4, -2.4, 2.4]

t_now = 0
t_pre = 0

fig = plt.figure('real-time prediction plot')

ax_xy = fig.add_subplot(121, aspect=1)
ax_demo = fig.add_subplot(122, aspect=1)
fig.tight_layout()
plt.subplots_adjust(wspace=0, hspace=0)
ax_demo.grid()
ax_demo.set_ylim(1500, 3000)
ax_demo.set_xlim(2000, 4500)
ax_demo.set_title("demo")
ax_demo.set_ylabel("m")
ax_demo.set_xlabel("m")

init_x = [0]
init_y = [0]
init_ind = [0]


with open("qidi_city_demo_fd.json", "r") as f:
    for line in f.readlines():
        point_data = line.strip('\n')
        if(point_data.count(",") != 0):
            demo_x = float(point_data.split("[")[1].split(",")[0])
            demo_y = float(point_data.split(",")[1].split(",")[0])
            demo_point_x.append(demo_x)
            demo_point_y.append(demo_y)
line_demo, = ax_demo.plot([], [], lw=1, color='black')
line_demo.set_data(demo_point_x, demo_point_y)


def callback_debuginfo(debuginfo_msg):
    global count_indi
    count_indi = count_indi + 1
    lane_x_list_list.clear()
    lane_y_list_list.clear()
    obs_x_list_list.clear()
    obs_y_list_list.clear()
    point_x_list_list.clear()
    point_y_list_list.clear()

    i1 = 0

    info = debuginfo_msg.string_data

    lane_info = info.split("Lane:")[1].split("Obstacle:")[0]
    obs_info = info.split("Obstacle:")[1].split("Indicator:")[0]
    indicator_info = info.split("Indicator:")[1]
    ego_info = indicator_info.split("Mean_Error:")[0]
    error_info = indicator_info.split("Mean_Error:")[1]

    count_lane = lane_info.count("[")

    while(i1 < count_lane):

        lane = lane_info.split("]")[i1]
        i2 = 0
        count_lane_point = lane.count(")")

        lane_x_list = []
        lane_y_list = []
        while(i2 < count_lane_point):

            lane_point = lane.split(")")[i2]
            lane_x = float(lane_point.split(",")[0].split("(")[1])
            lane_y = float(lane_point.split(",")[1])
            lane_x_list.append(lane_x)
            lane_y_list.append(lane_y)
            i2 = i2 + 1

        lane_x_list_list.append(lane_x_list)
        lane_y_list_list.append(lane_y_list)

        i1 = i1 + 1

    count_obs = obs_info.count("State:")

    i3 = 1

    while (i3 < count_obs+1):
        obs_x_list = []
        obs_y_list = []

        obs = obs_info.split("State:")[i3]

        obs_state = obs.split("Trajectory:")[0]
        obs_traj = obs.split("Trajectory:")[1]

        obs_x = float(obs_state.split("[")[1].split(",")[0])
        obs_y = float(obs_state.split("[")[1].split(",")[1])
        obs_yaw = float(obs_state.split("[")[1].split(",")[2])
        obs_width = float(obs_state.split("[")[1].split(",")[3])
        obs_length = float(obs_state.split("[")[1].split(",")[4].split("]")[0])

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

        i3 = i3 + 1

    count_indicator = error_info.count(")")
    if(count_indi % 10 == 0):
        i8 = 0
        for i8 in range(count_indicator):
            error_msg = error_info.split(")")[i8].split("(")[1]
            obs_his_x = float(error_msg.split(",")[0])
            obs_his_y = float(error_msg.split(",")[1])
            obs_error = float(error_msg.split(",")[2])
            predictor = float(error_msg.split(",")[3])
            if (predictor == 0):
                indicator0_x.append(obs_his_x)
                indicator0_y.append(obs_his_y)
                indicator0.append(obs_error)
            if (predictor == 1):
                indicator1_x.append(obs_his_x)
                indicator1_y.append(obs_his_y)
                indicator1.append(obs_error)


def data_gen(t=0.0):
    while True:

        yield t, lane_x_list_list, lane_y_list_list, obs_x_list_list, obs_y_list_list, point_x_list_list, point_y_list_list, indicator0_x, indicator0_y, indicator0, indicator1_x, indicator1_y, indicator1


def init():
    ax_xy.grid()
    ax_xy.set_ylim(0, 80)
    ax_xy.set_xlim(-10, 10)
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
    time, lane_xs, lane_ys, obs_xs, obs_ys, traj_xs, traj_ys, ind_x0, ind_y0, ind0, ind_x1, ind_y1, ind1 = data
    line_ind0 = ax_demo.scatter(
        init_x, init_y, c=init_ind, cmap='afmhot_r')
    cb0 = fig.colorbar(line_ind0)
    cb0.remove()
    ax_xy.clear()
    ax_xy.grid()
    ax_xy.set_ylim(0, 80)
    ax_xy.set_xlim(-20, 20)
    ax_xy.set_title("center_lane")
    ax_xy.set_ylabel("m")
    ax_xy.set_xlabel("m")

    # ax_demo.clear()
    # ax_demo.grid()
    # ax_demo.set_ylim(1500, 3000)
    # ax_demo.set_xlim(2000, 4500)
    # ax_demo.set_title("demo")
    # ax_demo.set_ylabel("m")
    # ax_demo.set_xlabel("m")

    xdata_lane = lane_xs
    ydata_lane = lane_ys

    # print("lanexx=")
    # print(len(xdata_lane))
    # print("laneyy=")
    # print(len(ydata_lane))
    # for i in range(len(xdata_lane)):
    #     print(xdata_lane[i])
    i4 = 0
    for i4 in range(len(xdata_lane)):
        # print("lanex=")
        # print(len(xdata_lane[i4]))
        # print("laney=")
        # print(len(ydata_lane[i4]))
        # print(i4)
        try:
            line_lane, = ax_xy.plot([], [], lw=2, color='black')
            line_lane.set_data(xdata_lane[i4], ydata_lane[i4])
        except:
            print("errori4")

    xdata_obs = obs_xs
    ydata_obs = obs_ys
    i5 = 0

    for i5 in range(len(obs_xs)):

        try:
            line_obs, = ax_xy.plot([], [], lw=2, color='red')
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
    try:
        line_ind0 = ax_demo.scatter(ind_x0, ind_y0, s=20,  marker='^',
                                    c=ind0, cmap='afmhot_r')
    except:
        pass
    try:
        ax_demo.scatter(ind_x1, ind_y1, s=20, marker='x',
                        c=ind1, cmap='afmhot_r')
    except:
        pass

    # plt.scatter(x, y, c=colors, cmap='afmhot_r')
    # plt.colorbar()

    line_echo, = ax_xy.plot([], [], lw=1, color='blue')
    line_echo.set_data(xdata_echo, ydata_echo)

    cb0 = fig.colorbar(line_ind0, extend='max')
    # cb1 = fig.colorbar(line_ind1)

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
