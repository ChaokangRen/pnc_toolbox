import rospy
from sg_debug_msgs.msg import DebugInfo
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

model_default_lists = []
model_default_listl = []

model_act_lists = []
model_act_listl = []

model_traj_list1s = []
model_traj_list1l = []

model_traj_list2s = []
model_traj_list2l = []

model_traj_list3l = []
model_traj_list3s = []

model_traj_list4l = []
model_traj_list4s = []

model_traj_list5l = []
model_traj_list5s = []

model_traj_list6l = []
model_traj_list6s = []

model_default_score_list = []
model_traj1_score_list = []
model_traj2_score_list = []
model_traj3_score_list = []
model_traj4_score_list = []

rtk_traj_listl = []
rtk_traj_lists = []

current_score_list = []
left_score_list = []
right_score_list = []

turn_right_list = []
turn_left_list = []

model_dir_list =[]

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

fig = plt.figure('real-time model traj plot')

ax_xy = fig.add_subplot(111, aspect=1)


def callback_planning_debuginfo(debuginfo_msg):
    model_default_lists.clear()
    model_default_listl.clear()

    model_act_lists.clear()
    model_act_listl.clear()

    model_traj_list1s.clear()
    model_traj_list1l.clear()

    model_traj_list2s.clear()
    model_traj_list2l.clear()

    model_traj_list3l.clear()
    model_traj_list3s.clear()

    model_traj_list4l.clear()
    model_traj_list4s.clear()

    model_traj_list5l.clear()
    model_traj_list5s.clear()

    model_traj_list6l.clear()
    model_traj_list6s.clear()

    model_default_score_list.clear()
    model_traj1_score_list.clear()
    model_traj2_score_list.clear()
    model_traj3_score_list.clear()
    model_traj4_score_list.clear()

    rtk_traj_listl.clear()
    rtk_traj_lists.clear()

    current_score_list.clear()
    left_score_list.clear()
    right_score_list.clear()

    turn_right_list.clear()
    turn_left_list.clear()

    model_dir_list.clear()

    info = debuginfo_msg.string_data

    has_model_default_msg = info.count("{model_default_traj:")
    has_model_act_msg = info.count("{model_act_traj:")
    has_model_traj1_msg = info.count("{model_traj1:")
    has_model_traj2_msg = info.count("{model_traj2:")
    has_model_traj3_msg = info.count("{model_traj3:")
    has_model_traj4_msg = info.count("{model_traj4:")
    has_model_traj5_msg = info.count("{model_traj5:")
    has_model_traj6_msg = info.count("{model_traj6:")

    has_rtk_traj_msg = info.count("{rtk_traj:")

    if(info.count("{model_default_score:")):
        model_default_score_list.append(info.split("{model_default_score:")[1].split(
            "model_default_score_end}")[0].split(")")[0].split("(")[1])
        # print(model_default_score)
    if(info.count("{model_score1:")):
        model_traj1_score_list.append(info.split("{model_score1:")[1].split(
            "model_score1_end}")[0].split(")")[0].split("(")[1])
        # print(model_default_score)
    if(info.count("{model_score2:")):
        model_traj2_score_list.append(info.split("{model_score2:")[1].split(
            "model_score2_end}")[0].split(")")[0].split("(")[1])
        # print(model_default_score)
    if(info.count("{model_score3:")):
        model_traj3_score_list.append(info.split("{model_score3:")[1].split(
            "model_score3_end}")[0].split(")")[0].split("(")[1])
        # print(model_default_score)
    if(info.count("{model_score4:")):
        model_traj4_score_list.append(info.split("{model_score4:")[1].split(
            "model_score1_end}")[0].split(")")[0].split("(")[1])
        # print(model_default_score)
    
    if(info.count("{current_score:")):
        current_score_list.append(info.split("{current_score:")[1].split(
            "current_score_end}")[0].split(")")[0].split("(")[1])
    if(info.count("{left_score:")):
        left_score_list.append(info.split("{left_score:")[1].split(
            "left_score_end}")[0].split(")")[0].split("(")[1])
    if(info.count("{right_score:")):
        right_score_list.append(info.split("{right_score:")[1].split(
            "right_score_end}")[0].split(")")[0].split("(")[1])
    
    if(info.count("{turn_right_score:")):
        turn_right_list.append(info.split("{turn_right_score:")[1].split(
            "turn_right_score_end}")[0].split(")")[0].split("(")[1])

    if(info.count("{turn_left_score:")):
        turn_left_list.append(info.split("{turn_left_score:")[1].split(
            "turn_left_score_end}")[0].split(")")[0].split("(")[1])

    if(info.count("{model_dir:")):
        model_dir_list.append(info.split("{model_dir:")[1].split(
            "model_dir_end}")[0].split(")")[0].split("(")[1])



    if(has_model_default_msg):
        model_default_msg = info.split("{model_default_traj:")[
            1].split("model_default_traj_end}")[0]
        for k0 in range(model_default_msg.count(")")):
            sl_point = model_default_msg.split(")")[k0].split("(")[1]
            point_sls = float(sl_point.split(",")[0])
            point_sll = float(sl_point.split(",")[1])
            model_default_lists.append(point_sls)
            model_default_listl.append(point_sll)

    if(has_model_act_msg):
        model_act_msg = info.split(
            "{model_act_traj:")[1].split("model_act_traj_end}")[0]
        for k1 in range(model_act_msg.count(")")):
            sl_point = model_act_msg.split(")")[k1].split("(")[1]
            point_sls = float(sl_point.split(",")[0])
            point_sll = float(sl_point.split(",")[1])
            model_act_lists.append(point_sls)
            model_act_listl.append(point_sll)

    if(has_model_traj1_msg):
        model_traj1_msg = info.split(
            "{model_traj1:")[1].split("model_traj1_end}")[0]
        for k2 in range(model_traj1_msg.count(")")):
            sl_point = model_traj1_msg.split(")")[k2].split("(")[1]
            point_sls = float(sl_point.split(",")[0])
            point_sll = float(sl_point.split(",")[1])
            model_traj_list1s.append(point_sls)
            model_traj_list1l.append(point_sll)

    if(has_model_traj2_msg):
        model_traj2_msg = info.split(
            "{model_traj2:")[1].split("model_traj2_end}")[0]
        for k4 in range(model_traj2_msg.count(")")):
            sl_point = model_traj2_msg.split(")")[k4].split("(")[1]
            point_sls = float(sl_point.split(",")[0])
            point_sll = float(sl_point.split(",")[1])
            model_traj_list2s.append(point_sls)
            model_traj_list2l.append(point_sll)

    if(has_model_traj3_msg):
        model_traj3_msg = info.split(
            "{model_traj3:")[1].split("model_traj3_end}")[0]
        for k5 in range(model_traj3_msg.count(")")):
            sl_point = model_traj3_msg.split(")")[k5].split("(")[1]
            point_sls = float(sl_point.split(",")[0])
            point_sll = float(sl_point.split(",")[1])
            model_traj_list3s.append(point_sls)
            model_traj_list3l.append(point_sll)

    if(has_model_traj4_msg):
        model_traj4_msg = info.split(
            "{model_traj4:")[1].split("model_traj4_end}")[0]
        for k6 in range(model_traj4_msg.count(")")):
            sl_point = model_traj4_msg.split(")")[k6].split("(")[1]
            point_sls = float(sl_point.split(",")[0])
            point_sll = float(sl_point.split(",")[1])
            model_traj_list4s.append(point_sls)
            model_traj_list4l.append(point_sll)
    
    if(has_model_traj5_msg):
        model_traj5_msg = info.split(
            "{model_traj5:")[1].split("model_traj5_end}")[0]
        for k7 in range(model_traj5_msg.count(")")):
            sl_point = model_traj5_msg.split(")")[k7].split("(")[1]
            point_sls = float(sl_point.split(",")[0])
            point_sll = float(sl_point.split(",")[1])
            model_traj_list5s.append(point_sls)
            model_traj_list5l.append(point_sll)

    if(has_model_traj6_msg):
        model_traj6_msg = info.split(
            "{model_traj6:")[1].split("model_traj6_end}")[0]
        for k8 in range(model_traj6_msg.count(")")):
            sl_point = model_traj6_msg.split(")")[k8].split("(")[1]
            point_sls = float(sl_point.split(",")[0])
            point_sll = float(sl_point.split(",")[1])
            model_traj_list6s.append(point_sls)
            model_traj_list6l.append(point_sll)

    if(has_rtk_traj_msg):
        rtk_traj_msg = info.split("{rtk_traj:")[1].split("rtk_traj_end}")[0]
        for k9 in range(rtk_traj_msg.count(")")):
            sl_point = rtk_traj_msg.split(")")[k9].split("(")[1]
            point_sls = float(sl_point.split(",")[0])
            point_sll = float(sl_point.split(",")[1])
            rtk_traj_lists.append(point_sls)
            rtk_traj_listl.append(point_sll)



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
        # print(count_obs)

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
            brokenlane_y_list_list, stop_x_list_list, stop_y_list_list, model_default_lists, model_default_listl,\
            model_act_lists, model_act_listl, model_traj_list1s, model_traj_list1l, model_traj_list2s, model_traj_list2l,\
            model_traj_list3s, model_traj_list3l, model_traj_list4s, model_traj_list4l, model_default_score_list,\
            model_traj1_score_list, model_traj2_score_list, model_traj3_score_list, model_traj4_score_list,current_score_list,left_score_list,\
            right_score_list,model_dir_list,turn_right_list,turn_left_list,model_traj_list5s, model_traj_list5l,model_traj_list6s, model_traj_list6l,\
            rtk_traj_lists,rtk_traj_listl


def init():
    ax_xy.set_ylim(0, 60)
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
    time, lane_xs, lane_ys, obs_xs, obs_ys, traj_xs, traj_ys, \
        solidlane_xs, solidlane_ys, brokenlane_xs, brokenlane_ys, \
        stopline_xs, stopline_ys, model_defaults, model_defaultl,\
        model_acts, model_actl, model_traj1s, model_traj1l, model_traj2s, model_traj2l,\
        model_traj3s, model_traj3l, model_traj4s, model_traj4l, model_default_score,\
        model_score1, model_score2, model_score3, model_score4,current_score,left_score,\
        right_score,model_dir,turn_right,turn_left, model_traj5s, model_traj5l,model_traj6s, model_traj6l,rtk_s,rtk_l= data

    ax_xy.clear()
    ax_xy.set_ylim(0, 40)
    ax_xy.set_xlim(-10, 10)
    ax_xy.set_title("center_lane")
    ax_xy.set_ylabel("m")
    ax_xy.set_xlabel("m")

    xdata_lane = lane_xs
    ydata_lane = lane_ys

    # print(model_acts)
    # print(model_actl)

    # model_acts = [0, 10, 20]
    # model_actl = [-1, -1, -1]

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

    line1, = ax_xy.plot(model_defaultl, model_defaults, lw=3, color='blue')
    try:
        plt.text(-25, 40, 'default_score: ' +
                 str(model_default_score[0]), color='blue')
    except:
        print("error score")
    line2, = ax_xy.plot(model_actl, model_acts, lw=4, color='red')
    line3, = ax_xy.plot(model_traj1l, model_traj1s, lw=3, color='pink')
    try:
        plt.text(-25, 38, 'score1: ' +
                 str(model_score1[0]), color='pink')
    except:
        print("error score")
    line4, = ax_xy.plot(model_traj2l, model_traj2s, lw=3, color='purple')
    try:
        plt.text(-25, 36, 'score2: ' +
                 str(model_score2[0]), color='purple')
    except:
        print("error score")
    line5, = ax_xy.plot(model_traj3l, model_traj3s, lw=3, color='grey')
    try:
        plt.text(-25, 34, 'score3: ' +
                 str(model_score3[0]), color='grey')
    except:
        print("error score")
    line6, = ax_xy.plot(model_traj4l, model_traj4s, lw=3, color='green')

    #line7, = ax_xy.plot(model_traj5l, model_traj5s, lw=3, color='orange')
    #line8, = ax_xy.plot(model_traj6l, model_traj6s, lw=3, color='gold')
    line9, = ax_xy.plot(rtk_l, rtk_s, lw=3, color='gold')
    try:
        plt.text(-25, 32, 'score4: ' +
                 str(model_score4[0]), color='green')
    except:
        print("error score")

    try:
        plt.text(-25, 30, 'current_score: ' +
                 str(current_score[0]), color='black')
    except:
        print("error score")
    try:
        plt.text(-25, 28, 'leftt_score: ' +
                 str(left_score[0]), color='black')
    except:
        print("error score")
    try:
        plt.text(-25, 26, 'right_score: ' +
                 str(right_score[0]), color='black')
    except:
        print("error score")

    try:
        plt.text(15, 36, 'model_dir: ' +
                 str(model_dir[0]), color='black')
    except:
        print("error score")

    try:
        plt.text(15, 34, 'turn_left: ' +
                 str(turn_left[0]), color='black')
    except:
        print("error score1")
    try:
        plt.text(15, 32, 'turn_right: ' +
                 str(turn_right[0]), color='black')
    except:
        print("error score1")

    return


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/planning/prediction/debuginfo",
                     DebugInfo, callback_debuginfo)
    rospy.Subscriber("/planning/debuginfo",
                     DebugInfo, callback_planning_debuginfo)


if __name__ == '__main__':

    try:
        listener()
        ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=10,
                                      repeat=False, init_func=init)
        plt.show()
    except rospy.ROSInterruptException:
        pass
