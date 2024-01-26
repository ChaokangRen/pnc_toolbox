from turtle import width
import uuid
import rospy
import math
from sg_perception_msgs.msg import TrackedObjects
import matplotlib.pyplot as plt
import matplotlib.animation as animation

fig = plt.figure()
fig.suptitle('obstacle plot')

str_default = "default"
list_default = [[0, 1], [2, 3]]
uuid_map = {str_default: list_default}

t_now = 0
t_pre = 0
is_init = 0

plot_vel = fig.add_subplot(2, 3, 1)
line_vel, = plot_vel.plot([], [], lw=2)

plot_length = fig.add_subplot(2, 3, 2)
line_length = plot_length.plot([], [], lw=2)

plot_width = fig.add_subplot(2, 3, 3)
line_width = plot_length.plot([], [], lw=2)

plot_x = fig.add_subplot(2, 3, 4)
line_x = plot_x.plot([], [], lw=2)

plot_y = fig.add_subplot(2, 3, 5)
line_y = plot_y.plot([], [], lw=2)

plot_yaw = fig.add_subplot(2, 3, 6)
line_yaw = plot_yaw.plot([], [], lw=2)


t_actual = 0


# color
color_list = ['#FFEBCD', '#A52A2A', '#6495ED', '#B8860B', '#556B2F', '#6495ED', '#40E0D0',
              '#EE82EE', '#F5DEB3', '#FFFFFF', '#F5F5F5', '#9ACD32']
cnames = {
    'aqua':                 '#00FFFF',
    'aquamarine':           '#7FFFD4',
    'azure':                '#F0FFFF',
    'bisque':               '#FFE4C4',
    'black':                '#000000',
    'blanchedalmond':       '#FFEBCD',
    'blue':                 '#0000FF',
    'blueviolet':           '#8A2BE2',
    'brown':                '#A52A2A',
    'burlywood':            '#DEB887',
    'cadetblue':            '#5F9EA0',
    'chartreuse':           '#7FFF00',
    'chocolate':            '#D2691E',
    'coral':                '#FF7F50',
    'cornflowerblue':       '#6495ED',
    'cornsilk':             '#FFF8DC',
    'crimson':              '#DC143C',
    'cyan':                 '#00FFFF',
    'darkblue':             '#00008B',
    'darkcyan':             '#008B8B',
    'darkgoldenrod':        '#B8860B',
    'darkgray':             '#A9A9A9',
    'darkgreen':            '#006400',
    'darkkhaki':            '#BDB76B',
    'darkmagenta':          '#8B008B',
    'darkolivegreen':       '#556B2F',
    'darkorange':           '#FF8C00',
    'darkorchid':           '#9932CC',
    'darkred':              '#8B0000',
    'darksalmon':           '#E9967A',
    'darkseagreen':         '#8FBC8F',
    'darkslateblue':        '#483D8B',
    'darkslategray':        '#2F4F4F',
    'darkturquoise':        '#00CED1',
    'darkviolet':           '#9400D3',
    'deeppink':             '#FF1493',
    'deepskyblue':          '#00BFFF',
    'dimgray':              '#696969',
    'dodgerblue':           '#1E90FF',
    'firebrick':            '#B22222',
    'floralwhite':          '#FFFAF0',
    'forestgreen':          '#228B22',
    'fuchsia':              '#FF00FF',
    'gainsboro':            '#DCDCDC',
    'ghostwhite':           '#F8F8FF',
    'gold':                 '#FFD700',
    'goldenrod':            '#DAA520',
    'gray':                 '#808080',
    'green':                '#008000',
    'greenyellow':          '#ADFF2F',
    'honeydew':             '#F0FFF0',
    'hotpink':              '#FF69B4',
    'indianred':            '#CD5C5C',
    'indigo':               '#4B0082',
    'ivory':                '#FFFFF0',
    'khaki':                '#F0E68C',
    'lavender':             '#E6E6FA',
    'lavenderblush':        '#FFF0F5',
    'lawngreen':            '#7CFC00',
    'lemonchiffon':         '#FFFACD',
    'lightblue':            '#ADD8E6',
    'lightcoral':           '#F08080',
    'lightcyan':            '#E0FFFF',
    'lightgoldenrodyellow': '#FAFAD2',
    'lightgreen':           '#90EE90',
    'lightgray':            '#D3D3D3',
    'lightpink':            '#FFB6C1',
    'lightsalmon':          '#FFA07A',
    'lightseagreen':        '#20B2AA',
    'lightskyblue':         '#87CEFA',
    'lightslategray':       '#778899',
    'lightsteelblue':       '#B0C4DE',
    'lightyellow':          '#FFFFE0',
    'lime':                 '#00FF00',
    'limegreen':            '#32CD32',
    'linen':                '#FAF0E6',
    'magenta':              '#FF00FF',
    'maroon':               '#800000',
    'mediumaquamarine':     '#66CDAA',
    'mediumblue':           '#0000CD',
    'mediumorchid':         '#BA55D3',
    'mediumpurple':         '#9370DB',
    'mediumseagreen':       '#3CB371',
    'mediumslateblue':      '#7B68EE',
    'mediumspringgreen':    '#00FA9A',
    'mediumturquoise':      '#48D1CC',
    'mediumvioletred':      '#C71585',
    'midnightblue':         '#191970',
    'mintcream':            '#F5FFFA',
    'mistyrose':            '#FFE4E1',
    'moccasin':             '#FFE4B5',
    'navajowhite':          '#FFDEAD',
    'navy':                 '#000080',
    'oldlace':              '#FDF5E6',
    'olive':                '#808000',
    'olivedrab':            '#6B8E23',
    'orange':               '#FFA500',
    'orangered':            '#FF4500',
    'orchid':               '#DA70D6',
    'palegoldenrod':        '#EEE8AA',
    'palegreen':            '#98FB98',
    'paleturquoise':        '#AFEEEE',
    'palevioletred':        '#DB7093',
    'papayawhip':           '#FFEFD5',
    'peachpuff':            '#FFDAB9',
    'peru':                 '#CD853F',
    'pink':                 '#FFC0CB',
    'plum':                 '#DDA0DD',
    'powderblue':           '#B0E0E6',
    'purple':               '#800080',
    'red':                  '#FF0000',
    'rosybrown':            '#BC8F8F',
    'royalblue':            '#4169E1',
    'saddlebrown':          '#8B4513',
    'salmon':               '#FA8072',
    'sandybrown':           '#FAA460',
    'seagreen':             '#2E8B57',
    'seashell':             '#FFF5EE',
    'sienna':               '#A0522D',
    'silver':               '#C0C0C0',
    'skyblue':              '#87CEEB',
    'slateblue':            '#6A5ACD',
    'slategray':            '#708090',
    'snow':                 '#FFFAFA',
    'springgreen':          '#00FF7F',
    'steelblue':            '#4682B4',
    'tan':                  '#D2B48C',
    'teal':                 '#008080',
    'thistle':              '#D8BFD8',
    'tomato':               '#FF6347',
    'turquoise':            '#40E0D0',
    'violet':               '#EE82EE',
    'wheat':                '#F5DEB3',
    'white':                '#FFFFFF',
    'whitesmoke':           '#F5F5F5',
    'yellow':               '#FFFF00',
    'yellowgreen':          '#9ACD32'}

# color


def callback_tracked(tracked_objects):
    size = len(tracked_objects.vehicles)
    uuid_list = {}

    global t_now
    t_now = rospy.Time.now().to_sec()
    global t_pre
    global is_init
    global t_actual

    # print(t_actual)

    if (is_init == 0):
        t_pre = t_now
        is_init = 1
    for i in range(size):
        uuid = tracked_objects.vehicles[i].tracker.uuid
        center_x = tracked_objects.vehicles[i].static_prop.center_point_m.x
        center_y = tracked_objects.vehicles[i].static_prop.center_point_m.y
        center_z = tracked_objects.vehicles[i].static_prop.center_point_m.z

        length = tracked_objects.vehicles[i].static_prop.dimension_m.x
        weight = tracked_objects.vehicles[i].static_prop.dimension_m.y
        height = tracked_objects.vehicles[i].static_prop.dimension_m.z

        rotation_x = tracked_objects.vehicles[i].static_prop.rotation_deg.x
        rotation_y = tracked_objects.vehicles[i].static_prop.rotation_deg.y
        rotation_z = tracked_objects.vehicles[i].static_prop.rotation_deg.z

        vel_x = tracked_objects.vehicles[i].movement_prop.linear_velocity_mps.x
        vel_y = tracked_objects.vehicles[i].movement_prop.linear_velocity_mps.y
        vel_z = tracked_objects.vehicles[i].movement_prop.linear_velocity_mps.z

        obs_data = [center_x, center_y, center_z, length, weight, height,
                    rotation_x, rotation_y, rotation_z, vel_x, vel_y, vel_z, uuid, t_actual]

        if(uuid in uuid_map):
            uuid_map[uuid].append(obs_data)
        else:
            uuid_map[uuid] = [obs_data]

        uuid_list[uuid] = i

    uuid_key = uuid_map.keys()
    del_id_list = []
    for id in uuid_key:
        if((id in uuid_list) == False):
            del_id_list.append(id)
    for del_id in del_id_list:
        del uuid_map[del_id]
        # print(del_id)


def data_gen(t=0.0):
    global t_now
    global t_pre
    global t_actual
    while True:
        t += (t_now - t_pre)
        t_pre = t_now
        t_actual = t
        yield t


def init():
    plot_vel.grid()
    plot_vel.set_ylim(0, 10)
    plot_vel.set_xlim(0, 10)
    plot_vel.set_title("vel")
    plot_vel.set_ylabel("m/s")
    plot_vel.set_xlabel("s")
    plot_vel.legend(loc='upper right')

    plot_length.grid()
    plot_length.set_ylim(0, 10)
    plot_length.set_xlim(0, 10)
    plot_length.set_title("length")
    plot_length.set_ylabel("m")
    plot_length.set_xlabel("s")

    plot_width.grid()
    plot_width.set_ylim(0, 10)
    plot_width.set_xlim(0, 10)
    plot_width.set_title("width")
    plot_width.set_ylabel("m")
    plot_width.set_xlabel("s")

    plot_x.grid()
    plot_x.set_ylim(-10, 10)
    plot_x.set_xlim(0, 10)
    plot_x.set_title("x")
    plot_x.set_ylabel("m")
    plot_x.set_xlabel("s")

    plot_y.grid()
    plot_y.set_ylim(-50, 50)
    plot_y.set_xlim(0, 10)
    plot_y.set_title("y")
    plot_y.set_ylabel("m")
    plot_y.set_xlabel("s")

    plot_yaw.grid()
    plot_yaw.set_ylim(-180, 180)
    plot_yaw.set_xlim(0, 10)
    plot_yaw.set_title("yaw")
    plot_yaw.set_ylabel("deg")
    plot_yaw.set_xlabel("s")

    return line_vel, line_length, line_width, line_x, line_y, line_yaw


def run(data):
    t = data
    vel_data_list = []
    length_data_list = []
    width_data_list = []
    x_data_list = []
    y_data_list = []
    yaw_data_list = []
    t_data_list = []

    vel_data = []
    length_data = []
    width_data = []
    x_data = []
    y_data = []
    yaw_data = []
    t_data = []

    uuid_key_list = []

    for obs_id in uuid_map:
        size = len(uuid_map[obs_id])
        del vel_data[:]
        del length_data[:]
        del width_data[:]
        del x_data[:]
        del y_data[:]
        del yaw_data[:]
        del t_data[:]

        for i in range(size):
            speed = uuid_map[obs_id][i][9] * uuid_map[obs_id][i][9] + \
                uuid_map[obs_id][i][10] * uuid_map[obs_id][i][10]
            speed = math.sqrt(speed)
            vel_data.append(speed)

            length_data.append(uuid_map[obs_id][i][3])
            width_data.append(uuid_map[obs_id][i][4])
            x_data.append(uuid_map[obs_id][i][0])
            y_data.append(uuid_map[obs_id][i][1])
            yaw_data.append(uuid_map[obs_id][i][8])
            t_data.append(uuid_map[obs_id][i][-1])

        vel_data_list.append(vel_data)
        length_data_list.append(length_data)
        width_data_list.append(width_data)
        x_data_list.append(x_data)
        y_data_list.append(y_data)
        yaw_data_list.append(yaw_data)
        t_data_list.append(t_data)

        uuid_count = len(uuid_map[obs_id][0]) - 2
        uuid_key_list.append(uuid_map[obs_id][0][uuid_count])

    t_newest = t_data[-1]
    plot_vel.set_xlim(t_newest - 4, t_newest + 1)
    plot_length.set_xlim(t_newest - 4, t_newest + 1)
    plot_width.set_xlim(t_newest - 4, t_newest + 1)
    plot_x.set_xlim(t_newest - 4, t_newest + 1)
    plot_y.set_xlim(t_newest - 4, t_newest + 1)
    plot_yaw.set_xlim(t_newest - 4, t_newest + 1)
    # line_vel.set_data(t_data,vel_data)

    vel_size = len(vel_data_list)
    uuid_size = len(uuid_key_list)

    # print("count")
    # print(vel_size, uuid_size)

    for i in range(vel_size):
        line_vel, = plot_vel.plot(
            [], [], lw=2, color=color_list[i], label='')
        line_vel.set_data(t_data_list[i], vel_data_list[i])

        line_length, = plot_length.plot([], [], lw=2, color=color_list[i])
        line_length.set_data(t_data_list[i], length_data_list[i])

        line_width, = plot_width.plot([], [], lw=2, color=color_list[i])
        line_width.set_data(t_data_list[i], width_data_list[i])

        line_x, = plot_x.plot([], [], lw=2, color=color_list[i])
        line_x.set_data(t_data_list[i], x_data_list[i])

        line_y, = plot_y.plot([], [], lw=2, color=color_list[i])
        line_y.set_data(t_data_list[i], y_data_list[i])

        line_yaw, = plot_yaw.plot([], [], lw=2, color=color_list[i])
        line_yaw.set_data(t_data_list[i], yaw_data_list[i])

    return line_vel, line_length, line_width, line_x, line_y, line_yaw


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/perception/lidar/tracked",
                     TrackedObjects, callback_tracked)


if __name__ == '__main__':
    try:
        listener()
        ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=10,
                                      repeat=False, init_func=init)
        plt.show()
    except rospy.ROSInterruptException:
        pass
