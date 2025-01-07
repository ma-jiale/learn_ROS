from __future__ import print_function

import rospy
from jaka_msgs.srv import Move, MoveRequest
from std_msgs.msg import Int32, Int32MultiArray
import threading



# 参数定义
has_ref = True
ref_joint = [0]
mvvelo = 60
mvacc = 20
mvtime = 0.0
mvradii = 0.0
coord_mode = 0
index = 0

start_pose = [3.0248514363041132, -0.044787409304253775, 1.5677342719046798, 0.06558799612958398, 1.6588882873168558, 1.5498372747823412]
center_pre_pose = [1.693847469702398, 0.44569096190045343, 0.7149607986917752, -0.12910772108867472, 1.933362989486607, 0.2573026700094212]
end_pre_pose = [0.7280627559768637, 0.8422369785836348, 0.2918457554745006, -0.0055024257920404576, 1.994230867595159, -0.6631577052623735]
end_pose = [0.7296087209849367, 0.8544129510503188, 0.6498624892045425, 0.01839411859631408, 1.6663424751852391, -0.6631457210375047]

target_pre_position = {1:[1.9840814275746, -0.015174389653489448, 1.352293861438557, 0.002275336147800112, 1.7645891506595273, 0.5996919745105693],
                       2:[1.884648313838302, 0.4776648738503682, 0.7758766136998023, 0.016488626842177487, 1.8504201691697555, 0.5410891149022173],
                       3:[1.881820036769269, 0.8262500226086773, 0.2547545795056153, -0.031472241082694564, 1.8441644037882503, 0.4595724173447512],
                       4:[1.5598039145450533, -0.052121754923949354, 1.435799940324241, 0.02897618915545002, 1.727941391010787, 0.13821542748831842],
                       5:[1.5900041612143883, 0.35384386250609573, 1.0061415103286107, 0.04001366025959973, 1.7098212430091861, 0.17956100328562188],
                       6:[1.5749639590040645, 0.840834824273987, 0.17666537026062126, 0.040277313206712975, 1.9043851337538882, 0.17555827217944817],
                       7:[1.1828041686228583, 0.0011960615172689772, 1.404077697096571, -0.00574211028941613, 1.6667499388307776, -0.20870391401322985],
                       8:[1.3679005217212212, 0.4796183025039799, 0.7863388420102504, -0.02606735566687315, 1.8240428902335628, -0.09577656307468177],
                       9:[1.382868818582332, 0.7588267734969006, 0.4022684034154729, 0.022504707726306864, 1.7555290766587268, -0.0921693113891779]
                       }

target_position = {1:[1.95826740720724, 0.04173869424836396, 1.8117330902331144, 0.03969008618814258, 1.2725408459970093, 0.5618577765998194],
                   2:[1.9002997115169336, 0.4532769762423935, 1.2734855987014357, 0.0022993045975376798, 1.377402813598866, 0.5336708797084403],
                   3:[1.8421043155541201, 1.064316649627064, 0.2602433544955182, 0.000238017920106897, 1.782553503737834, 0.4196769327565705],
                   4:[1.662964122215543, -0.0558128961835347, 1.8938849517086263, -0.0675088052631269, 1.286071035873866, 0.30213565524354075],
                   5:[1.5871638999204865, 0.3566721395751287, 1.4239355577041453, 0.009274123471169746, 1.2971804123272284, 0.17456358151533913],
                   6:[1.6178914524840475, 0.948225463323157, 0.47138142823374807, -0.006053700136004504, 1.6655635005687681, 0.2082272691717523],
                   7:[1.139061747851798, -0.012693655105651238, 1.849183792948063, -0.0016075527096857806, 1.2296013682921574, -0.2477245501859893],
                   8:[1.3484501247591854, 0.42658810745961234, 1.3203319337135109, -0.055344817021311526, 1.32140053078704, -0.06219676499235007],
                   9:[1.421194369712702, 0.8642639838924588, 0.6111534428783714, -0.04973619978272079, 1.5297103274562371, -0.031960565648409]
                   }

target_list = []


def move_start():
    move_request = MoveRequest()
    move_request.pose = start_pose
    move_request.has_ref = has_ref
    move_request.ref_joint = ref_joint
    move_request.mvvelo = mvvelo
    move_request.mvacc = mvacc
    move_request.mvtime = mvtime
    move_request.mvradii = mvradii
    move_request.coord_mode = coord_mode
    move_request.index = index

    rospy.wait_for_service('/jaka_driver/joint_move')  # 等待服务启动
    try:
        # 创建服务代理
        joint_move = rospy.ServiceProxy('/jaka_driver/joint_move', Move)
        joint_move(move_request)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    

 


def move_center_pre(joint_move):
    move_request = MoveRequest()
    move_request.pose = center_pre_pose
    move_request.has_ref = has_ref
    move_request.ref_joint = ref_joint
    move_request.mvvelo = mvvelo
    move_request.mvacc = mvacc
    move_request.mvtime = mvtime
    move_request.mvradii = mvradii
    move_request.coord_mode = coord_mode
    move_request.index = index
    joint_move(move_request)



def move_end_pre(joint_move):
    move_request = MoveRequest()
    move_request.pose = end_pre_pose
    move_request.has_ref = has_ref
    move_request.ref_joint = ref_joint
    move_request.mvvelo = mvvelo
    move_request.mvacc = mvacc
    move_request.mvtime = mvtime
    move_request.mvradii = mvradii
    move_request.coord_mode = coord_mode
    move_request.index = index
    joint_move(move_request)

def move_end(joint_move):
    move_request = MoveRequest()
    move_request.pose = end_pose
    move_request.has_ref = has_ref
    move_request.ref_joint = ref_joint
    move_request.mvvelo = mvvelo
    move_request.mvacc = mvacc
    move_request.mvtime = mvtime
    move_request.mvradii = mvradii
    move_request.coord_mode = coord_mode
    move_request.index = index
    joint_move(move_request)

def move_arm(joint_move, target_num = 5):
    # 到达抓取等待区
    move_request = MoveRequest()
    move_request.pose = target_pre_position[target_num]
    move_request.has_ref = has_ref
    move_request.ref_joint = ref_joint
    move_request.mvvelo = mvvelo
    move_request.mvacc = mvacc
    move_request.mvtime = mvtime
    move_request.mvradii = mvradii
    move_request.coord_mode = coord_mode
    move_request.index = index
    joint_move(move_request)

    # 到达抓取区
    move_request = MoveRequest()
    move_request.pose = target_position[target_num]
    move_request.has_ref = has_ref
    move_request.ref_joint = ref_joint
    move_request.mvvelo = mvvelo
    move_request.mvacc = mvacc
    move_request.mvtime = mvtime
    move_request.mvradii = mvradii
    move_request.coord_mode = coord_mode
    move_request.index = index
    joint_move(move_request)

    # 关爪子
    close_gripper()

    # 到达抓取等待区
    move_request = MoveRequest()
    move_request.pose = target_pre_position[target_num]
    move_request.has_ref = has_ref
    move_request.ref_joint = ref_joint
    move_request.mvvelo = mvvelo
    move_request.mvacc = mvacc
    move_request.mvtime = mvtime
    move_request.mvradii = mvradii
    move_request.coord_mode = coord_mode
    move_request.index = index
    joint_move(move_request)

    #到达放置准备区
    move_end_pre(joint_move)

    # #到达放置区
    # move_end(joint_move)

    #开爪子
    open_gripper()

    # #到达中央准备区
    # move_center_pre(joint_move)


def get_fruits_position_from_tpoic():
    rospy.Subscriber("fruits_position", Int32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin() # 当有新的消息到达订阅的topic时，rospy.spin()会自动调用相应的回调函数

def callback(data):
    global target_list
    # rospy.loginfo(rospy.get_caller_id() + "以下位置有水果：%s", data)
    if len(data.data) > 0:
        target_list = data.data

def call_arm_service(target):
    # 等待服务可用
    rospy.wait_for_service('/jaka_driver/joint_move')  # 等待服务启动
    try:
        # 创建服务代理
        joint_move = rospy.ServiceProxy('/jaka_driver/joint_move', Move)
        for i in target:
            move_arm(joint_move, i)
    
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def open_gripper():
    # 创建一个发布者，发布到 set_gripper 话题
    pub = rospy.Publisher('set_gripper', Int32, queue_size=10)
    
    rospy.sleep(0.5)
    # 创建消息并发送
    my_info = Int32()
    my_info.data = 0  # 设置数据为 0
    
    # 发布消息
    pub.publish(my_info)
    # rospy.sleep(0.5) #等待爪子到达指定位置
    rospy.loginfo("Gripper opened with value: %d", my_info.data)

def close_gripper():
    # 创建一个发布者，发布到 set_gripper 话题
    pub = rospy.Publisher('set_gripper', Int32, queue_size=10)
    
    rospy.sleep(0.5) # 等待发布者建立连接
    # 创建消息并发送
    my_info = Int32()
    my_info.data = 1  # 设置数据为 1
    
    # 发布消息
    pub.publish(my_info)
    # rospy.sleep(0.5) #等待爪子到达指定位置
    rospy.loginfo("Gripper closed with value: %d", my_info.data)

if __name__ == "__main__":
    rospy.init_node('control_arm_node',anonymous=True)  # 初始化节点
    move_start()
    open_gripper()
    threading.Thread(target=get_fruits_position_from_tpoic).start()
    while True:
        if len(target_list) > 0:
            call_arm_service(target_list)
            target_list = []
            move_start()
    






