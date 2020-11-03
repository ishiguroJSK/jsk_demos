#!/usr/bin/python
#-*- coding: utf-8 -*-

# 難点
# client -> server には非同期送信なのでコールバックやタイマーで勝手なタイミングで高頻度に送信すると詰まって止まる→ループで定期送信
# server側の全(topic + type)を返すサービス(/rosapi/topics)が1HZくらいしか出ない．帯域が圧迫されると帰ってこない
# nodeがpubしたtopicを自分でsubしないように，そのtopicのpublishersをみてtopicを決める必要がある
# client側は全(topic + type + publisher)の情報が簡単に取得できるが(これもmelodic以降？)，server側は(topic + type)までしかない
# server側からdownloadする有効なtopicsは，(全topic) - (/rosbridge_websocketがpubしているtopic)
# しかしそれだとserver側でsubしているだけのtopic名もclient側にpubが形成されてコンフリクトする
# server側とclient側で同じtopicに対してpubしているときはclient->serverへの送信を優先
# server側からしっかりと全(topic + type + publisher)の情報を返す仕組みを作ったらいいのでは．なんならserviceやめてtopicで定期的に垂れ流したほうが．
# 全topicのpublisher作るとubuntuの1プロセスのファイルディスクリプタの上限=1024に達する
# そしてserver側のプロセス負荷も高い


import rospy
import rostopic
import roslib.message
import roslibpy
from functools import partial
from rosbridge_library.internal import message_conversion

class ServerToClientConnection:
    def __init__(self, topic, type, websocket_handler):
        self.topic      = topic
        self.type       = type
        self.ws_sub     = roslibpy.Topic(websocket_handler, self.topic, self.type)
        self.ros_pub    = rospy.Publisher(self.topic, roslib.message.get_message_class(self.type), queue_size=1)

    def start(self): # start downloading from server
        self.ws_sub.subscribe(self.cb_server_sub_to_client_pub)

    def stop(self): # just stop downloading from server
        self.ws_sub.unsubscribe()

    def shutdown(self): # unregister unpublished server topic from client topic list
        self.stop()
        self.ros_pub.unregister()

    def cb_server_sub_to_client_pub(self, ws_msg): # this callback is commonly called by any topic/type
        ros_msg_instance = roslib.message.get_message_class(self.ros_pub.type)() # 'geometry_msgs/PoseStamped' -> PoseStamped()
        message_conversion.populate_instance(ws_msg, ros_msg_instance) # convert JSON to ros msg
        self.ros_pub.publish(ros_msg_instance)

class ClientToServerConnection:
    latest_ros_msg = None # used for throttling
    has_new_ros_msg = False
    def __init__(self, topic, type, websocket_handler):
        self.topic      = topic
        self.type       = type
        self.ros_sub    = rospy.Subscriber(self.topic, roslib.message.get_message_class(self.type), self.cb_client_sub_to_server_pub)
        self.ws_pub     = roslibpy.Topic(websocket_handler, self.topic, self.type)

    def start(self): # start uploading to server
        self.ws_pub.advertise()

    def stop(self): # just stop uploading to server
        self.ws_pub.unadvertise()

    def shutdown(self): # unregister unpublished server topic from client topic list
        self.stop()
        self.ros_sub.unregister()

    def cb_client_sub_to_server_pub(self, ros_msg):
        self.latest_ros_msg = ros_msg
        self.has_new_ros_msg = True
        # self.ws_pub.publish(message_conversion.extract_values(ros_msg)) # don't publish here, throttling is needed for save websocket traffic

    def send_latest_msg_once(self):
        if self.latest_ros_msg:
            self.ws_pub.publish(message_conversion.extract_values(self.latest_ros_msg))
            self.has_new_ros_msg = False

class WebSocketToClientROS:
    HZ = 10 # actually, around 1~2 hz
    s2c_cons = {} # { topic_name : ServerToClientConnection() } for server -> client
    c2s_cons = {} # { topic_name : ClientToServerConnection() } for server <- client
    # all published topics in server/client side except the ones which is published by this node
    server_topics_and_types_without_mine = [] # [('/aaa', 'std_msgs/Float64'), ('/bbb', 'geometry_msgs/PoseStamped')] = rostopic.get_topic_list() like style
    client_topics_and_types_without_mine = [] # [('/aaa', 'std_msgs/Float64'), ('/bbb', 'geometry_msgs/PoseStamped')] = rostopic.get_topic_list() like style
    server_side_rosbridge_name = 'rosbridge_websocket'
    client_side_rosbridge_name = 'websocket_to_client_ros'


    def __init__(self, hostname, port):
        import resource
        print('Max connection limit is, ' + str(resource.getrlimit(resource.RLIMIT_NOFILE)[0])) # if exceed, 'IOError: [Errno 24] Too many open files'

        rospy.loginfo('Init ROS node')
        rospy.init_node(self.client_side_rosbridge_name, anonymous=False) # this node should be only one

        rospy.loginfo('Setup WebSocket handler')
        self.ws_handler = roslibpy.Ros(hostname, port)
        self.ws_handler.run()

        rospy.loginfo('run()')
        self.run()


    def update_server_and_client_topic_info(self):
        self.update_client_topics_and_types_without_mine()
        self.update_server_topics_and_types_without_mine()
        # if same topic name is published in both server and client, delete/stop download connection from server side 
        for t, ty in self.client_topics_and_types_without_mine:
            if (t, ty) in self.server_topics_and_types_without_mine:
                # rospy.logwarn('Same topic ' + t + ' is found both in server and client. Delete this from download list')
                self.server_topics_and_types_without_mine.remove((t, ty))


    def update_server_topics_and_types_without_mine(self): # call service of /rosapi node in server side to get all topics/types (except: published by server_side_rosbridge, /rosout, /rosout_agg)
        topics_and_types = [] 
        server_side_rosbridge_info = []
        get_server_topics = roslibpy.Service(self.ws_handler, '/rosapi/topics', 'rosapi/Topics')
        try:
            topics_and_types = get_server_topics.call(roslibpy.ServiceRequest(), timeout = 1) # {'topics' : ['/aaa', '/bbb'], 'types' : ['std_msgs/Float64', 'geometry_msgs/PoseStamped']}
        except Exception as e:
            rospy.logwarn('Timeout in service response:' + get_server_topics.name + ' ' + str(e) + 'server topic info will not be updated now')
        
        get_server_node_details = roslibpy.Service(self.ws_handler, '/rosapi/node_details', 'rosapi/NodeDetails')
        try:
            server_side_rosbridge_info = get_server_node_details.call(roslibpy.ServiceRequest({'node': '/' + self.server_side_rosbridge_name}), timeout = 1) # {u'services': [u'/aaa/bbb', u'/ccc/ddd'], u'subscribing': ['/eee'], u'publishing': [u'/fff', u'/ggg']}
        except Exception as e:
            rospy.logwarn('Timeout in service response:' + get_server_node_details.name + ' ' + str(e) + 'server topic info will not be updated now')

        if topics_and_types and server_side_rosbridge_info:
            self.server_topics_and_types_without_mine = [ (t, ty) for t, ty in zip(topics_and_types['topics'], topics_and_types['types']) if t not in server_side_rosbridge_info['publishing'] and '/rosout' not in t]


    def update_client_topics_and_types_without_mine(self): # get all published topics in client side (except: published by this node, /rosout, /rosout_agg)
        client_topic_info = rostopic.get_topic_list()[0] # [('/aaa', 'std_msgs/Float64', ['/node1', '/node2]), ('/bbb', 'geometry_msgs/PoseStamped', [/node3'])] # over 18.04 ???
        self.client_topics_and_types_without_mine = []
        for t, ty, node_list in client_topic_info:
            if '/rosout' in t:
                continue                
            try:
                node_list.remove('/' + self.client_side_rosbridge_name)
            except ValueError:
                pass
            if node_list:
                self.client_topics_and_types_without_mine.append((t,ty))


    def update_server_to_client_connection(self): # advertize all server side topics, but not publish yet
        # (1) clean up about unadvertised topics in server
        for k, c in self.s2c_cons.items():
            if (c.topic, c.type) not in self.server_topics_and_types_without_mine:
                rospy.loginfo('Del websocket subscriber and ros publisher: '  + c.topic + ' (' + c.type + ')')
                c.shutdown()
                del self.s2c_cons[k]
        # (2) add newly found topics in server
        for t, ty in self.server_topics_and_types_without_mine:
            ##### hotfix: pr2 has over 1300 topics, reduce here #####
            if '_stereo' in t or '_gripper_sensor_controller' in t:
                continue
            if len(self.s2c_cons) > 1000: # ubuntu default ulimit of a process = 1024 
                rospy.logwarn('Too many topics !! Skip: ' + t + ' (' + ty + ')')
                continue
            ##########################################################
            if not self.ros_msg_class_exist_in_client(ty):
                rospy.logwarn_throttle(10, 'Can not get ros message class in client. Skip: ' + t + ' (' + ty + ') ' )
                continue
            if not self.s2c_cons.get(t):
                rospy.loginfo('Add websocket subscriber and ros publisher: ' + t + ' (' + ty + ')')
                self.s2c_cons[t] = ServerToClientConnection(t, ty, self.ws_handler)
        # (3) start receiving from server if the topic is subscrived now
        for c in self.s2c_cons.values():
            if      c.ros_pub.get_num_connections() > 0     and not c.ws_sub.is_subscribed: # if the advertising ros topic is subscribed, start websocket subscriber
                rospy.loginfo('Sub websocket subscriber: '      + c.topic + ' (' + c.type + ')')
                c.start()
            elif    c.ros_pub.get_num_connections() <= 0    and     c.ws_sub.is_subscribed: # if the advertising ros topic is not subscribed, and websocket subscriber is still exist
                rospy.loginfo('Unsub websocket subscriber: '    + c.topic + ' (' + c.type + ')')
                c.stop()


    def update_client_to_server_connection(self): # subscribe all client side topics
        # (1) clean up about unpublished topics in client
        for k, c in self.c2s_cons.items():
            if (c.topic, c.type) not in self.client_topics_and_types_without_mine:
                rospy.loginfo('Del websocket publisher and ros subscriber: '   + c.topic + ' (' + c.type + ')')
                c.shutdown()
                del self.c2s_cons[k]
        # (2) add newly found topics in client
        for t, ty in self.client_topics_and_types_without_mine:
            if not self.ros_msg_class_exist_in_client(ty):
                rospy.logwarn_throttle(10, 'Can not get ros message class in client. Skip: ' + t + ' (' + ty + ') ' )
                continue
            if not self.c2s_cons.get(t):
                rospy.loginfo('Add websocket publisher and ros subscriber: ' + t + ' (' + ty + ')')
                self.c2s_cons[t] = ClientToServerConnection(t, ty, self.ws_handler)
        # (3) stop sending to server if the topic is not subscrived at server side ?
        # ?????????? currently, all published ros msg in client will be sent to server if has_new_ros_msg


    def ros_msg_class_exist_in_client(self, msg_type):
        try:
            cls = roslib.message.get_message_class(msg_type)
            if not cls: # e.g. msg_type = 'unknown_msgs/UninstalledType'
                return None
            return cls
        except Exception as e: # e.g. msg_type = 'CompletelyNonsenseStr'
            return None


    def run(self):
        while not rospy.is_shutdown():
            self.update_server_and_client_topic_info()
            self.update_server_to_client_connection()
            self.update_client_to_server_connection()

            for c in self.c2s_cons.values():
                if c.has_new_ros_msg:
                    c.send_latest_msg_once()

            # rospy.loginfo_throttle(1,\
            rospy.loginfo(\
                '[server] topic: '  + str(len(self.server_topics_and_types_without_mine)) + \
                ' / ws pub: ('      + str(sum(1 for c in self.c2s_cons.values() if c.ws_pub.is_advertised))             + '/' + str(len(self.c2s_cons)) + ')' + \
                ' / ws sub: ('      + str(sum(1 for c in self.s2c_cons.values() if c.ws_sub.is_subscribed))             + '/' + str(len(self.s2c_cons)) + ')' + \
                ' <==> [client]'    + \
                ' / ros pub: ('     + str(sum(1 for c in self.s2c_cons.values() if c.ros_pub.get_num_connections()))    + '/' + str(len(self.s2c_cons)) + ')' + \
                ' / ros sub: ('     + str(sum(1 for c in self.c2s_cons.values() if c.ros_sub.get_num_connections()))    + '/' + str(len(self.c2s_cons)) + ')' )
            rospy.Rate(self.HZ).sleep()
        
        
if __name__ == '__main__':
    instance = WebSocketToClientROS('pr1040.jsk.imi.i.u-tokyo.ac.jp', 9090)