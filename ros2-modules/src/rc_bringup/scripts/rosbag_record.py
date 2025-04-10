'''
保存启动时候的ros2话题到data,之后的话题不会被记录
最多保存n个文件夹,每个文件夹最大max_size G
'''
import sys
import subprocess
import time
import signal
import os
import argparse
#默认记录在
def getRecordPath(num=5):
    record_root_path=os.path.abspath(sys.path[0] + '/../../../data/')
    #查询有几个文件夹
    record_dirs = [d for d in os.listdir(record_root_path) if os.path.isdir(os.path.join(record_root_path, d))]
    #按照创建时间排序
    record_dirs.sort(key=lambda d: os.path.getctime(os.path.join(record_root_path, d)))
    folder_count = len(record_dirs)
    if(folder_count>=num):
        #删除最早的文件夹
        os.system('rm -rf '+record_root_path+'/'+record_dirs[0])
    #创建新的文件夹
    file_name = time.strftime("%m-%d-%H-%M", time.localtime())
    file_path = record_root_path + '/' + file_name
    # print("\033[1;31;40m您输入的帐号或密码错误！\033[0m")
    print("\033[1;35mrecord path is {} \033[0m".format(file_path))
    return file_path
# 获取启动时的话题列表
def get_current_topics():
    result = subprocess.run(['ros2', 'topic', 'list'], stdout=subprocess.PIPE)
    topics = result.stdout.decode('utf-8').split('\n')
    return [topic for topic in topics if topic]

# 启动 ros2 bag 记录
def start_bag_recording(topics,file_path):
    if(topics==[]):
        print("\033[1;35mno topics to record\033[0m")
        return
    process=subprocess.Popen(['ros2', 'bag', 'record'] + topics+['-o',file_path])
    return process
def stop_bag_recording(file_path,max_size=2):
    #当文件大于2G或者收到停止信号时停止记录
    #查找文件路径下所有文件大小
    total_size = 0
    for root, dirs, files in os.walk(file_path):
        for file in files:
            total_size += os.path.getsize(os.path.join(root, file))
    if(total_size>max_size*1024*1024*1024):
        return True
    else:
        return False
def get_topic_type(topic):
    result = subprocess.run(['ros2', 'topic', 'info', topic], stdout=subprocess.PIPE)
    info = result.stdout.decode('utf-8')
    # 查找消息类型行，返回消息类型
    for line in info.splitlines():
        if line.startswith('Type:'):
            return line.split(':')[1].strip()
    return None
# if __name__ == "__main__":
def main():
    parser = argparse.ArgumentParser(description='ROS2话题录制脚本')
    parser.add_argument('--max_num', type=int, default=5, 
                        help='最大保存文件夹数量 (默认:5)')
    parser.add_argument('--max_size', type=int, default=2,
                        help='单个文件夹最大大小(GB) (默认:2)')
    parser.add_argument('--use_tf', action='store_true',
                        help='是否记录TF话题 (默认:False)')
    parser.add_argument('--record_image', action='store_false',)
    args = parser.parse_args()

    file_path = getRecordPath(args.max_num)
    topics = get_current_topics()
    
    topic_filtered = []
    for (topic) in topics:
        topic_type = get_topic_type(topic)
        if topic_type=='sensor_msgs/msg/PointCloud2' or topic_type=='sensor_msgs/msg/Imu':
            topic_filtered.append(topic)
        elif(args.use_tf and topic_type=='tf2_msgs/msg/TFMessage'):
            topic_filtered.append(topic)
        #排除带有compressedDepth后缀的话题
        elif(args.record_image and topic_type=='sensor_msgs/msg/CompressedImage' and 'compressedDepth' not in topic):
            topic_filtered.append(topic)
    print("\033[1;35mtopics are {}\033[0m".format(topic_filtered))
    rosbag_process=start_bag_recording(topic_filtered,file_path)    # 开始记录

    while True:
        if stop_bag_recording(file_path,args.max_size):
            for i in range(10):
                print("\033[1;35mstop recording\033[0m")
            break
        time.sleep(5)
    #结束记录
    rosbag_process.send_signal(signal.SIGINT)  # 发送 Ctrl+C 信号优雅终止
    rosbag_process.wait()  # 等待进程结束
if __name__ == "__main__":
    main()