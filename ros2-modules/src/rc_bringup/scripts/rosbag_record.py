'''
保存启动时候的ros2话题到data,之后的话题不会被记录
最多保存n个文件夹,每个文件夹最大max_size G
'''
import sys
import subprocess
import time
import os
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
    subprocess.run(['ros2', 'bag', 'record'] + topics+['-o',file_path])
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
if __name__ == "__main__":
    print("命令行参数:", sys.argv)
    if len(sys.argv)> 2:
        max_num=int(sys.argv[1])
        max_size=int(sys.argv[2])
    else:
        max_num=5
        max_size=2
    file_path=getRecordPath(max_num)
    topics = get_current_topics()  # 获取当前已有的话题
    print("\033[1;35mtopics are {}\033[0m".format(topics))
    start_bag_recording(topics,file_path)    # 开始记录

    while True:
        if stop_bag_recording(file_path,max_size):
            break
        time.sleep(5)
