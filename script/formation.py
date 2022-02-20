import os
import time
import subprocess

def main():
    subprocess.call("roslaunch multiuav_formation two_uav_and_one_insulator_formation.launch &", shell=True)
    time.sleep(10)

    print("Start uav2")
    os.system("timeout 1s rostopic pub /uav2/bridge/arm std_msgs/Bool 'True'")
    os.system("timeout 1s rostopic pub /uav3/bridge/arm std_msgs/Bool 'True'")
    os.system("timeout 1s rostopic pub /uav4/bridge/arm std_msgs/Bool 'True'")

    os.system("timeout 1s rostopic pub /uav2/autopilot/start std_msgs/Empty")
    os.system("timeout 1s rostopic pub /uav3/autopilot/start std_msgs/Empty")
    os.system("timeout 1s rostopic pub /uav4/autopilot/start std_msgs/Empty")
    time.sleep(4)

    os.system("timeout 1s rostopic pub /ready_to_formation std_msgs/Bool 'True'")

    time.sleep(10)

    os.system("timeout 1s rostopic pub /uav4_ready_to_fly std_msgs/Bool 'True'")

    time.sleep(6)
    os.system("timeout 1s rostopic pub /ready_to_yaw std_msgs/Bool 'True'")
    time.sleep(50)
    os.system("pkill -9 gzserver")


if __name__ == '__main__':
    main()