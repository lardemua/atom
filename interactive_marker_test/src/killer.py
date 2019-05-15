import rospy
import subprocess
import os
if __name__ == '__main__':
    rospy.init_node('killer')
    os.system("rosnode kill /sensors_first_guess")
    # cmd = "rosnode kill /sensors_first_guess"
    # # print "Executing command: " + cmd
    # p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    # for line in p.stdout.readlines():
    #     print(line)
    #     p.wait()

    rospy.sleep(1)

    # cmd = "clear && rosrun interactive_marker_test create_first_guess.py"
    # # print "Executing command: " + cmd
    # p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    # for line in p.stdout.readlines():
    #     print(line)
    #     p.wait()

    rospy.spin()

