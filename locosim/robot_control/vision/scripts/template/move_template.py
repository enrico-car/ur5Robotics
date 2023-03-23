import os
import glob

path=os.path.join(os.path.expanduser("~"), "Desktop","ur5Robotics", "locosim", "robot_control","vision","scripts","template", "10", "*jpg")
path_q1=os.path.join(os.path.expanduser("~"), "Desktop","ur5Robotics", "locosim", "robot_control","vision","scripts","template", "0.75_0.3125")
path_q2=os.path.join(os.path.expanduser("~"), "Desktop","ur5Robotics", "locosim", "robot_control","vision","scripts","template", "0.75_0.475")
path_q3=os.path.join(os.path.expanduser("~"), "Desktop","ur5Robotics", "locosim", "robot_control","vision","scripts","template","0.75_0.6375")
path_q4=os.path.join(os.path.expanduser("~"), "Desktop","ur5Robotics", "locosim", "robot_control","vision","scripts","template", "0.5_0.3125")
path_q5=os.path.join(os.path.expanduser("~"), "Desktop","ur5Robotics", "locosim", "robot_control","vision","scripts","template", "0.5_0.475")
path_q6=os.path.join(os.path.expanduser("~"), "Desktop","ur5Robotics", "locosim", "robot_control","vision","scripts","template", "0.5_0.6375")
path_q7=os.path.join(os.path.expanduser("~"), "Desktop","ur5Robotics", "locosim", "robot_control","vision","scripts","template", "0.25_0.3125")
path_q8=os.path.join(os.path.expanduser("~"), "Desktop","ur5Robotics", "locosim", "robot_control","vision","scripts","template", "0.25_0.475")
path_q9=os.path.join(os.path.expanduser("~"),"Desktop","ur5Robotics", "locosim", "robot_control","vision","scripts","template", "0.25_0.6375")


for img in glob.glob(path):
    #print (img)
    img1=img.split("/")
    #print(img1[7])
    img1=img1[11]
    img1=img1.split("_")
    #classe=img1[0]
    x=img1[1]
    y=img1[2]
    #print(x,y)
    # # roll=img1[3]
    # pitch=img1[4]
    # yaw=img1[5][:len(img[5])-5]
    #print(yaw)
    if(x=='0.75' and y=='0.3125'):
        command="cp "+img+" "+path_q1
        os.system(command)
    elif(x=='0.75' and y=='0.475'):
        command="cp "+img+" "+path_q2
        os.system(command)
    elif(x=='0.75' and y=='0.6375'):
        command="cp "+img+" "+path_q3
        os.system(command)
    elif(x=='0.5' and y=='0.3125'):
        command="cp "+img+" "+path_q4
        os.system(command)
    elif(x=='0.5' and y=='0.475'):
        command="cp "+img+" "+path_q5
        os.system(command)
    elif(x=='0.5' and y=='0.6375'):
        command="cp "+img+" "+path_q6
        os.system(command)
    elif(x=='0.25' and y=='0.3125'):
        command="cp "+img+" "+path_q7
        os.system(command)
    elif(x=='0.25' and y=='0.475'):
        command="cp "+img+" "+path_q8
        os.system(command)
    elif(x=='0.25' and y=='0.6375'):
        command="cp "+img+" "+path_q9
        os.system(command)