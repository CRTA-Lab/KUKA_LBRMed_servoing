from sksurgerynditracker.nditracker import NDITracker
import ndicapy
import socket
import datetime
import numpy as np
import numpy.linalg as lin
from scipy import optimize
from scipy.spatial.transform import Rotation
from scipy.signal import butter,sosfilt,sosfilt_zi
import matplotlib.pyplot as plt


def setup_TCP():
    '''
    Configurates the TCP/IP connection with the robot

    Args : none

    Output : socket
    '''
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(('192.168.40.200',30000))
    print('Connected to server!')
    return client

def camera_setup():
    '''
    Configures the connection to the camera and initiation of the tracking

    Args : none

    Output : NDITracker - camera comunication object
    '''
    SETTINGS = {
        "tracker type": "vega",
        "ip address": "169.254.121.249",
        "romfiles" : ["C:\\Files\\Polaris_integration\\markeri\\8700339.rom",
                      "C:\\Files\\Polaris_integration\\markeri\\FEM0001.rom",
                      "C:\\Files\\Polaris_integration\\markeri\\8700338.rom"],
        
        }
    TRACKER = NDITracker(SETTINGS)

    ndicapy.ndiCommand(TRACKER._device,'SET Param.Tracking.Track Frequency=2')
    

    TRACKER.start_tracking()
    return TRACKER

def avg_filter(data:np.ndarray):
    '''
    Applies avarageing filter on the data window

    Args :
        data - data window to be avaraged (ndarray[n,6])

    Output :
        trans - avaraged translation data (ndarray[1,3])
        rot - avaraged rotation data (ndarray[1,3])
    '''
    trans = data[0,:3]
    rot = data[0,3:]
    order = data.shape[0]
    for i in range(6):
        for j in range(1,order):
            if i < 3:
                trans[i] += data[j,i]
            else:
                rot[i-3] += data[j,i]
    return trans/order,rot/order


def calculate_transform(transforms:list,tool = False):
    """
    Calculates the transformation between fixed and mobile marker from their transforms in coordinate frame of the camera

    Args :
        handles - list of the marker labels
        transforms - list of transformation matrices of each marker
    Output:
        trans - translation vector of the mobile marker w/respect to fixed marker (ndarray[1,3])
        rot_angles - euler angle vector of the mobile marker w/respect to the fixed marker (ndarray[1,3]), angles are in Z,Y',X'' order compatible wiht KUKA controler
    """
    T_st = np.array(transforms[0])
    T_din = np.array(transforms[1+tool])

    if np.isnan(T_st).any() or np.isnan(T_din).any():
        return np.nan, np.nan
    T_robot = np.matmul(lin.inv(T_st),T_din)

    rot = T_robot[:3,:3]
    r = Rotation.from_matrix(rot)
    rot_angles = r.as_euler('ZYX',degrees=False)

    trans = T_robot[:3,-1]
    
    return trans,rot_angles


def create_msg(trans:np.ndarray,rot:np.ndarray):
    '''
    Creating the message to be sent ovet TCP/IP

    Args :
        trans - translation vector (ndarray[1,3])
        rot - euler angle vector (ndarray[1,3])
    
    Output :
        msg - message assambled from the data, or 0\\n if data is unavailible (String)  
    '''
    if np.isnan(trans).any() or np.isnan(rot).any():
        return '0\n'
    msg = '1'
    for i in trans:
        msg += ',' + str(i)
    for i in rot:
        msg += ',' + str(i)
    msg += '\n'
    return msg

def calculate_distance(tramsforms:list):
    '''
    Calculation of the tool and target position in the coordinate frame of the fixed marker

    Args:
        transforms - list of the transformation matrices od the markers
    
    Output:
        tool_pos - position of the tool in coordinate frame of the fixed marker
        target_pos - position of the target in coordinate frame of the fixed marker
    '''
    frame = tramsforms[0]
    target = tramsforms[1]
    tool = tramsforms[2]


    trans_z = np.array([1,0,0,0,0,1,0,0,0,0,1,-50,0,0,0,1])
    trans_z.shape = [4,4]
    target = np.matmul(target,trans_z)

    T_tool = np.matmul(lin.inv(frame),tool)
    T_target = np.matmul(lin.inv(frame),target)

    tool_pos = T_tool[:3,-1]
    target_pos = T_target[:3,-1]
    return tool_pos,target_pos


def parse_msg(msg:str):
    '''
    Parsing the robots coordinates from the message

    Args:
        msg - message receved from robot
    
    Output:
        T - transformation matrix from robot base to the TCP
    '''
    msg = msg.split(']')[0]
    msg_list = msg.split(' ')
    pos = []
    for i in msg_list:
        temp = i.split('=')
        pos.append(float(temp[1]))
    #print(pos)
    r = Rotation.from_euler('ZYX',pos[3:],degrees=False)
    rot = r.as_matrix()
    T = np.eye(4)
    T[:3,:3] = rot
    T[:3,-1] = pos[:3]
    return T

def frame_calibration(client:socket.socket,tracker:NDITracker):
    '''
    Calibrating the fixed frame on the robot using camera and robot data and sending the frame data to the robot

    Args:
        client - TCP client for comunication with the robot
        tracker - camera comunication object

    Output: None
    '''
    client.send(bytes('Ready\n','UTF-8'))
    prev_frame=0

    data = []

    for i in range(4):
        j = 0
        msg = client.recv(1024)
        while j < 60:
            port_handles, timestamps, framenumbers, tracking, quality = tracker.get_frame()
            if framenumbers[0] != prev_frame:
                
                T_T = tracking[2] # transform from camera to tool
                T_F = tracking[0] # transform from camera to fixed frame

                T_R = parse_msg(str(msg)) # transform from robots base to TCP

                # transform from robot base to fixed frame: T_B = T_R*T_T^(-1)*T_F
                T_B = np.matmul(T_R,lin.inv(T_T))
                T_B = np.matmul(T_B,T_F)

                data.append(T_B)
                
                client.send(bytes('Next\n','UTF-8'))

                j += 1
                #print('cekam poruku')
                msg = client.recv(1024)
                #print(msg)
                prev_frame = framenumbers[0]
                
        print('Idući')        
        client.send(bytes('Done\n','UTF-8'))
        print(data[-1])

    # avarageing the data of all the measurements
    def min_func(x:np.ndarray):
        sum = np.zeros([16,])
        for i in data:
            j = i.reshape((16,))
            sum += abs(j-x)
        #sum = sum.reshape((16,))
        return sum
    '''T = np.zeros([4,4])
    for i in data:
        T += i
    print(len(data))
    T /= len(data)
    print(T)'''

    x0 = data[0]
    x0 = x0.reshape((16,))
    print(type(x0))
    print(x0.shape)
    result = optimize.least_squares(min_func,x0)
    T = result.x
    T.shape = [4,4]
    
    # transforming the data into x,y,z,A,B,C format to send to the robot
    rot = T[:3,:3]
    r = Rotation.from_matrix(rot)
    rot_angles = r.as_euler('ZYX',degrees=False)
    trans = T[:3,-1]

    # creating and sending the message to the robot
    msg = create_msg(trans,rot_angles)
    print(msg)
    #a = input('Waiting...')
    client.send(bytes(msg,'UTF-8'))
    f = open('frame.txt','w')
    f.write(msg)
    f.close()
    return

def load_frame(client:socket.socket):
    f = open('frame.txt','r')
    msg = f.readline()
    client.send(bytes(msg,'UTF-8'))
    return


                


    



def main():

    tracker = camera_setup()
    #a = input('Press to start...')
    client = setup_TCP()

    msg = str(client.recv(1024))
    if 'Calibrate' in msg:
        frame_calibration(client,tracker)
    elif 'Load' in msg:
        load_frame(client)
        
    #initialization of the variables
    prev_frame = 0
    n = 3*60
    k = 0
    data_buffer = np.zeros([n,6])
    position_marker = np.zeros([1,3])
    position_robot = np.zeros([1,3])
   
    data_raw = np.zeros([1,6])
    data_filt = np.zeros([1,6])    

    #design of the Butterworth filter
    filter = butter(1,3,btype='lowpass',output='sos',fs=60)
    zi = sosfilt_zi(filter)
    zi = np.dstack((zi,zi,zi,zi,zi,zi))



    while 1:
        port_handles, timestamps, framenumbers, tracking, quality = tracker.get_frame()
        
        if framenumbers[0] != prev_frame:
            # try-catch loop that will end the program when server (robot app) breaks the connection
            prev_frame = framenumbers[0]
            try:
                trans,rot = calculate_transform(tracking)
                if np.isnan(trans).any() or np.isnan(rot).any():
                    continue
                data_raw = np.vstack((data_raw,np.hstack((trans,rot))))
                data_buffer = np.vstack((data_buffer,np.hstack((trans,rot))))
                data_buffer = np.delete(data_buffer,0,0)
                if k < n:
                    k += 1
                else:
                    #avarageing filter
                    #trans,rot = avg_filter(data_buffer)
                    #data_filt = np.vstack((data_filt,np.hstack((trans,rot))))

                    #Butterworkth filter
                    data = sosfilt(filter,data_buffer,axis=0)
                    trans = data[-1,:3]
                    rot = data[-1,3:]
                    data_filt = np.vstack((data_filt,data[-1,:]))
                    
                    msg = create_msg(trans,rot)

                    client.send(bytes(msg,'UTF-8'))
                    #incoming_msg = client.recv(1024)

                    tool_pos,target_pos = calculate_distance(tracking)

                    position_marker = np.vstack((position_marker,target_pos))
                    position_robot = np.vstack((position_robot,tool_pos))


            except OSError as e:
                print('Dogodila se greška:\n{}'.format(e))
                tracker.stop_tracking()
                client.close()
                position_marker = np.delete(position_marker,0,0)
                position_robot = np.delete(position_robot,0,0)
                data_raw = np.delete(data_raw,[0,n+1],0)
                data_filt = np.delete(data_filt,0,0)
                #saving recorded data to file
                #data = np.hstack((position_marker,position_robot))
                #now = datetime.datetime.now()
                #np.savetxt('saved_data\\data{}_{}.csv'.format(now.date(),now.hour,now.minute),data,delimiter=',')
                plt.subplot(3,1,1)
                plt.plot(np.arange(len(data_raw)),data_raw[:,0],label='raw')
                plt.plot(np.arange(n,len(data_filt)+n),data_filt[:,0],label='filtered')
                plt.legend()
                plt.subplot(3,1,2)
                plt.plot(np.arange(len(data_raw)),data_raw[:,1],label='raw')
                plt.plot(np.arange(n,len(data_filt)+n),data_filt[:,1],label='filtered')
                plt.legend()
                plt.subplot(3,1,3)
                plt.plot(np.arange(len(data_raw)),data_raw[:,2],label='raw')
                plt.plot(np.arange(n,len(data_filt)+n),data_filt[:,2],label='filtered')
                plt.legend()
                plt.show()
                
                plt.subplot(3,1,1)
                plt.plot(np.arange(len(position_marker)-500),position_marker[500:,0],label='marker')
                plt.plot(np.arange(len(position_robot)-500),position_robot[500:,0],label='tool')
                plt.legend()
                plt.subplot(3,1,2)
                plt.plot(np.arange(len(position_marker)-500),position_marker[500:,1],label='marker')
                plt.plot(np.arange(len(position_robot)-500),position_robot[500:,1],label='tool')
                plt.legend()
                plt.subplot(3,1,3)
                plt.plot(np.arange(len(position_marker)-500),position_marker[500:,2],label='marker')
                plt.plot(np.arange(len(position_robot)-500),position_robot[500:,2],label='tool')
                plt.legend()
                plt.show()
                return
            #time.sleep(0.5)



        

if __name__ == '__main__':
    main()