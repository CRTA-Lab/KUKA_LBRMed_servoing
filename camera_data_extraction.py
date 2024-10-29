from sksurgerynditracker.nditracker import NDITracker
import ndicapy
import socket
import datetime
import numpy as np
import numpy.linalg as lin
from scipy.spatial.transform import Rotation
from scipy.signal import butter,sosfilt,sosfilt_zi
import matplotlib.pyplot as plt


def setup_TCP():
    '''
    Function for configuration of TCP/IP connection with the robot
    in : none
    out : socket
    '''
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(('192.168.40.200',30000))
    print('Connected to server!')
    return client

def camera_setup():
    '''
    Fonction fo confiugration of the connection to the camera and initiation of the tracking
    in : none
    out : NDITracker - camera comunication object
    '''
    SETTINGS = {
        "tracker type": "vega",
        "ip address": "169.254.121.249",
        "romfiles" : ["C:\\Files\\Polaris_integration\\markeri\\8700339.rom",
                      "C:\\Files\\Polaris_integration\\markeri\\FEM0001.rom",
                      "C:\\Files\\Polaris_integration\\markeri\\8700338.rom"],
        
        }
    TRACKER = NDITracker(SETTINGS)
    # TODO set right tracking frequency
    ndicapy.ndiCommand(TRACKER._device,'SET Param.Tracking.Track Frequency=2')
    

    TRACKER.start_tracking()
    return TRACKER

def avg_filter(data):
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


def calculate_transform(handles,transforms):
    '''
    Function for calculation of transformation between fixed and mobile marker from their transforms in coordinate frame of the camera
    in :    handles - lista prepoznatih markera
            transforms - lista matrica transformcija pojedinih markera u odnosu na koordinatni sustav kamere
    out:    trans - translacijski vektor pomičnog markera u koordinatnom sustavu fiksnog markera
            rot_angles - vektor eulerovih kutova zakreta pomičnog markera u koordinatnom sustavu fiksnog markera, eulerovi kutovi su u redosljedu Z,Y,X kompatibilnim sa KUKA kontrolerom
    '''
    if handles[0] == 1:
        T_st = np.array(transforms[0])
        T_din = np.array(transforms[1])
    else:
        print('a jebiga')
        T_st = np.array(transforms[1])
        T_din = np.array(transforms[0])

    if np.isnan(T_st).any() or np.isnan(T_din).any():
        return np.nan, np.nan
    T_robot = np.matmul(lin.inv(T_st),T_din)

    rot = T_robot[:3,:3]
    r = Rotation.from_matrix(rot)
    rot_angles = r.as_euler('ZYX',degrees=False)

    trans = T_robot[:3,-1]
    

    return trans,rot_angles


def create_msg(trans,rot):
    '''
    Funkcija za stvaranje poruke za prijenos preko TCP/IP veze

    in :    trans - vektor translacije pomičnog markera
            rot - vektor eulerovih kutova pomičnog markera u Z,Y,X redosljedu
    
    out :  msg - string poruka za prijenos preko mreže ili "0\\n" ako nema podataka sa kamere  
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

def calculate_distance(tramsforms):
    frame = tramsforms[0]
    target = tramsforms[1]
    tool = tramsforms[2]


    trans_z = np.array([1,0,0,0,0,1,0,0,0,0,1,-40,0,0,0,1])
    trans_z.shape = [4,4]
    target = np.matmul(target,trans_z)

    T_tool = np.matmul(lin.inv(frame),tool)
    T_target = np.matmul(lin.inv(frame),target)

    tool_pos = T_tool[:3,-1]
    target_pos = T_target[:3,-1]
    return tool_pos,target_pos


def parse_msg(msg):
    msg_list = msg.split(' ')
    pos = []
    for i in msg_list[1:4]:
        temp = i.split('=')
        pos.append(float(temp[1]))
    return np.array(pos)
    



def main():


    tracker = camera_setup()
    a = input('Press to start...')
    client = setup_TCP()


    prev_frame = 0
    n = 10
    k = 0
    data_buffer = np.zeros([n,6])
    position_marker = np.zeros([1,3])
    position_robot = np.zeros([1,3])
   
    data_raw = np.zeros([1,6])
    data_filt = np.zeros([1,6])

    filter = butter(6,0.6,btype='lowpass',output='sos',fs=60)
    zi = sosfilt_zi(filter)


    while 1:
        port_handles, timestamps, framenumbers, tracking, quality = tracker.get_frame()
        if framenumbers[0] != prev_frame:

            try:
                trans,rot = calculate_transform(port_handles,tracking)
                data_raw = np.vstack((data_raw,np.hstack((trans,rot))))
                if np.isnan(trans).any() or np.isnan(rot).any():
                    continue
                data_buffer = np.vstack((data_buffer,np.hstack((trans,rot))))
                data_buffer = np.delete(data_buffer,0,0)
                if k < n:
                    k += 1
                else:
                    trans,rot = avg_filter(data_buffer)
                    data_filt = np.vstack((data_filt,np.hstack((trans,rot))))

                    '''data = sosfilt(filter,data_buffer,axis=0)
                    trans = data[-1,:3]
                    rot = data[-1,3:]
                    data_filt = np.vstack((data_filt,data[-1,:]))'''
                    
                    msg = create_msg(trans,rot)

                    client.send(bytes(msg,'UTF-8'))
                    incoming_msg = client.recv(1024)

                    tool_pos,target_pos = calculate_distance(tracking)

                    position_marker = np.vstack((position_marker,target_pos))
                    position_robot = np.vstack((position_robot,tool_pos))

                prev_frame = framenumbers[0]

            except Exception as e:
                print('Dogodila se greška:\n{}'.format(e))
                tracker.stop_tracking()
                client.close()
                position_marker = np.delete(position_marker,0,0)
                position_robot = np.delete(position_robot,0,0)
                data_raw = np.delete(data_raw,[0,n+1],0)
                data_filt = np.delete(data_filt,0,0)
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
                plt.plot(np.arange(len(position_marker)),position_marker[:,0],label='marker')
                plt.plot(np.arange(len(position_robot)),position_robot[:,0],label='tool')
                plt.legend()
                plt.subplot(3,1,2)
                plt.plot(np.arange(len(position_marker)),position_marker[:,1],label='marker')
                plt.plot(np.arange(len(position_robot)),position_robot[:,1],label='tool')
                plt.legend()
                plt.subplot(3,1,3)
                plt.plot(np.arange(len(position_marker)),position_marker[:,2],label='marker')
                plt.plot(np.arange(len(position_robot)),position_robot[:,2],label='tool')
                plt.legend()
                plt.show()
                return
            #time.sleep(0.5)



        

if __name__ == '__main__':
    main()