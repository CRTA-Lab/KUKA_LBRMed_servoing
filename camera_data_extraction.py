from sksurgerynditracker.nditracker import NDITracker
import sksurgerycore.transforms.transform_manager as tm
import socket
import time
import numpy as np
import numpy.linalg as lin
from scipy.spatial.transform import Rotation

def setup_TCP():
    '''
    Funkcija za konfiguraciju TCP/IP veze sa robotom
    in : none
    out : socket
    '''
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(('192.168.40.200',30000))
    print('Connected to server!')
    return client

def camera_setup():
    '''
    Funkcija za konfiguraciju veze sa kamerom i pokretanje praćenja markera
    in : none
    out : NDITracker - objekt za upravljanje veze sa kamerom
    '''
    SETTINGS = {
        "tracker type": "vega",
        "ip address": "169.254.121.249",
        "romfiles" : ["C:\\Files\\Polaris_integration\\markeri\\8700339.rom","C:\\Files\\Polaris_integration\\markeri\\FEM0001.rom"]
        }
    TRACKER = NDITracker(SETTINGS)

    TRACKER.start_tracking()
    return TRACKER

def calculate_transform(handles,transforms):
    '''
    Funkcija za izračun transformacije između fiksnog i pomičnog markera iz njihovih položaja u koordinatnom sustavu kamere
    in :    handles - lista prepoznatih markera
            transforms - lista matrica transformcija pojedinih markera u odnosu na koordinatni sustav kamere
         
    out:    trans - translacijski vektor pomičnog markera u koordinatnom sustavu fiksnog markera
            rot_angles - vektor eulerovih kutova zakreta pomičnog markera u koordinatnom sustavu fiksnog markera, eulerovi kutovi su u redosljedu Z,Y,X kompatibilnim sa KUKA kontrolerom
    '''
    if handles[0] == 1:
        T_st = np.array(transforms[0])
        T_din = np.array(transforms[1])
    else:
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




def main():


    tracker = camera_setup()
    a = input('Press to start...')
    client = setup_TCP()
    

    prev_frame = 0

    while 1:
        port_handles, timestamps, framenumbers, tracking, quality = tracker.get_frame()
        if framenumbers[0] != prev_frame:

            try:
                trans,rot = calculate_transform(port_handles,tracking)
                
                msg = create_msg(trans,rot)

                client.send(bytes(msg,'UTF-8'))

            except Exception as e:
                print('Dogodila se greška:\n{}'.format(e))
                tracker.stop_tracking()
                client.close()
                return
            #time.sleep(0.5)
        

if __name__ == '__main__':
    main()