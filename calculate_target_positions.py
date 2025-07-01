from scipy.spatial.transform import Rotation
import numpy as np

class TargetCalculator:
    def __init__(self):
        self.read_data()
        self.calculate_initial()
        return
        

    def read_data(self):
        f = open('./US_pics/targets/targets.txt','r')
        lines = f.readlines()
        f.close()
        self.targets = np.zeros((len(lines),3))
        self.positions = np.zeros((len(lines),3))
        for i in range(len(lines)):
            targets = lines[i].split('(')[1].split(')')[0].split(',')
            t = np.array([float(targets[0]),float(targets[1]),float(targets[2])])
            self.targets[i,:] = t
            needles = lines[i].split('[')[1].split(']')[0].strip(' ').replace('  ',' ')
            while '  ' in needles:
                needles = needles.replace('  ',' ')
            needles = needles.split(' ')
            n = np.array([float(needles[0]),float(needles[1]),float(needles[2])])
            self.positions[i,:] = n
        return
    
    def calculate_initial(self):
        self.probe_pose = np.eye(4)
        self.probe_pose[:3,3] = np.array([34.9,62.85,-70.96])
        rot = Rotation.from_euler('ZYX',[-174.18,-0.40,-2.82],degrees=True)
        self.probe_pose[:3,:3] = rot.as_matrix()
        self.probe_pose = np.array([[ -1.0,   0,   0,  20],
                                    [  0,  -1.0,   0, -30],
                                    [  0,   0,   1, -35],
                                    [  0,   0,   0,   1]])
        return


    def calculate_target(self,target_no):
        rot = Rotation.from_euler('x',self.targets[target_no][2],degrees=False)
        rot_mat = np.zeros((4,4))
        rot_mat[:3,:3] = rot.as_matrix()
        rot_mat[3,3] = 1
        target_position = np.matmul(self.probe_pose ,rot_mat)
        trans_mat = np.eye(4)
        trans_mat[:3,3] = np.array([30-self.targets[target_no][0],-self.targets[target_no][1]+9,0])
        target_position = np.matmul(target_position,trans_mat)
        return target_position[:3,3]
    
    def calculate_target2(self,target_no):
        target = [self.probe_pose[0,3]+(self.targets[target_no][0]-30), self.probe_pose[1,3]+(self.targets[target_no][1] + 9)*np.cos(self.targets[target_no][2]), self.probe_pose[2,3]+(self.targets[target_no][1] + 9)*np.sin(self.targets[target_no][2])]
        return target

def main():
    calc = TargetCalculator()
    print(calc.probe_pose)
    for i in range(np.size(calc.targets,0)):
        print('Target {}: {}; Needle position : {}'.format(i+1, calc.calculate_target2(i), calc.positions[i]))
    
main()