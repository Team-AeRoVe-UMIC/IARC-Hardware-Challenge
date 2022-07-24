#!/usr/bin/env python3
import controller_ardu as controler
from time import sleep
import numpy as np
import math
import matplotlib.pyplot as plt
import PID
import rospy

'lol just checking git'
def next_move(xyz, angle, clockwise = True):
    #print(clockwise)
    
    if clockwise:
        r = math.sqrt(np.sum(np.array(xyz)*np.array(xyz)))  
        print('angle =', angle,' r =',r)
        #if angle > 5 or r>2:
            #r = math.sqrt(np.sum(np.array(xyz)*np.array(xyz)))                   #*0.7
        # if r < 2:
        #     #return 0,0
        #     r = 4
        dx = -1*r*(math.cos(math.radians(angle - 5)) - math.cos(math.radians(angle))) *3.5
        dy = -1*r*(math.sin(math.radians(angle - 5)) - math.sin(math.radians(angle)))
        return dx, dy
        # else:
        #     return 0,0

    else:
        r = math.sqrt(np.sum(np.array(xyz)*np.array(xyz)))  
        #if angle > 5 or r>2:
            #r = math.sqrt(np.sum(np.array(xyz)*np.array(xyz)))
        # if r < 2:
        #     #return 0,0
        #     r = 4
        dx = r*(math.cos(math.radians(angle - 5)) - math.cos(math.radians(angle))) *3.5
        dy = -r*(math.sin(math.radians(angle - 5)) - math.sin(math.radians(angle)))
        return dx, dy
        # else:
        #     return 0,0

def d_yaw_cal(xyz):
    return math.degrees(math.atan2(xyz[2], xyz[0])) - 90




class follow:
    def __init__(self, xyz0) -> None:
        self.pidx = PID.PID()
        self.pidx.Kp = 0.8
        self.pidx.Kd = 1
        self.pidz = PID.PID()
        self.pidy = PID.PID()
        self.pidy.Kp = 0.8
        self.pidy.Kd = 0.8
        self.frequency = 10
        self.xyz0 = xyz0

    def relative_vel(self):
        self.rel_posx = self.xyz1[0]
        self.rel_posz = self.xyz1[2]
        self.rel_posy = self.xyz1[1]
        self.rel_velx = (self.xyz1[0] - self.xyz0[0])*self.frequency
        self.rel_vely = (self.xyz1[1] - self.xyz0[1])*self.frequency
        self.rel_velz = (self.xyz1[2] - self.xyz0[2])*self.frequency

    def update(self, xyz1):
        self.xyz1 = xyz1
        self.relative_vel()
        self.xyz0 = self.xyz1
        self.outx = self.pidx.update(self.rel_posx, self.rel_velx) # acceleration in x and z
        self.outz = self.pidz.update(self.rel_posz, self.rel_velz)
        self.outy = self.pidy.update(self.rel_posy, self.rel_vely)


# def clear_rotation_offset():

def camera_to_local(x,y,z):
    # return z, -x,-y 
    return x, z, -y

def local_to_camera(x,y,z):
    # return -y, -z, x
    return x, -z, y 



if __name__ == '__main__':
    
    ic = controler.Flight_controller(cv= True)
    
    rate = rospy.Rate(10) # rate of publishing acceleration commands
    sleep(2)
    #ic.test_control()
    # print(ic.curr_x)
    # ic.toggle_arm(1)
    # sleep(0.5)
    # ic.set_offboard_mode()
    # sleep(1.5)
    # ic.takeoff(5)
    # sleep(1.5)
    mast=[-4.98675,5]
    to_go=[-4,2,0]
    to_yaw= math.degrees(math.atan2((mast[1]-to_go[1]),(mast[0]-to_go[0])))
    print('to_yaw = ',to_yaw) 
    ic.set_orientation(0,0,to_yaw)
    ic.set_waypoint(to_go[0],to_go[1],1.2)
    #print('sd')
    ic.set_pose()
    print('df')
    mast_approach=True
    radius=2*math.sqrt((ic.curr_x-mast[0])**2 + (ic.curr_y-mast[1])**2)
    while True:
        print('while loop running')
        
        if ic.got_mldata :
            print('detected')
            print(ic.mast_xyz)

            while ic.mast_angle > 30:
                print('mast angle : ',ic.mast_angle)
                if ic.mast_sense < 0:
                    clockwise = True
                    dx,dy=-0.3,0
                else:
                    clockwise = False
                    dx,dy=0.3,0
                # print('mast angle = '+str(ic.mast_angle))
                
                
                dyaw = d_yaw_cal(ic.mast_xyz)
                yaw = math.radians(ic.curr_yaw) # get the current yaw of the drone
                rotation_matrix = np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]])

                new_dx, new_dy = np.matmul(rotation_matrix, np.array([dx, dy]))
                
                ic.d_yaw(dyaw=dyaw)
                # ic.pub_next_pose()
                ic.move_wrtDrone_fixedaxis(new_dy, new_dx, 0)
                ic.pub_next_pose()
                rate.sleep()

            r = math.sqrt(np.sum(np.array(ic.mast_xyz)*np.array(ic.mast_xyz)))
            print(r)
            while r>2: #ic.mast_angle > 5 or r>2:
                
                if ic.mast_sense < 0:
                    clockwise = True
                else:
                    clockwise = False
                # print('mast angle = '+str(ic.mast_angle))
                dx, dy = next_move(ic.mast_xyz, ic.mast_angle, clockwise)
                
                dyaw = d_yaw_cal(ic.mast_xyz)
                yaw = math.radians(ic.curr_yaw) # get the current yaw of the drone
                rotation_matrix = np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]])

                new_dx, new_dy = np.matmul(rotation_matrix, np.array([dx, dy]))
                
                ic.d_yaw(dyaw=dyaw)
                # ic.pub_next_pose()
                ic.move_wrtDrone_fixedaxis(new_dy, new_dx, 0)
                ic.pub_next_pose()
                rate.sleep()
                r = math.sqrt(np.sum(np.array(ic.mast_xyz)*np.array(ic.mast_xyz)))
                print('mast angle : ',ic.mast_angle, ' r=',r)
            print ("TRACKING NOW!!")
            print('Mast approached')
            break
        
            ic.set_waypoint(ic.curr_x,ic.curr_y,ic.curr_z)
            ic.set_pose()


            # exit()
            # foll = follow(ic.mast_xyz)
            # while True:
            #     if ic.detected:
            #         foll.update(ic.mast_xyz)
            #         print(foll.outx,foll.outy,foll.outz)
            #         accel_x = foll.outx # left and right
            #         # accel_z = foll.outz
            #         accel_z = 0 # in and out
            #         accel_y = -foll.outz # up and down

            #         print('cam frame accel :',accel_x,accel_y)
            #         # yaw = math.radians(ic.curr_yaw)
            #         yaw = -1*math.radians(ic.curr_yaw) # get the current yaw of the drone
            #         rotation_matrix = np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]]) # define the rotation matrix
            #         accel_x,accel_y,accel_z = camera_to_local(accel_x,accel_y,accel_z) # get the acceleration of the drone in the real non-offset global frame
            #         accel_old = np.array([accel_x,accel_y])
            #         accel_new = np.matmul(rotation_matrix,accel_old)
            #         #print ("X-acceleration:",[accel_new,accel_z])
            #     else:
            #         accel_new = [0,0]
            #         accel_z = 0
            #     print('ground frame accel :',accel_new[0],accel_new[1], accel_z)
            #     #ic.accel_command(accel_new[0],accel_new[1], accel_z)
            #     r.sleep()

        else:

            pass