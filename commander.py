"""
Client side simulation controller.
2-phases: Formation lap for mapping with road centering. Use road_centering.py controller
          Then switch to LMPC at the end of formation lap.
          
"""

import glob
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla
import pickle
import random
import time
import numpy as np
import cv2
import math
from RunLMPC import RaceLMPC
from Track import Map
import weakref
try:
    import queue
except ImportError:
    import Queue as queue

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

class commander(object):
    def __init__(self, world, maxLaps, fps, vehicle, imu, lc_cam, trail_cam, gnss, Map, lmpc, max_steer):
        
        self.world = world
        self.vehicle = vehicle
        
        self.frame = None
        self._queues = []
        self.sensors = []
        self.delta_seconds = 1.0 / fps
        self.lap = -1  
        self.maxLaps = maxLaps
        self.LapTime = 100.0
        self.Time = 0.0
        self.SimTime = 0.0
        self.deviation = 0.0
        self.lap_dat = np.zeros((11,2))###########################################
        self._tick = 0.0
        self.dist = 0.0
        self.del_dist = 0.01
        self.Flag = 0
        self.EndFlag = 0
        self.LapTrigger = 0
        self.End = 0
        self.Init_dist = 0.0
        self.dt = 0.05
        self.max_steer_rat = 1/max_steer
        self.deltaT = 0.001
        #Sensors
        self.imu = imu
        self.sensors.append(self.imu)
        self.lc_cam = lc_cam
        self.sensors.append(self.lc_cam)
        self.trail_cam = trail_cam
        self.sensors.append(self.trail_cam)
        self.gnss = gnss
        self.sensors.append(self.gnss)
        self._settings = None
        self.X = np.array([0., 0., 0., 0., 0., 0.], dtype=float)
        self.X_glob = np.array([0., 0., 0., 0., 0., 0.], dtype=float)
        #Previously reset
        self.actor_list=[]
        self.track_dat = np.load('pnt_curv.npy')
        self.Map = Map
        self.lmpc = lmpc
        # # if not np.load('pnt_curv.npy'):
        # if self.Map.StartComputation() != True:
        #     print('RoadBlock!')
        #     self.End = 1
        # # else:
        # #     curv_dat = np.load('pnt_curv.npy')
        # #     self.Map.Update(curv_dat)
        
        self.actor_list.append(self.vehicle)         
        self.previous = self.vehicle.get_location()
        
    def __enter__(self):
        self._settings = self.world.get_settings()
        self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))
        
        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            # q.put(register_event)
            self._queues.append(q)
        
        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            # sensor.listen()
            make_queue(sensor.listen)
        return self
    
    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data
    
        
    
    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data
    
    def lap_record(self, snap, gyro, deviation, epsi, lmpc):
        """
        Attribute:
            lap - update current lap
            _laptime - update lap times
        clock: current time
        _tick: current time
        last_tick: save last lap trigger time
        """
        self.lmpc = lmpc
        self.LapTrigger = 0
        timestamp = snap.timestamp
        dt = timestamp.delta_seconds
        transform = self.vehicle.get_transform()
        location = transform.location
        velocities = self.vehicle.get_velocity()
        deviation = deviation
        epsi = math.radians(epsi) #in Degrees
        vx = velocities.x
        vy = velocities.y
        wz = gyro[2]
        
        self.del_dist = location.distance(self.previous)
        #Add distance travelled between spawn point and the finish line to get 
        # actual distance in PointandTangent dataset
        self.dist = self.dist + self.del_dist + self.Init_dist 
        self.SimTime = self.SimTime + dt
        self.previous = location
        
        self.X = [vx, vy, wz, epsi, self.dist, deviation]
        self.X_glob = [vx, vy, wz, math.radians(transform.rotation.yaw), location.x, location.y]
            
        #Records Laptime and updates number of laps
        
        
        # #Lap Action
        if self.lap<0:
            self.Init_dist = self.Init_dist + self.del_dist
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.20, steer=0.0))
            
        else:            
            #For 1st Lap or while lap count is zero
            #Runs PID
            if self.lap == 0: 
                new_x, new_x_glob = self.lmpc.PIDcont(self.X, self.X_glob, self.LapTime, self.SimTime)
            
            #For 2nd lap
            #Runs LTI controller 
            elif self.lap== 1:
                if self.EndFlag == 1:
                    new_x, new_x_glob = self.lmpc.PIDcont(self.X, self.X_glob, self.LapTime, self.SimTime, EndFlag = self.EndFlag)
                    self.EndFlag = 0

                new_x, new_x_glob = self.lmpc.LTIcont(self.X, self.X_glob, self.LapTime, self.SimTime)
            
            #For 3rd Lap
            #Runs PID controller
            elif self.lap == 2:
                if self.EndFlag == 1:
                    new_x, new_x_glob = self.lmpc.LTIcont(self.X, self.X_glob, self.LapTime, self.SimTime, EndFlag = self.EndFlag)
                    self.EndFlag = 0
                new_x, new_x_glob = self.lmpc.PIDcont(self.X, self.X_glob, self.LapTime)
            
            #For 4th Lap
            #Runs LTV controller
            elif self.lap == 3:
                if self.EndFlag == 1:
                    new_x, new_x_glob = self.lmpc.PIDcont(self.X, self.X_glob, self.LapTime, self.SimTime, EndFlag = self.EndFlag)
                    self.EndFlag = 0
                new_x, new_x_glob = self.lmpc.LTVcont(self.X, self.X_glob, self.LapTime, self.SimTime)
            
            #Run LMPC after first 4 laps
            else:
                if self.EndFlag == 1:
                    new_x, new_x_glob = self.lmpc.LTVcont(self.X, self.X_glob, self.LapTime, self.SimTime, EndFlag = self.EndFlag)
                    self.EndFlag = 0
                new_x, new_x_glob = self.lmpc.LMPCcont(self.X, self.X_glob, self.LapTime, self.SimTime, LapTrigger = self.LapTrigger)
                self.LapTrigger = 0 #Is 1 when a lap ends. Saves the lap trajectory in LMPC Controller. Then reset it to 0.
            
            
        
        
        if location.x>=-5.000 and location.x<=-1.000 and location.y>=-73.270 and location.y<=-71.570:           
            print('Lap flag')
            if self.lap < 0:
                print('zero lap over!')
                self.lap = self.lap + 1
                self.LapTrigger = 1
                self._tick = self.SimTime
            elif self.SimTime-self._tick>15:
                print('formal lap tick!')
                self.lap += 1
                self.LapTime = self.SimTime - self._tick
                self._tick = self.SimTime
                self.EndFlag = 1
                self.LapTrigger = 1
                self.dist = 0
                self.SimTime = 0.0
            
        
        if self.lap == self.maxLaps:
            self.lmpc.LMPCcont(self.X, self.X_glob, self.LapTime, self.SimTime, EndFlag = 1)
            self.End = 1
         
        if self.lap<= 0:
            return self.End, self.lap, self.LapTrigger, self.lmpc, self.LapTime, self.X, self.X_glob, self.Init_dist
        else:
            return self.End, self.lap, self.LapTrigger, self.lmpc, self.LapTime, new_x, new_x_glob, self.Init_dist
    def __exit__(self, *args, **kwargs):
        for actor in self.actor_list:
            actor.destroy()
        for sensor in self.sensors:
            sensor.destroy()
        self.world.apply_settings(self._settings)

class Track:
    def __init__(self, vehicle, map):
        self.width = 0.0
        self.previous = vehicle.get_location()
        self.dat = np.zeros((1, 4))
        self.dat[0, 0] = self.previous.x
        self.dat[0, 1] = self.previous.y

        self.center_prev = self.previous
        
        self.tangent = 0.5
        self.map = map
    
    def PosnLog(self, location, yaw, deviation, epsi, del_dist, LapTrigger):
        #Current vehicle coordinate, left distance, right distance, deviation, compute tangent#
        # psi = math.atan((current.y - self.previous.y)/(current.x - self.previous.x)) #world reference frame
        self.map.create(self.dat, LapTrigger)
        
        psi = yaw
        current = location
        epsi = epsi
        deviation = deviation
        s = del_dist * math.cos(epsi)
        self.tangent = psi - epsi
        
        center_x = self.center_prev.x + s * math.cos(math.radians(self.tangent))
        center_y = self.center_prev.y + s * math.sin(math.radians(self.tangent))
        
        self.previous = current
        self.center_prev.x = center_x
        self.center_prev.y = center_y
        
        self.dat[0, 0] = center_x
        self.dat[0, 1] = center_y
        self.dat[0, 2] = self.tangent
        self.dat[0, 3] = s
        
        return self.tangent, center_x, center_y
    def done(self):
        return self.map
            
class IMUSensor(object):
    def __init__(self, imu):
        self.accel = (0.0,0.0,0.0)
        self.gyro = (0.0,0.0,0.0)
        self.imu = imu
        
    # def listen(self):
    #     weak_self=weakref.ref(self)
    #     self.imu.listen(lambda sensor_data: IMUSensor.IMUData(weak_self, sensor_data))
        
    # @staticmethod
    # def IMUData(, sensor_data):
    #     self = weak_self()
    #     if not self:
    #         return
        
    def Data(self, sensor_data):
        self.accel = (sensor_data.accelerometer.x, sensor_data.accelerometer.y, sensor_data.accelerometer.z)
        self.gyro = (sensor_data.gyroscope.x,sensor_data.gyroscope.y,sensor_data.gyroscope.z)
        return self.accel, self.gyro

class GNSSSensor(object):
    def __init__(self, gnss):
        self.gnss = None
        self.lat = 0.0
        self.long = 0.0
        self.gnss = gnss
       
        
        
    # def listen(self):
    #     weak_self = weakref.ref(self)
    #     self.gnss.listen(lambda data: GNSSSensor.GPSposn(weak_self,data))
    
    # def GPSposn(weak_self, data):
    #     self = weak_self()
    #     if not self:
    #         return
        
        
    def Data(self, data):
        self.lat = data.latitude
        self.long = data.longitude
        return self.lat, self.long

class CamManager(object):
    def __init__(self, cam_height, cam_angle, blind, halfwidth):
        self.left_dist = None
        self.left_dist0 = halfwidth
        self.right_dist = None
        self.right_dist0 = None
        self.left_angle = None
        self.right_angle = None
        self.epsi = None
        self.epsi0 = 0.0
        self.dist = 0.0
        self.deviation = None
        self.fc_width = 1280
        self.fc_height = 720
        self.image = None
        self.count = 0
        self.cam_height = cam_height
        self.cam_angle = abs(math.radians(cam_angle))
        self.blind = blind
        self.reference_dist = (self.cam_height*abs(1/math.tan(self.cam_angle)))
        
    def Data(self, angle, dist):
        #Deviation Correction
        # if self.deviation == None:
        #     if self.left_dist != None and self.right_dist != None:
        #         self.deviation = self.left_dist - self.right_dist
        #     elif self.left_dist != None and self.right_dist == None:
        #         self.deviation = self.left_dist - self.left_dist0
        #     elif self.left_dist == None and self.right_dist!= None:
        #         self.deviation = self.right_dist0 - self.right_dist
         
        #Just using left camera to find deviation (subject to the type of the track)
        #also subject to the reliability of the left camera
        # if self.deviation == None:
        self.epsi = angle
        self.left_dist = dist
               
        if self.count == 0 and self.left_dist != None:
            # self.right_dist0 = self.right_dist
            self.left_dist0 = self.left_dist
            self.count = 1
        if self.epsi == None and self.left_dist == None:
            self.epsi = 0.0
            self.left_dist = self.left_dist0
        self.deviation = self.left_dist - self.left_dist0
        self.right_dist = self.left_dist - self.deviation 
        # print(self.deviation, self.epsi)
        #Angle correction
        # if self.left_angle == None and self.right_angle == None:
        #     self.epsi = 0.0
        # elif self.left_angle != None and self.right_angle == None:
        #     self.epsi = self.left_angle
        # elif self.left_angle == None and self.right_angle != None:
        #     self.epsi = self.right_angle #inversion
        # else:
        #     self.epsi = (self.right_angle + self.left_angle) / 2.0 #Mean 
        
        return self.epsi, self.deviation

    
    def process_img(image):
        """
        Process the image captures in the CARLA sim. The CARLA returns an
        1D array of BGR data which needs to be rearranged to be viewable.
        """
        i = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        i = np.reshape(i,(image.height, image.width, 4))
        i = i[:, :, :3]
        np.save('image',i)
        return i

    
    def lateral_imgproc(self, data):
        image = CamManager.process_img(data)
        try:
            image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
            image=image[0:360,220:420]
            img_size=(image.shape[1],image.shape[0])
            #Perspective Transform
            src = np.float32([[200,115],#351,180
                              [200,360],#420,360
                              [0,360],#220,360
                              [0,115]])#289,180
            
            dst = np.float32([[200,0],
                              [200,360],
                              [0,360],
                              [0,0]])
                    
            M = cv2.getPerspectiveTransform(src,dst)
            warped = cv2.warpPerspective(image,M,img_size,flags=cv2.INTER_LINEAR)
            
            # boundary=cv2.inRange(warped,(30,30,30),(60,60,60))    
            
            #Thresholding
            thresh = cv2.threshold(image, 190, 225, cv2.THRESH_BINARY)[1]
            thresh = cv2.erode(thresh, None, iterations=4)
            thresh = cv2.dilate(thresh, None, iterations=10)
            
            contours,hierarchy=cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            # cv2.drawContours(roi,contours,-1,(0,255,0),2)
           
            #Measure Angle
            miny_f=360
            miny_l=360
            
            #this step is crucial if there is only one contour else range(0,0) will fail
            
            for i in range(0,len(contours)):
                for j in range(0,len(contours[i])):
                    for maxx in range(170,200):
                        if contours[i][j][0][0]==maxx and contours[i][j][0][1]<=miny_l:
                            xl=contours[i][j][0][0]
                            yl=contours[i][j][0][1]
                            miny_l=yl
                    
                    for minx in range(0,30):
                        if contours[i][j][0][0]==minx and contours[i][j][0][1]<=miny_f:
                            xf=contours[i][j][0][0]
                            yf=contours[i][j][0][1]
                            miny_f=yf
        
            angle = math.degrees(math.atan(((360-yl)-(360-yf))/(xl-xf)))
            
            #Measure distance
            
            dist = (self.reference_dist-self.blind) * (360-((yl+yf)/2)) / 264
            corrected_dist = dist * math.cos(math.radians(angle)) + self.blind
            
            return angle, corrected_dist
        
        except:
            print("img fail \n")
            return None, None
     
 

def draw_image(surface, image, blend=False):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))

def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def main():
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    settings = world.get_settings()
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    # pygame.init()

    # display = pygame.display.set_mode(
    #     (800, 600),
    #     pygame.HWSURFACE | pygame.DOUBLEBUF)
    # font = get_font()
    # clock = pygame.time.Clock()
    # spectator = world.get_spectator()
    maxLaps = 10
    fps = 20
    Flag = 0
    lat_width = 640
    lat_height = 360
    cam_height = 0.6 #ought to be 0.5m
    cam_angle = -10
    cam_offset = 1.06
    blind_zone = 0.4 + cam_offset #n meters
    halfwidth = 3.7
    count = 0
    store = None
    dt = 0.05
    
    
    try:
         blueprint_library = world.get_blueprint_library()
         bp = blueprint_library.filter('model3')[0]
         # spawn_point = random.choice(world.get_map().get_spawn_points())
         loc = carla.Location(x=-2.99717595 , y=-87.820107 , z=0.30)
         rot = carla.Rotation(pitch = 0.0, yaw = 90.47937, roll = 0.0)
         tran = carla.Transform(loc, rot)
         vehicle = world.spawn_actor(bp, tran)
         # vehicle.set_simulate_physics(False)
         
         #Camera Sensor
         bp = world.get_blueprint_library().find('sensor.camera.rgb')
         bp.set_attribute('image_size_x', f'{lat_width}')
         bp.set_attribute('image_size_y', f'{lat_height}')
         bp.set_attribute('fov', '110')
         lc_cam = world.spawn_actor(bp, carla.Transform(carla.Location(y=-cam_offset, z=cam_height),carla.Rotation(pitch=cam_angle, yaw=-90.0)), attach_to=vehicle)
         
         #TrailingCamera
         trail_cam = world.spawn_actor(
         blueprint_library.find('sensor.camera.rgb'),
         carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
         attach_to=vehicle)
         
         #IMU Sensor
         bp = world.get_blueprint_library().find('sensor.other.imu')
         imu = world.spawn_actor(bp, carla.Transform(), attach_to=vehicle)
         
         #GNSS Sensor
         bp = world.get_blueprint_library().find('sensor.other.gnss')
         gnss = world.spawn_actor(bp,carla.Transform(),attach_to=vehicle)
         
         #Teleport vehicle to new location
         
         
         #Initialize sensor objects
         IMU = IMUSensor(imu)
         CAM = CamManager(cam_height, cam_angle, blind_zone, halfwidth)
         GPS = GNSSSensor(gnss)
         track_dat = np.load('pnt_curv.npy')
         map = Map(3.40, track_dat)
         phys = vehicle.get_physics_control()
         lmpc = RaceLMPC(dt, 60, maxLaps, map, phys)
         
         max_steer = phys.wheels[0].max_steer_angle
         world.tick()
         count = 0 
         lap_dat = np.zeros((30000, 9))
         # world_snapshot = world.wait_for_tick()
         # actor_snapshot = world_snapshot.find(vehicle.id)

         ## Set spectator at given transform (vehicle transform)
         # spectator.set_transform(actor_snapshot.get_transform())
         
         # time.sleep(2)
         with commander(world, maxLaps, fps, vehicle, imu, lc_cam, trail_cam, gnss, Map, lmpc, max_steer) as controller:
            while Flag == 0:
                # clock.tick()
                
                snap, imu_dat, cam_dat, t_dat, gps_dat = controller.tick(timeout=5.0)
                accel, gyro = IMU.Data(imu_dat)
                lat, long = GPS.Data(gps_dat)
                #Processing for Image data
                angle, dist = CAM.lateral_imgproc(cam_dat)
                epsi, deviation = CAM.Data(angle, dist) 
                
                Flag, lap, LapTrigger, lmpc, LapTime, x, x_glob, predist = controller.lap_record(snap, gyro, deviation, epsi, lmpc)
                lap_dat[count,0:1] = x[0:1]
                lap_dat[count, 2] = x[4]
                lap_dat[count, 3:4] = x_glob[4:5]
                lap_dat[count, 5] = accel[1]
                lap_dat[count, 6] = LapTime
                lap_dat[count, 7] = lap
                lap_dat[count, 8] = predist
                if lap > 0:
                    #Apply new control
                    new_v = carla.Vector3D(x = x[0]*3.6, y = x[1]*3.6)
                    new_om = carla.Vector3D(z = x[2])
                    vehicle.set_velocity(new_v)
                    vehicle.set_angular_velocity(new_om)
                    
                if LapTrigger == 1:
                    if lap == 0:  
                        lmpc.PIDinit(x)
                        
                    elif lap==1:
                        lmpc.LTIinit(x, LapTime)
                        
                    elif lap==2:
                        lmpc.PIDinit(x)
                        
                    elif lap == 3:
                        lmpc.LTVinit(x, LapTime)
                        
                    elif lap == 4:
                        lmpc.LMPCinit(x, LapTime)
                  
                count += 1
                # fpsn = round(1.0 / snap.timestamp.delta_seconds)
                #   # Draw the display.
                # draw_image(display, t_dat)
                # display.blit(
                #     font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)),
                #     (8, 10))
                # display.blit(
                #     font.render('% 5d FPS (simulated)' % fpsn, True, (255, 255, 255)),
                #     (8, 28))
                # pygame.display.flip()
                
    
    finally:
        vehicle.destroy()
        print('We are Done!')
        lap_dat = np.delete(lap_dat, slice(count-1, 29999), axis=0)
        np.save('Lap_data',lap_dat)
        pygame.quit()

if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')     
        
        