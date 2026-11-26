import random
import math
import numpy as np
import cv2
import logging
from enum import Enum
import pixy2sim, numpy as np

""" ©2018 Tudor Popovici"""
""" module for physical engine, modelling car dynamic mechanics and camera characteristics"""
""" depends on driving module for driver observe and control methods"""

LATERAL_TRANSLATE_M = 0
LOCAL_ROTATE_DEG = 0

class Camera:
    """"Camera characteristics descriptor, currently matching approximatively pixy 2"""

    def __init__(self, height_m=0.4, rotation_deg=0, inclination_deg=55+12
                 , fwd_displacement_relative_to_rear_axle_m=0.25
                 , hfov_deg=80, aspect_ratio=636.0/418.0):
        self.hfov_deg = hfov_deg
        self.rotation_deg = rotation_deg
        self.aspect_ratio = aspect_ratio
        self.height_m = height_m
        self.inclination_deg = inclination_deg
        self.fwd_displacement_m = fwd_displacement_relative_to_rear_axle_m
        self.pixy_in = None
        self.raster = None  # raw camera perception image
        self.raster_out = None  # pixy2 internal processed camera image
        self.pixy_vector, self.pixy_lines, self.pixy_intes = None, [], []
        self.pixy_out = np.array([128] * (318 * 208), dtype=np.uint8).reshape((208, 318))

        # vector callback (called by the simulator during process_frame function execution to signal vector detection)
        def pixy2_vector_callback(a, b, c, d, e):
            self.pixy_vector = [a, *map(lambda x: x * 4, [b, c, d, e])]
            # print("VECT:", self.pixy_vector)
            return

        pixy2sim.set_vector_callback(pixy2_vector_callback)

        # line callback (called by the simulator for each detected line during during process_frame function execution)
        def pix2_line_callback(id, x0, y0, x1, y1):
            self.pixy_lines.append([id, *map(lambda x: x * 4, [x0, y0, x1, y1])])
            # print("LINE:", self.pixy_lines[-1])
            return

        pixy2sim.set_line_callback(pix2_line_callback)

        # intersection callback (called by the simulator for each detected intersection during process_frame function execution)
        def pixy2_intersection_callback(segment_count, x, y):
            self.pixy_intes.append([segment_count, x * 4, y * 4])
            # print("INTE:", pixy_intes[-1])
            return

        pixy2sim.set_intersection_callback(pixy2_intersection_callback)
    def take_snapshot(self, raster_raw, timestamp):
        self.pixy_in = raster_raw
        # clean previous camera scan results
        self.pixy_vector, self.pixy_lines, self.pixy_intes = None, [], []
        # get new camera scan results (uses the vector, lines and intersection callbacks to populate the result lists)
        pixy2sim.process_frame(raster_raw, int(timestamp * 1e6), self.pixy_out)


class CalibrationPattern:
    """
	©2018 Tudor Popovici

	Camera calibration routines based on placing in front of the car, properly aligned,
	a simple pattern composed of two straight lines parallel to the wheel axles,
	one close by and another distant, both centered to the longitudinal axis of the car

	Helps in easily producing a top orthogonal projection of the data observed by the camera
	that is dimensional accurate and can be used for 2D measurements after calibration"""

    default_lightness_threshold = 60  # very important for transforming bgr images to black and white
    default_saturation_threshold = 40  # very important for transforming bgr images to black and white

    def __init__(self, ortho_map_width_m, ortho_map_height_m, ortho_map_bottom_offset_m=0.0
                 , distance_rear_axle_to_closest_calibration_line_m=.60 - 0.3
                 , closest_calibration_line_length_m=.60
                 , distance_from_rear_axle_to_farthest_calibration_line_m=1.6-0.6
                 , farthest_calibration_line_length_m=0.897
                 , scale_px_per_m=100
                 , closest_calibration_line_drawing_thickness_m=0.05
                 , farthest_calibration_line_drawing_thickness_m=0.05
                 , pattern_height=0.001):
        self.scale_px_per_m = scale_px_per_m
        self.ortho_map_width_m = ortho_map_width_m
        self.ortho_map_height_m = ortho_map_height_m
        self.ortho_map_bottom_offset_m = ortho_map_bottom_offset_m
        self.width_px = self.m_to_px(ortho_map_width_m)
        self.height_px = self.m_to_px(ortho_map_height_m)
        self.bottom_px = self.m_to_px(ortho_map_bottom_offset_m)
        self.z = pattern_height
        self.d = distance_rear_axle_to_closest_calibration_line_m
        self.l = closest_calibration_line_length_m
        self.D = distance_from_rear_axle_to_farthest_calibration_line_m
        self.L = farthest_calibration_line_length_m
        self.w = closest_calibration_line_drawing_thickness_m
        self.W = farthest_calibration_line_drawing_thickness_m
        self.Mx = np.empty((self.height_px, self.width_px), np.float32)
        self.My = np.empty((self.height_px, self.width_px), np.float32)
        self.M = None
        self.mask = None

    @staticmethod
    def load(dump):
        cp = CalibrationPattern(ortho_map_width_m=dump[3][0], ortho_map_height_m=dump[3][1], scale_px_per_m=dump[1])
        cp.M = dump[0]
        return cp

    def dump(self):
        return self.M, self.scale_px_per_m, (self.width_px, self.height_px)

    @staticmethod
    def bgr_to_bw(raster, lit_th=None, sat_th=None):
        if lit_th is None:
            lit_th = CalibrationPattern.default_lightness_threshold
        if sat_th is None:
            sat_th = CalibrationPattern.default_saturation_threshold
        raster = cv2.cvtColor(raster, cv2.COLOR_BGR2HLS)
        lower_white = np.array([0, lit_th, 0], dtype=np.uint8)
        upper_white = np.array([255, 255, sat_th], dtype=np.uint8)
        return cv2.inRange(raster, lower_white, upper_white)

    @staticmethod
    def flood_search(raster, x, y, th=128):
        '''serches for left most and right most dark pixels starting from the given point'''
        '''the purpose is to find the endpoints of a horizontal line'''
        q = [(x, y)]
        l, r = x, x
        yl, yr = 0, 0
        t = raster.shape[0] - 1
        while len(q) > 0:
            x, y = q.pop(0)
            if x is not None and y is not None and 0 <= x < raster.shape[1] and 0 <= y < raster.shape[0] and raster[y][
                x] < th:
                raster[y][x] = th  # color current pixel
                q.append((x + 1, y))
                q.append((x - 1, y))
                q.append((x, y + 1))
                q.append((x, y - 1))
                q.append((x + 1, y + 1))
                q.append((x - 1, y + 1))
                q.append((x - 1, y + 1))
                q.append((x + 1, y - 1))
                # remember the coordinates of the left most pixel
                if x < l:
                    l = x
                    yl = y
                # remember the coordinates of the right most pixel
                if x > r:
                    r = x
                    yr = y
                # remember the y coordinate of the highest pixel to know where to start looking from
                if y < t:
                    t = y
        return l, yl, r, yr, t

    @staticmethod
    def get_lowest_point_on_vertical_x_matching_lambda(raster, x, y=0, cond=lambda x: x < 128):
        for Y in range(y, 0, -1):
            if cond(raster[Y][x]):
                return Y
        return None

    def cam_coord_to_orto_px_float(self, M, x, y):
        d = M[2, 0] * x + M[2, 1] * y + M[2, 2]
        return (M[0, 0] * x + M[0, 1] * y + M[0, 2]) / d, (M[1, 0] * x + M[1, 1] * y + M[1, 2]) / d

    def cam_coord_to_orto_int(self, M, x, y):
        return tuple(map(lambda z: int(round(z)), self.cam_coord_to_orto_px_float(M, x, y)))

    def cam_coord_to_orto_m(self, x, y):
        """
        Transform pixy2 camera coordinates (scaled up by 4)
            to coordinates relative to the midpoint of the rear axle
            where X axis is along the rear axle toward the right side wheel
            and Y axis is ahead, to the midpoint of the forward axle
        :param x: coordinates reported by pixy2 up scaled by 4
        :param y: coordinates reported by pixy2 up scaled by 4
        :return: return the point in the horizontal plane relative to the midpoint of the rear axle
        """
        M = self.M
        d = M[2, 0] * x + M[2, 1] * y + M[2, 2]
        return (round(((M[0, 0] * x + M[0, 1] * y + M[0, 2]) / d-self.width_px/2.0)/self.scale_px_per_m,3),
                round((self.height_px-(M[1, 0] * x + M[1, 1] * y + M[1, 2]) / d)/self.scale_px_per_m + self.ortho_map_bottom_offset_m, 3))


    def calibrate(self, raster):
        h, w = raster.shape[:2]
        w2, h2 = divmod(w, 2)[0], divmod(h, 2)[0]

        if len(raster.shape) > 2:
            raster = CalibrationPattern.bgr_to_bw(raster)
        # TrackRefs.show_image(raster)

        # TrackRefs.show_image(raster)
        #cv2.imshow('filtered_input', raster)
        #cv2.waitKey(delay=1000)
        #cv2.waitKey()
        m = CalibrationPattern.get_lowest_point_on_vertical_x_matching_lambda(raster, w2, h - int(h2 / 4))
        if m is None:
            logging.getLogger('sim').critical(
                'Lighting or image filtering issues detected when searching for close calibration line.')

        l, yl, r, yr, M = self.flood_search(raster, w2, m)
        M = None if M+2>=raster.shape[0] else CalibrationPattern.get_lowest_point_on_vertical_x_matching_lambda(raster, w2, M+2)
        if M is None:
            logging.getLogger('sim').critical(
                'Lighting or image filtering issues detected when searching for far calibration line.')
        L, yL, R, yR, M = self.flood_search(raster, w2, M)

        dl = divmod(self.m_to_px(self.l), 2)[0]
        dL = divmod(self.m_to_px(self.L), 2)[0]
        yb = self.m_to_px(self.d)
        yt = self.m_to_px(self.D)
        xc = divmod(self.width_px, 2)[0]

        tl = (L, yL)
        Tl = (xc - dL, self.height_px + self.bottom_px - yt)
        tr = (R, yR)
        Tr = (xc + dL, self.height_px + self.bottom_px - yt)
        bl = (l, yl)
        Bl = (xc - dl, self.height_px + self.bottom_px - yb)
        br = (r, yr)
        Br = (xc + dl, self.height_px + self.bottom_px - yb)
        pts1 = np.float32([tl, tr, bl, br])
        pts2 = np.float32([Tl, Tr, Bl, Br])
        self.M = cv2.getPerspectiveTransform(pts1, pts2)
        M = cv2.getPerspectiveTransform(pts2, pts1)
        # prep ortho mapping optimization
        for x in range(self.width_px):
            for y in range(self.height_px):
                self.Mx[y, x], self.My[y, x] = self.cam_coord_to_orto_px_float(M, x, y)
        mask = np.ones((h, w), dtype=np.uint8) * 255
        # print('BEFORE self.mask dtype =', mask.dtype)
        # print('BEFORE self.mask shape =', mask.shape)
        self.mask = self.perspective_to_ortho(mask)
        # print('AFTER self.mask dtype =', self.mask.dtype)
        # print('AFTER self.mask shape =', self.mask.shape)
        _, self.mask = cv2.threshold(self.mask, 254, 255, cv2.THRESH_BINARY)
        ksize = 3
        self.mask = cv2.erode(self.mask, np.ones((ksize, ksize), np.uint8), iterations=1)
        self.mask = self.mask.astype(np.uint8)
        return raster

    def perspective_to_ortho(self, img):
        import time
        # rpi = cv2.warpPerspective(img, self.M, (self.width_px, self.height_px))
        rpi = cv2.remap(img, self.Mx, self.My, cv2.INTER_AREA)
        return rpi

    def trim_border(self, image):
        # print('rpi dtype =', image.dtype)
        # print('rpi shape =', image.shape)
        # print('self.mask dtype =', self.mask.dtype)
        # print('self.mask shape =', self.mask.shape)
        image = image.astype(np.uint8)
        image = cv2.bitwise_and(image, self.mask)
        return image

    def edge_detect_then_trim(self, image, cannyLower=0, cannyUpper=500):  # send gray image as parameter
        image = self.perspective_to_ortho(image)
        image = cv2.Canny(image, cannyLower, cannyUpper)
        image = self.trim_border(image)
        return image

    def m_to_px(self, m):
        return int(m * self.scale_px_per_m)

    def px_to_m(self, px, decimals=2):
        if px is None:
            return None
        return round(px * 1.0 / self.scale_px_per_m, decimals)

    def analyse_snapshot(self, snapshot, d_m=1.0):
        """look at a specified distance in the snapshot and detect the left most and right most track cue"""
        d_px = self.m_to_px(d_m - self.ortho_map_bottom_offset_m)
        if d_px > snapshot.shape[0]:
            d_px = snapshot.shape[0]
        r = l = None
        offset = snapshot.shape[1] / 2
        for i in range(snapshot.shape[1]):
            if snapshot[d_px][i] > 128:
                if l is None:
                    l = i - offset
                r = i - offset
        return 0, self.px_to_m(l), self.px_to_m(r)


class DynamicPhysicalModel:
    """dynamic physical model usable in very simple discreet time simulations"""

    def __init__(self, parent=None):
        """initialize internal time reference"""
        self.components = []
        self.t = 0.0
        if parent is not None:
            parent.register_component(self)

    def step(self, dt_s):
        """
        Execute one simulation step of time length dt_s by updating all the system states accordingly
        :param dt_s: the amounth of time in seconds to simulate further
        :return: True if any of the components require the simulation to continue to a new step
        When simulation should complete, all components should report False
        """
        go = False
        for c in self.components:
            # keep going while at least one car has something to do
            go = c.step(c.t - self.t + dt_s) or go
        self.t += dt_s
        return go

    def reset(self):
        self.t = 0.0
        for c in self.components:
            c.reset()

    def register_component(self, child):
        self.components.append(child)


class ProportionalIntegrativeDerivativeRegulator(DynamicPhysicalModel):
    """PID controller with basic saturation and integrative antiwindup with clamping"""

    def __init__(self, kp, ki=0.0, kd=0.0, output_min=None, output_max=None):
        super(ProportionalIntegrativeDerivativeRegulator, self).__init__()
        self.output_min = output_min
        self.output_max = output_max
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.err = 0.0
        self.last_err = 0.0
        self.accumulated_err = 0.0
        self.control_output = self.saturate(0.0)

    def saturate(self, val):
        if None in (self.output_min, self.output_max):
            return val
        if val < self.output_min:
            return self.output_min
        if val > self.output_max:
            return self.output_max
        return val

    def set_measured_error(self, error):
        """normally called before step method (especialy when kd is not 0"""
        self.err = error

    def step(self, dt_s):
        """calculates the new control value based on measured error, its accumulation and variation in time"""
        self.err_derivate = (self.err - self.last_err) / dt_s
        self.control_output = self.saturate(self.kp * self.err
                                            + self.ki * self.accumulated_err
                                            + self.kd * self.err_derivate)
        self.accumulated_err = self.saturate(self.accumulated_err + (self.last_err + self.err) * dt_s / 2)
        self.last_err = self.err
        super(ProportionalIntegrativeDerivativeRegulator, self).step(dt_s)

    def get_control_output(self):
        return self.control_output


class Car(DynamicPhysicalModel):
    """generic car interface"""

    def __init__(self, parent=None):
        super(Car, self).__init__(parent)
        self.calibration_pattern = CalibrationPattern(ortho_map_width_m=3.0, ortho_map_height_m=2.0, scale_px_per_m=100,
                                                      ortho_map_bottom_offset_m=0.01)

    def set_course_reference(self, speed_m, curvature_1_per_m):
        pass

    def get_course_reference(self):
        pass

    def get_course_current(self):
        pass

class Driver:
    def __init__(self, start_info):
        self.start_info = list(start_info)
        self.start_info[2] = math.radians(self.start_info[2])
        self.log = logging.getLogger('sim')

    def prep_car(self, car):
        car.reset()
        car.x, car.y, car.a = self.start_info
        pass

    def observe(self, car,  line_info=None):
        return None

    def control(self, car):
        """
        Called on each mechanical simulation cycle to update the mechanical refereces of the drive car
        :param car: the car subject to reference adjustment
        :return: True if simulation steps for the car still need to be executed
        """
        return True

    def finished(self):
        """
        Called by the simulation environment when the simulation is complete to allow for custom actions
        :return: nothing
        """
        pass


class DriverBlind(Driver):
    class DriverState(Enum):
        WaitingStart = 1  # at simulation start
        Refreshing = 2
        Running = 3  # normal operation
        Stopping = 4  # decelerating to stop
        Stopped = 6  # stopped
        Finishing = 7  # decelarate to finish
        Finished = 0  # finished
    def __init__(self, track_ref, max_speed):
        self.track = track_ref
        super(DriverBlind, self).__init__(self.track.start_info)
        self.max_speed = max_speed
        self.wait_time = 0.0
        self.wait_until = 0.0
        self.steering_planned = 0
        self.last_segment = None
        self.state = DriverBlind.DriverState.WaitingStart

    def prep_car(self, car):
        self.log.info(' '.join(map(str, ('car', car.cid, 'running on trajectory', self.track.start_info[0:2],
                                         round(self.track.start_info[2], 1)))))
        super(DriverBlind, self).prep_car(car)
        car.set_course_reference(0.0, 0.0)
        car.motor.set_state(0.0)
        car.steering.set_state(0.0)
        self.state = DriverBlind.DriverState.WaitingStart
        if car.steering.radius_min > 0.27 or car.steering.steer_time > 0.01:
            print("\nERROR: To use DriverBlind implementation the SteeringAckerman needs to be configured with at most 0.27 radius_min and at most 0.01 steering time! Fix it and run again!")
            exit(0)

    def control(self, car):
        segment = self.track.get_segment(car.distance)
        if DriverBlind.DriverState.WaitingStart == self.state:
            self.state = DriverBlind.DriverState.Running
            car.set_course_reference(self.max_speed, None)
        if segment is not None:
            if segment != self.last_segment:
                if self.state != DriverBlind.DriverState.Running:
                    self.log.critical('State should refresh only during Running state... an action was not completed')
                    return False
                else:
                    pass
                # self.log.reset_last_error_location()

                self.last_segment = segment
                self.state = DriverBlind.DriverState.Refreshing
        else:
            if DriverBlind.DriverState.Running == self.state:
                self.state = DriverBlind.DriverState.Finishing
                # prepare to stop
                self.steering_planned = 0.0
                car.set_course_reference(0.0, self.steering_planned)

        if DriverBlind.DriverState.Finishing == self.state and not (abs(car.motor.state) > 0):
            self.state = DriverBlind.DriverState.Finished
            self.log.info(
                ' '.join(map(str, ('finish', car.cid, car.distance, 'm', car.t, 's', 'x', car.x, 'y', car.y))))

        if DriverBlind.DriverState.Refreshing == self.state:
            self.steering_planned = segment.curvature
            car.set_course_reference(None, self.steering_planned)  # keep steering ref according to track ref
            self.state = DriverBlind.DriverState.Running
            if segment.stop_time > 0:
                self.state = DriverBlind.DriverState.Stopping
                self.wait_time = segment.stop_time
                car.set_course_reference(0, None)

        if DriverBlind.DriverState.Stopping == self.state and not (abs(car.motor.state) > 0):
            self.wait_until = car.t + self.wait_time
            self.state = DriverBlind.DriverState.Stopped
        if DriverBlind.DriverState.Stopped == self.state and self.wait_until <= car.t:
            # accelerate to full speed
            car.set_course_reference(self.max_speed, None)
            self.state = DriverBlind.DriverState.Running

        return DriverBlind.DriverState.Finished != self.state

    def observe(self, car, line_info=None):
        # snapshot = car.calibration_pattern.bgr_to_bw(snapshot)
        # car.set_course_reference(self.max_speed, None)
        return

    def finished(self):
        pass


class VirtualCar(Car):
    """Car model with Ackerman steering and fixed traction acceleration"""

    def __init__(self, parent, driver, cid='0', lateral_offset_m=0.0, rotation_offset_deg=0.0):
        super(VirtualCar, self).__init__(parent)
        self.cid = cid
        self.x, self.y, self.a = 0, 0, 0
        self.distance = 0.0
        self.motor = Traction(self, 0, 5, 6)
        self.steering = SteeringAckerman(self, 0.27+0.2, 0.1) # min turn radius is about 0.5m in reality...
        self.driver = driver
        self.driver.prep_car(self)
        self.lateral_offset_m = lateral_offset_m
        self.rotation_offset_deg = rotation_offset_deg
        self.camera = Camera()
        # self.camerar= Camera(inclination_deg=80, rotation_deg=-90)
        self.markers = []

        global LATERAL_TRANSLATE_M, LOCAL_ROTATE_DEG
        self.lateral_translate_m = LATERAL_TRANSLATE_M
        self.local_rotate_deg = LOCAL_ROTATE_DEG




    def set_course_reference(self, speed_m_per_s, curvature_1_per_m):
        """set reference values for speed and steering (None will not make changes to existign values)"""
        if speed_m_per_s is not None:
            self.motor.set_ref(speed_m_per_s)
        if curvature_1_per_m is not None:
            self.steering.set_ref(curvature_1_per_m)

    def get_course_reference(self):
        return self.motor.ref, self.steering.curvature_ref

    def get_course_current(self):
        return self.motor.state, self.steering.curvature_state

    def step(self, dt_s=0.001):
        """
        The simulation step is done after the input of the driver is requested
        The driver updates the components references and confirms if additional simulation steps are required
        :param dt_s: the amounth of time to simulate further in the current step after references are updated
        :return: True if the driver requires an aditional simulation step after this one
        """
        continue_sim = self.driver.control(self)
        self._move_ackerman(dt_s)
        return super(VirtualCar, self).step(dt_s) or continue_sim

    def _move_ackerman(self, dt_s):
        delta_distance = self.motor.state * dt_s
        self.distance += delta_distance
        c = self.steering.curvature_state
        delta_angle = delta_distance * c
        if abs(c) > 0:
            delta_distance = 2 * math.sin(delta_angle / 2) / c

        self.x += math.cos(self.a + delta_angle / 2) * delta_distance
        self.y += math.sin(self.a + delta_angle / 2) * delta_distance
        self.a += delta_angle


class Actuator(DynamicPhysicalModel):
    """" general first order state system approximation with constant acceleration,
    including support for maximum and minimum saturation of references and states"""

    def __init__(self, parent, min, max, max_ds_dt, ref_dispersion, var_dispersion=0.0):
        super(Actuator, self).__init__(parent)
        self.min = min
        self.max = max
        self.ref_dispersion = ref_dispersion
        self.var_dispersion = var_dispersion
        self.state = self.filter_min_max(0.0)
        self.ref = self.filter_min_max(0.0)
        self.max_ds_dt = max_ds_dt
        self.set_ref(0.0)

    def filter_min_max(self, val):
        if val < self.min:
            return self.min
        if val > self.max:
            return self.max
        return val

    def set_state(self, state):
        self.state = self.filter_min_max(state)

    def set_ref(self, ref):
        if ref is None:
            return
        nr = random.gauss(ref, self.ref_dispersion)
        self.ref = self.filter_min_max(nr)

    def step(self, dt_s):
        ds = self.state - self.ref
        ds_max = math.copysign(self.max_ds_dt * dt_s, ds)
        if abs(ds) > abs(ds_max):
            ds = ds_max
        self.set_state(self.state - ds)
        super(Actuator, self).step(dt_s)
        return True


class SteeringAckerman(Actuator):
    """ Steering state model"""
    """ Steering curvature (reciprocal of radius) is nonlinear compared to 
        to the steering angle which can be approximated linear to the servo motor position
        as such although the received reference represents curvature 
        the internal reference is the actual steering angle dictated by the servo motor
        for which a first order state system approximation is used.
        As a result the relevant trajectory information is given by the curvature_state
        while the pwm duty cycle can be approximated py the internal state representing the steering angle"""

    def __init__(self, parent, min_radius, steer_time, l=0.26, t=0.14, dispersion=0.0):
        self.length = l  # distance between front and rear axels
        self.tread = t  # distance between centers of the left and right tires
        self.steering_angle = 0.0  # angle of an imaginary middle front tire
        self.curvature_state = 0.0
        self.curvature_ref = 0.0
        self.radius_min = min_radius
        self.steer_time = steer_time
        c = 1.0 / min_radius
        super(SteeringAckerman, self).__init__(parent, self.curvature_to_steering_angle(-c)
                                               , self.curvature_to_steering_angle(c)
                                               , self.curvature_to_steering_angle(2 * c) / steer_time
                                               , dispersion)

    def curvature_to_steering_angle(self, c):
        return math.atan(c * self.length)

    def steering_angle_to_curvature(self, s):
        return math.tan(s) / self.length

    def set_ref(self, curvature_ref):
        if curvature_ref is None:
            return
        self.curvature_ref = curvature_ref
        steering_angle_ref = 0
        if 0 != self.curvature_ref:
            steering_angle_ref = self.curvature_to_steering_angle(self.curvature_ref)
        super(SteeringAckerman, self).set_ref(steering_angle_ref)

    def step(self, dt_s):
        super(SteeringAckerman, self).step(dt_s)
        self.curvature_state = self.steering_angle_to_curvature(self.state)
        return True


class Traction(Actuator):
    """ simple first order state system approximation for motor with constant acceleration
    good enough for constant loads e.g. only plane surfaces, no inertia changes"""

    # traction state is velocity
    def __init__(self, parent_system, v_min=0.0, v_max=5.0, a_max=2.0, dispersion=0.0):
        super(Traction, self).__init__(parent_system, v_min, v_max, a_max, dispersion)


class SegmentRef:
    def __init__(self, from_distance, to_distance, curvature, stop_time):
        self.from_distance = from_distance
        self.to_distance = to_distance
        self.curvature = curvature
        self.stop_time = stop_time


class TrackRef:
    def __init__(self, data):
        self.comment = data[0]
        self.start_info = data[1]
        self.segments = []
        self.length = 0
        for s in data[2:]:
            curvature = s[1]
            if 0 != s[1]:
                curvature = 1.0 / curvature
            stop_time = 0.0
            if 3 == len(s):
                stop_time = s[2]
            self.segments.append(SegmentRef(self.length, self.length + s[0], curvature, stop_time))
            self.length = self.segments[-1].to_distance

    def get_segment(self, at_distance):
        '''
        Returns the segment the car should run on based on the accumulated distance from start
        TODO: can be optimized for O(1) response time
        :param at_distance: how far car went from start
        :return: segment descriptor (course info)
        '''
        for segment in self.segments:
            if segment.from_distance <= at_distance < segment.to_distance:
                return segment
        return None

class DriverDizzy(Driver):
    """
    Dizzy Driver model (nothing fancy, a driver that needs glasses and has none,
                        so it mostly brakes the rules by cutting corners too much,
                        just a demo of reacting to the pixy2 reported lines,
                            by using the top-view transformed coordinates,
                             relative to the midpoint or rear axle, expressed in meters
                            or by using segment coordinates as reported by pixy2)
    Please note that the SteeringAckerman min_radius limit is configured in easy mode in VirtualCar class:
    small minimum turn radius and almost realistic settling time, without any positioning dispersion
    """
    def __init__(self, start_info):
        super().__init__(start_info)
        self.speed = 0.0 # reference speed in m/s
        self.steer = 0.0 # steering curvature (0 - means straight, 1/R left turn, -1/R right turn (R  turn radius)

    def observe(self, car, pixy2_info=None):
        """
        Used to observe the data reported by pixy2 and plan action by setting steering curvature and traction speed
        :param car: the car instance to be controlled
        :param pixy2_info: [[vector_id, x1, y1, x2, y2, ((X1,Y1),(X2,Y2)], # vector_info (may be None)
                            [[line_id, x1, y1, x2, y2,((X1,Y2),(X2,Y2))],] # lines_info (list of lines)
                            [[intersecting_segment_count, x1, y1, (X1, Y1)]]] # intersections_info (list of inersections)
                    x, y coordinates are in the Pixy2 original frame, but multiplied by 4 for easy display
                    X, Y coordinates are in the transformed top view frame, better for geometry calculations
                            currently configured as 1px/cm, centered view on lateral direction
                             and offset by 10cm forward from the rear wheel axle
                    NOTE: x,y coordinates are relative to the top-left corner which is (0,0)
                            so bottom-right is (316,208), or (78, 51) in the original reference frame
                          X,Y coordinate are also relative to the top-left corner of the top-view transformed image,
                                but the bottom right corner depends on how the perspective transformation is configured,
                                 currently being (299, 199) since 100px/m is used and the view is 3m wide by 2m forward
                                   with 0.1m offset from the rear axle forward
        :return: nothing... should just update speed and steer that are used in control method
        """
        vector_info, lines_info, intersections_info = pixy2_info  # split pixy2_info to make things easier to track
        if 0:  # for debug purposes
            print("V", vector_info)
            # print("L", lines_info)
            # print("I", intersections_info)
        if vector_info is not None:  # vector available, try use it...
            #p_far = car.calibration_pattern.cam_coord_to_orto_m(vector_info[1], vector_info[2])
            #p_near = car.calibration_pattern.cam_coord_to_orto_m(vector_info[3], vector_info[4])
            # print(p_far, p_near)
            self.speed = 2.0  # speed away!
            x, L, g = 0, 0, -5.5 # x deviation, total weights, correction gain
            for li in lines_info:
                p1 = car.calibration_pattern.cam_coord_to_orto_m(li[3], li[4])
                p2 = car.calibration_pattern.cam_coord_to_orto_m(li[1], li[2])
                if 0: # TRY THIS to ignore topview perspective correction, use pixy2 original outputs for test
                    p1, p2 = (li[3]>>2, li[4]>>2), (li[1]>>2, li[2]>>2)
                    # transform coordinates so that x- is left, x=0 is center, x+ is right, y+ is forward, y=0 is bottom
                    p1 = p1[0] - (car.camera.pixy_out.shape[1] >> 3), (car.camera.pixy_out.shape[0] >> 2) - p1[1]
                    p2 = p2[0] - (car.camera.pixy_out.shape[1] >> 3), (car.camera.pixy_out.shape[0] >> 2) - p2[1]
                    g = -0.2 # other correction gain is needed since now we work with pixels, not meters
                l = math.sqrt(p1[0] ** 2 + p1[1] ** 2) # maybe the further points from rear axle are more relevant... or not
                x += p1[0] * l # set the wheight of the x deviation from centerline according to the weight
                L += l # add weights for calculating the normalization divider L
                l = math.sqrt(p2[0] ** 2 + p2[1] ** 2) # repeat for the segment's second endpoint
                x += p2[0] * l
                L += l
            x /= L
            self.steer = g * x # apply the correction coeficient to the estimated steering error to compensate

        else:  # when no vector in sight (sometimes lines might be detected, but no vector considered by pixy2)
            self.speed = 0.1  # reduce speed! don't brake walls, legs or wheels!
            self.steer = 0 # steer ahead
        # self.speed = 1.0; self.steer = -1.0/2 # run in right turning circle for test

    def control(self, car):
        """
        Called by the simulator to allow changing mechanical speed and steering references on each simulation step
        :param car: the car instance to be controlled
        :return: True to keep the simulation going
        """
        car.set_course_reference(self.speed, self.steer)
        return True
