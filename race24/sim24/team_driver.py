import math
from model import Driver

TRACK_WIDTH_M = 0.550   # Full track width in meters
MIN_TURN_RADIUS_M = 0.47    # SteeringAckerman min_radius
MAX_CURVATURE = 1.0 / MIN_TURN_RADIUS_M
BASE_SPEED_MPS = 1.5    # max = 5.0 in Traction

class TeamDriver(Driver):
    def __init__(self, start_info):
        super().__init__(start_info)
        self.speed = 0.0    # m/s
        self.steer = 0.0    # curvature 1/r (0 = straight, + = left, - = right)
        self.kp = 3.5       # proportional steering gain (TODO: Andrei trebuie sa ii dea tune la param asta)

    # Octav o sa se ocupe aici de partea de calcul al pozitiei x (stanga dreapta) care va fi trimisa catre observe, momentan doar incearca sa duca spre mijlocul segmentului
    # si nu ia in calcul daca avem curba si se vede doar un segment, daca avem intersectie etc.
    def _compute_target_x(self, car, lines_info):
        """
        Converts Pixy2 line endpoints into meters relative to the rear axle
        and returns a single horizontal target_x (negative = left, positive = right).
        """
        if not lines_info:
            return None

        sum_x = 0.0
        count = 0
        for line in lines_info:
            # line = [line_id, x1, y1, x2, y2, ((X1, Y1), (X2, Y2))]
            p_start = car.calibration_pattern.cam_coord_to_orto_m(line[1], line[2])
            p_end = car.calibration_pattern.cam_coord_to_orto_m(line[3], line[4])

            sum_x = p_start[0] + p_end[0]
            count += 2

        return sum_x / count

    # Andrei o sa se ocupe de partea de control, aici doar am implementat un P controller simplu pentru a demonstra cum se poate folosi pixy2_info pentru a calcula steer si speed.
    # Va trebui sa implementezi un PID controller (ai deja functia facuta in model.py sau in carsim.py) si sa modifici parametrii, vezi daca e mai bun PD fata de PID
    def observe(self, car, pixy2_info = None):
        """Called every camera frame (~60Hz) by carsim.py"""
        if pixy2_info is None:
            return

        vector_info, lines_info, intersection_info = pixy2_info

        target_x = self._compute_target_x(self, car, lines_info)

        if target_x is not None:
            raw_curvature = -self.kp * target_x
            self.steer = max(-MAX_CURVATURE, min(raw_curvature, MAX_CURVATURE))
            self.speed = BASE_SPEED_MPS
        else:
            self.speed = 0.2
            self.steer = 0.0

    def control(self, car):
        """Called every physics step (1000 Hz) by VirtualCar.step()."""
        car.set_course_reference(self.speed, self.steer)
        return True