import time
import math
import random
# Robot configuration
controller_1 = Controller(PRIMARY)
# drivetrain
dt_left = MotorGroup(Motor(Ports.PORT2, GearSetting.RATIO_6_1, True), Motor(Ports.PORT1, GearSetting.RATIO_6_1, True))
dt_right = MotorGroup(Motor(Ports.PORT3, GearSetting.RATIO_6_1, False), Motor(Ports.PORT4, GearSetting.RATIO_6_1, False))

dt_left.set_stopping(BRAKE)
dt_right.set_stopping(BRAKE)
# inertial & solenoid
orientation = Inertial(Ports.PORT9)
wings = DigitalOut(brain.three_wire_port.a)
# components
intake = Motor(Ports.PORT5, GearSetting.RATIO_18_1, True)
endgame = MotorGroup(Motor(Ports.PORT7, GearSetting.RATIO_18_1, True), Motor(Ports.PORT8, GearSetting.RATIO_18_1, False))
catapult = Motor(Ports.PORT6, GearSetting.RATIO_18_1, False)

intake.set_velocity(200, PERCENT)
# initialize everything
wait(300, MSEC)
print("\033[2J")
orientation.calibrate()
time.sleep(2)
orientation.set_rotation(0, DEGREES)
orientation.set_heading(0, DEGREES)
class Components:
    def __init__(self, endgame_ratio, cata_speed):
        self.endgame_ratio = endgame_ratio
        catapult.set_velocity(cata_speed, PERCENT)
        self.wing_value = False
        self.intake_value = None
        wings.set(False)
    def intake(self, direction):
        if direction == None:
            intake.stop()
        else:
            intake.spin(direction)
        self.intake_value = direction
    def endgame(self, direction, amount=None):
        endgame.set_velocity(25 if direction == FORWARD else 100, PERCENT)
        if direction == None:
            endgame.stop()
        elif amount == None:
            endgame.spin(direction)
        else:
            d = FORWARD if ((1 if direction == FORWARD else -1)*(1 if amount>=0 else -1)) == 1 else REVERSE
            endgame.spin_for(d, abs(amount)*self.endgame_ratio, TURNS)
    def catapult(self, direction=FORWARD, amount=None):
        if direction == None:
            dt_left.set_stopping(BRAKE)
            dt_right.set_stopping(BRAKE)
            catapult.stop()
        elif amount == None:
            dt_left.set_stopping(HOLD)
            dt_right.set_stopping(HOLD)
            catapult.spin(direction)
        else:
            d = FORWARD if ((1 if direction == FORWARD else -1)*(1 if amount>=0 else -1)) == 1 else REVERSE
            catapult.spin_for(d, abs(amount)*self.endgame_ratio, TURNS)
    def wings(self):
        self.wing_value = False if self.wing_value else True
        wings.set(self.wing_value)
class Drivetrain:
    def __init__(self, gear_ratio, wheel_diameter):
        self.gear_ratio = gear_ratio
        self.wheel_diameter = wheel_diameter
    def drive4(self, inches, speed=110, timeout=15):
        dt_left.set_timeout(15)
        dt_right.set_timeout(15)
        dt_left.set_velocity(speed/2, PERCENT)
        dt_right.set_velocity(speed/2, PERCENT)
        direction = FORWARD if inches > 0 else REVERSE
        dt_left.spin_for(direction, (abs(inches)/(math.pi*self.wheel_diameter))*self.gear_ratio, TURNS, wait=False)
        dt_right.spin_for(direction, (abs(inches)/(math.pi*self.wheel_diameter))*self.gear_ratio, TURNS, wait=False)
        old_time = brain.timer.time(SECONDS)
        while dt_left.is_spinning() or dt_right.is_spinning():
            if brain.timer.time(SECONDS)-old_time >= timeout:
                dt_left.stop()
                dt_right.stop()
    def turn2(self, angle_unmodded, speed=30):
        angle = angle_unmodded % 360
        while abs(angle - orientation.heading(DEGREES)) % 360 > 0.5:
            h = orientation.heading(DEGREES)
            dt_left.set_velocity(abs((angle - h + 180) % 360 - 180) * speed * 2 / 180 + 3, PERCENT)
            dt_right.set_velocity(abs((angle - h + 180) % 360 - 180) * speed * 2 / 180 + 3, PERCENT)
            if (angle - h + 180) % 360 - 180 > 0:
                # turn right
                dt_left.spin(FORWARD)
                dt_right.spin(REVERSE)
            else:
                # turn left
                dt_left.spin(REVERSE)
                dt_right.spin(FORWARD)
            wait(15, MSEC)
        print(orientation.heading(DEGREES))
        dt_left.stop()
        dt_right.stop()
        if abs(angle - orientation.heading(DEGREES)) % 360 > 0.5:
            self.turn2(angle, speed=10)
dt = Drivetrain(60/36, 3.25)
cp = Components(80/12, 60)
# driver control
def driver_control():
    controller_1.buttonL2.pressed(lambda: cp.wings())
    controller_1.buttonR2.pressed(lambda: cp.wings())

    controller_1.buttonL1.pressed(lambda: cp.intake(None if cp.intake_value == FORWARD else FORWARD))
    controller_1.buttonR1.pressed(lambda: cp.intake(None if cp.intake_value == REVERSE else REVERSE))
    
    controller_1.buttonUp.pressed(lambda: cp.endgame(FORWARD))
    controller_1.buttonDown.pressed(lambda: cp.endgame(REVERSE))
    controller_1.buttonUp.released(lambda: cp.endgame(None))
    controller_1.buttonDown.released(lambda: cp.endgame(None))

    controller_1.buttonX.pressed(lambda: cp.catapult())
    controller_1.buttonB.released(lambda: catapult.stop())

    while True:
        dt_left_speed = controller_1.axis3.position() + controller_1.axis1.position()
        dt_left.set_velocity(abs(dt_left_speed), PERCENT)
        dt_left_dir = FORWARD if dt_left_speed >= 0 else REVERSE

        dt_right_speed = controller_1.axis3.position() - controller_1.axis1.position()
        dt_right.set_velocity(abs(dt_right_speed), PERCENT)
        dt_right_dir = FORWARD if dt_right_speed >= 0 else REVERSE
        
        dt_left.spin(dt_left_dir)
        dt_right.spin(dt_right_dir)
        wait(20, MSEC)
