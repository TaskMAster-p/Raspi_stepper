import time
import math
import RPi.GPIO as GPIO

DIRECTION_CW = 1  # Clockwise
DIRECTION_CCW = 0  # Counterclockwise

class AccelStepper:
    def __init__(self, interface, pin1, pin2, pin3=None, pin4=None, enable=True):
        self._interface = interface
        self._currentPos = 0
        self._targetPos = 0
        self._speed = 0.0
        self._maxSpeed = 0.0
        self._acceleration = 0.0
        self._sqrt_twoa = 1.0
        self._stepInterval = 0
        self._minPulseWidth = 1
        self._enablePin = None
        self._lastStepTime = 0
        self._pin = [pin1, pin2, pin3, pin4]
        self._enableInverted = False
        
        self._n = 0
        self._c0 = 0.0
        self._cn = 0.0
        self._cmin = 1.0
        self._direction = DIRECTION_CCW

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._pin[0], GPIO.OUT)
        GPIO.setup(self._pin[1], GPIO.OUT)
        if self._interface in ["FULL4WIRE", "HALF4WIRE"] and pin3 is not None and pin4 is not None:
            GPIO.setup(self._pin[2], GPIO.OUT)
            GPIO.setup(self._pin[3], GPIO.OUT)
        
        if enable:
            self.enableOutputs()

        self.setAcceleration(1)
        self.setMaxSpeed(1)

    def moveTo(self, absolute):
        if self._targetPos != absolute:
            self._targetPos = absolute
            self.computeNewSpeed()

    def move(self, relative):
        self.moveTo(self._currentPos + relative)

    def runSpeed(self):
        if not self._stepInterval:
            return False
        
        time_now = time.time() * 1000000  # Current time in microseconds
        if time_now - self._lastStepTime >= self._stepInterval:
            if self._direction == DIRECTION_CW:
                self._currentPos += 1
            else:
                self._currentPos -= 1
            self.step(self._currentPos)
            self._lastStepTime = time_now
            return True
        else:
            return False

    def distanceToGo(self):
        return self._targetPos - self._currentPos

    def targetPosition(self):
        return self._targetPos

    def currentPosition(self):
        return self._currentPos

    def setCurrentPosition(self, position):
        self._targetPos = self._currentPos = position
        self._n = 0
        self._stepInterval = 0
        self._speed = 0.0

    def computeNewSpeed(self):
        distanceTo = self.distanceToGo()
        stepsToStop = (self._speed * self._speed) / (2.0 * self._acceleration)
        
        if distanceTo == 0 and stepsToStop <= 1:
            self._stepInterval = 0
            self._speed = 0.0
            self._n = 0
            return self._stepInterval

        if distanceTo > 0:
            if self._n > 0:
                if stepsToStop >= distanceTo or self._direction == DIRECTION_CCW:
                    self._n = -stepsToStop
            elif self._n < 0:
                if stepsToStop < distanceTo and self._direction == DIRECTION_CW:
                    self._n = -self._n
        elif distanceTo < 0:
            if self._n > 0:
                if stepsToStop >= -distanceTo or self._direction == DIRECTION_CW:
                    self._n = -stepsToStop
            elif self._n < 0:
                if stepsToStop < -distanceTo and self._direction == DIRECTION_CCW:
                    self._n = -self._n

        if self._n == 0:
            self._cn = self._c0
            self._direction = DIRECTION_CW if distanceTo > 0 else DIRECTION_CCW
        else:
            self._cn = self._cn - ((2.0 * self._cn) / ((4.0 * self._n) + 1))
            self._cn = max(self._cn, self._cmin)
        
        self._n += 1
        self._stepInterval = self._cn
        self._speed = 1000000.0 / self._cn
        if self._direction == DIRECTION_CCW:
            self._speed = -self._speed
        
        return self._stepInterval

    def run(self):
        if self.runSpeed():
            self.computeNewSpeed()
        return self._speed != 0.0 or self.distanceToGo() != 0

    def setMaxSpeed(self, speed):
        if speed < 0.0:
            speed = -speed
        if self._maxSpeed != speed:
            self._maxSpeed = speed
            self._cmin = 1000000.0 / speed
            if self._n > 0:
                self._n = (self._speed * self._speed) / (2.0 * self._acceleration)
                self.computeNewSpeed()

    def setAcceleration(self, acceleration):
        if acceleration == 0.0:
            return
        if acceleration < 0.0:
            acceleration = -acceleration
        if self._acceleration != acceleration:
            self._n = self._n * (self._acceleration / acceleration)
            self._c0 = 0.676 * math.sqrt(2.0 / acceleration) * 1000000.0
            self._acceleration = acceleration
            self.computeNewSpeed()

    def setSpeed(self, speed):
        if speed == self._speed:
            return
        speed = max(min(speed, self._maxSpeed), -self._maxSpeed)
        if speed == 0.0:
            self._stepInterval = 0
        else:
            self._stepInterval = abs(1000000.0 / speed)
            self._direction = DIRECTION_CW if speed > 0.0 else DIRECTION_CCW
        self._speed = speed

    def step(self, step):
        if self._interface == "DRIVER":
            self.step1(step)
        elif self._interface == "FULL2WIRE":
            self.step2(step)
        elif self._interface == "FULL4WIRE":
            self.step4(step)
        elif self._interface == "HALF4WIRE":
            self.step8(step)

    def step1(self, step):
        GPIO.output(self._pin[1], self._direction)
        GPIO.output(self._pin[0], GPIO.HIGH)
        time.sleep(self._minPulseWidth / 1000000.0)
        GPIO.output(self._pin[0], GPIO.LOW)

    def step2(self, step):
        if step & 0x3 == 0:
            GPIO.output(self._pin[0], GPIO.HIGH)
            GPIO.output(self._pin[1], GPIO.LOW)
        elif step & 0x3 == 1:
            GPIO.output(self._pin[0], GPIO.HIGH)
            GPIO.output(self._pin[1], GPIO.HIGH)
        elif step & 0x3 == 2:
            GPIO.output(self._pin[0], GPIO.LOW)
            GPIO.output(self._pin[1], GPIO.HIGH)
        else:
            GPIO.output(self._pin[0], GPIO.LOW)
            GPIO.output(self._pin[1], GPIO.LOW)

    def step4(self, step):
        if step & 0x3 == 0:
            GPIO.output(self._pin[0], GPIO.HIGH)
            GPIO.output(self._pin[2], GPIO.LOW)
        elif step & 0x3 == 1:
            GPIO.output(self._pin[1], GPIO.HIGH)
            GPIO.output(self._pin[2], GPIO.LOW)
        elif step & 0x3 == 2:
            GPIO.output(self._pin[1], GPIO.HIGH)
            GPIO.output(self._pin[3], GPIO.LOW)
        else:
            GPIO.output(self._pin[0], GPIO.LOW)
            GPIO.output(self._pin[3], GPIO.LOW)

    def step8(self, step):
        if step & 0x7 == 0:
            GPIO.output(self._pin[0], GPIO.HIGH)
        elif step & 0x7 == 1:
            GPIO.output(self._pin[0], GPIO.HIGH)
            GPIO.output(self._pin[2], GPIO.HIGH)
        elif step & 0x7 == 2:
            GPIO.output(self._pin[2], GPIO.HIGH)
        elif step & 0x7 == 3:
            GPIO.output(self._pin[2], GPIO.HIGH)
            GPIO.output(self._pin[3], GPIO.HIGH)
        elif step & 0x7 == 4:
            GPIO.output(self._pin[3], GPIO.HIGH)
        elif step & 0x7 == 5:
            GPIO.output(self._pin[3], GPIO.HIGH)
            GPIO.output(self._pin[1], GPIO.HIGH)
        elif step & 0x7 == 6:
            GPIO.output(self._pin[1], GPIO.HIGH)
        else:
            GPIO.output(self._pin[0], GPIO.HIGH)
            GPIO.output(self._pin[1], GPIO.HIGH)

    def enableOutputs(self):
        for pin in self._pin:
            if pin is not None:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)

    def disableOutputs(self):
        for pin in self._pin:
            if pin is not None:
                GPIO.output(pin, GPIO.LOW)
        
        if self._enablePin is not None:
            GPIO.setup(self._enablePin, GPIO.OUT)
            GPIO.output(self._enablePin, GPIO.LOW)

    def runToPosition(self):
        while self.run():
            pass

    def runSpeedToPosition(self):
        if self._targetPos == self._currentPos:
            return
        if self._targetPos > self._currentPos:
            self.setSpeed(self._maxSpeed)
        else:
            self.setSpeed(-self._maxSpeed)
        while self.runSpeed():
            pass

    def stop(self):
        if self._speed != 0.0:
            stepsToStop = (self._speed * self._speed) / (2.0 * self._acceleration)
            if self._speed > 0:
                self.move(stepsToStop)
            else:
                self.move(-stepsToStop)

##Example usage:

#stepper = AccelStepper("DRIVER", 17, 18)
#stepper.setMaxSpeed(1000)
#stepper.setAcceleration(500)

#stepper.moveTo(2000)

#while stepper.run():
#    pass

#GPIO.cleanup()
