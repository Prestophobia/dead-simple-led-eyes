import board
import neopixel
import random
import math
from micropython import const
from adafruit_seesaw.seesaw import Seesaw

BUTTON_X = const(6)
BUTTON_Y = const(2)
BUTTON_A = const(5)
BUTTON_B = const(1)
BUTTON_SELECT = const(0)
BUTTON_START = const(16)
button_mask = const(
    (1 << BUTTON_X)
    | (1 << BUTTON_Y)
    | (1 << BUTTON_A)
    | (1 << BUTTON_B)
    | (1 << BUTTON_SELECT)
    | (1 << BUTTON_START)
)

i2c_bus = board.STEMMA_I2C()  # The built-in STEMMA QT connector on the microcontroller

seesaw: Seesaw = Seesaw(i2c_bus, addr=0x50)

seesaw.pin_mode_bulk(button_mask, seesaw.INPUT_PULLUP)

last_x: int = 0
last_y: int = 0

brightness: float = 0.33

t_blink: int = 0
blink_eyes_open_duration: int = 160
blink_eyes_closed_duration: int = 4
blink_eyelid_frames: int = 5
blink_start: int = blink_eyes_open_duration
blink_eyes_close_start: int = blink_start + blink_eyelid_frames
blink_eyes_close_end: int = blink_eyes_close_start + blink_eyes_closed_duration
blink_end: int = blink_eyes_close_end + blink_eyelid_frames
inverseCloseStartMinusBlinkStart: float = 1/(blink_eyes_close_start - blink_start)
inverseBlinkEndMinusCloseEnd: float = 1/(blink_end - blink_eyes_close_end)

num_pixels: int = 128
eye_right: neopixel.NeoPixel = neopixel.NeoPixel(board.D13,num_pixels,auto_write=False,brightness=brightness)

debugLight: neopixel.NeoPixel = neopixel.NeoPixel(board.D4,1,brightness=brightness)

# colors here are represented a 1 byte per color

RED: tuple[int, int, int] = (127, 0, 0)
YELLOW: tuple[int, int, int] = (255, 255, 0)
GREEN: tuple[int, int, int] = (0, 127, 0)
CYAN: tuple[int, int, int] = (0, 255, 255)
BLUE: tuple[int, int, int] = (0, 0, 127)
MAGENTA: tuple[int, int, int] = (255, 0, 255)

GRADIENTS = [[RED,YELLOW],[YELLOW,GREEN],[GREEN,CYAN],[CYAN,BLUE],[BLUE,MAGENTA],[MAGENTA,RED]]

def HSVtoRGB(h: float, s: float, v: float):
    c: float = v * s
    x: float = c * (1 - abs((h / 60) % 2 - 1))
    m: float = v - c
    r: float = 0
    g: float = 0
    b: float = 0
    if h >= 0 and h < 60:
        r = c
        g = x
    elif h >= 60 and h < 120:
        r = x
        g = c
    elif h >= 120 and h < 180:
        g = c
        b = x
    elif h >= 180 and h < 240:
        g = x
        b = c
    elif h >= 240 and h < 300:
        r = x
        b = c
    elif h >= 300 and h <= 360:
        r = c
        b = x
    r = (r + m) * 255
    g = (g + m) * 255
    b = (b + m) * 255
    return (int(r), int(g), int(b))

def GetColor_HSV(h,s,v) -> tuple[int, int, int]:
    while h > 360:
        h = h - 360
    color3 = HSVtoRGB(h,s,v)
    return color3

conversionFactor = 7/1024

def GetJoyIn() -> tuple[int, int]:
    #seesaw.
    try:
        xInRaw: int = (1023 - seesaw.analog_read(14)) - 511
        yInRaw: int = (1023 - seesaw.analog_read(15)) - 511

        x = - xInRaw
        y = yInRaw
        cos_rad = math.cos(math.pi /4)
        sin_rad = math.sin(math.pi /4)
        qx = cos_rad * x + sin_rad * y
        qy = -sin_rad * x + cos_rad * y


        xIn = int(qx* conversionFactor)
        yIn = int(qy *conversionFactor)
        return (xIn,yIn)
    except:
        return (4,4)

framerate: int = 120
dt: float = 1/framerate

class Eye:
    def __init__(self,center: tuple[int, int],width: int,height: int,orientation: int, isRight: bool, neopixel: neopixel.NeoPixel):
        self.center: tuple[int, int] = center
        self.width: int = width
        self.height: int = height
        self.neopixel = neopixel
        self.isRight: bool = isRight
        self.R: list[int] = []
        self.G: list[int] = []
        self.B: list[int] = []
        self.R_prev: list[int] = []
        self.G_prev: list[int] = []
        self.B_prev: list[int] = []
        self.orientation: int = orientation
        for i in range(64):
            self.R.append(0)
            self.G.append(0)
            self.B.append(0)
            self.R_prev.append(0)
            self.G_prev.append(0)
            self.B_prev.append(0)

    def blit(eye,):
        j: int = 0
        for x in range(8):
            for y in range(8):
                iOut: int = 0
                i = y + x * 8
                if x % 2 == 0:
                    iOut = (7 - y) + x * 8
                else:
                    iOut = y + x * 8
                if not eye.isRight:
                    iOut = iOut + 64
                eye.neopixel[iOut] = (eye.R[i], eye.G[i], eye.B[i])
                eye.R_prev[i] = eye.R[i]
                eye.G_prev[i] = eye.G[i]
                eye.B_prev[i] = eye.B[i]
                j = j+1

    def show(eye):
        eye.neopixel.show()
    
    def updateGradientPattern_HSV(eye,h:int,xyIn: tuple[int, int], buttons):
        xIn: int = xyIn[0]
        yIn: int = xyIn[1]
        cX: int = eye.center[0]-xIn
        cY: int = eye.center[1]-yIn

        yDist: int = 0
        yDistSq: int = 0
        xDist: int = 0
        distSq: int = 0
        dist: float = 0

        s: float = 0
        v: float = 0

        pixelH: float = 0

        i: int = 0

        color: tuple[int, int, int] = (0,0,0)

        for y in range(8):
            yDist = abs(y-cY)
            yDistSq = yDist*yDist
            for x in range(8):
                xDist = abs(x-cX)
                distSq = xDist*xDist+yDistSq
                dist = math.sqrt(distSq)

                if not buttons & (1 << BUTTON_Y):
                    s = dist * 0.2
                    eye.neopixel.brightness = brightness * 5
                elif not buttons & (1 << BUTTON_START):
                    s = dist * 0.25
                    eye.neopixel.brightness = 1
                elif not buttons & (1 << BUTTON_X):
                    s = dist * 0.25
                    eye.neopixel.brightness = brightness * 4
                elif not buttons & (1 << BUTTON_A):
                    s = dist * 0.25
                    eye.neopixel.brightness = brightness * 2
                elif not buttons & (1 << BUTTON_B):
                    s = dist * 0.25
                    eye.neopixel.brightness = brightness * 2
                elif not buttons & (1 << BUTTON_SELECT):
                    s = dist * 0.25
                    eye.neopixel.brightness = brightness * 3
                else:
                    s = dist * 0.25
                    eye.neopixel.brightness = brightness

                if not buttons & (1 << BUTTON_START):
                    v = dist
                else:
                    v = s
                s = max(0,min(s,1))
                v = max(0,min(v,1))
                v = 1 - v

                pixelH = float(h) + float(y*x) * 0.5

                if y%2 == 0:
                    if x % 2 == 0:
                        pixelH = math.ceil(pixelH)
                    else:
                        pixelH = math.floor(pixelH)
                else:
                    if x % 2 == 0:
                        pixelH = math.floor(pixelH)
                    else:
                        pixelH = math.ceil(pixelH)

                if not buttons & (1 << BUTTON_SELECT):
                    color = GetColor_HSV(300+(y*5),s,1)
                elif not buttons & (1 << BUTTON_START):
                    color = GetColor_HSV(0,1,v)
                elif not buttons & (1 << BUTTON_B):
                    color = GetColor_HSV(180+(y*10),s,v)
                elif not buttons & (1 << BUTTON_X):
                    color = GetColor_HSV(pixelH,s,1)
                else:
                    color = GetColor_HSV(pixelH,s,v)

                i = y + x*8
                eye.R[i] = color[0]
                eye.G[i] = color[1]
                eye.B[i] = color[2]

    def applyBlink(eye,t: int) -> int:
        if not buttons & (1 << BUTTON_SELECT):
            return t
        elif not buttons & (1 << BUTTON_START):
            return t
        elif not buttons & (1 << BUTTON_B):
            return t
        elif not buttons & (1 << BUTTON_X):
            return t
        
        t = t+1
        if t > blink_end:
            return 0
        
        z:float = 0

        if t >= blink_start:
            if t <= blink_eyes_close_start:
                ##I am closing my eyes
                z = (t - blink_start)*inverseCloseStartMinusBlinkStart
                z = z * 8
                
                for y in range(8): 
                    for x in range(8):
                        if y + x < z:
                            i = y + x * 8
                            eye.R[i] = 0
                            eye.G[i] = 0
                            eye.B[i] = 0
                    
                        
            elif t <= blink_eyes_close_end:
                ##My eyes are closed
                for i in range(64):
                    eye.R[i] = 0
                    eye.G[i] = 0
                    eye.B[i] = 0
            else:
                ##I am opening my eyes
                z = (t - blink_eyes_close_end)*inverseBlinkEndMinusCloseEnd
                z = 1 - z
                z = z * 8
                for y in range(8): 
                    for x in range(8):
                        if y +x < z:
                            i = y + x * 8
                            eye.R[i] = 0
                            eye.G[i] = 0
                            eye.B[i] = 0
        return t
    
    def applyEyeMask(eye, xyIn: tuple[int, int], buttons):
        xIn: int = xyIn[0]
        yIn: int = xyIn[1]
        cX: int = eye.center[0]-xIn -1
        cY: int = eye.center[1]-yIn -1
        i: int = 0
        j: int = 0

        if not buttons & (1 << BUTTON_SELECT):
            for x in range(8):
                for y in range(8): 
                    i = y + x * 8

                    if x <= (cX - 1) and y <=(cY - 1):
                        eye.R[i] =  0
                        eye.G[i] =  0
                        eye.B[i] =  0
                    if x <= (cX - 3) or y <=(cY - 3):
                        eye.R[i] =  0
                        eye.G[i] =  0
                        eye.B[i] =  0
                    if x > (cX + 3) or y >(cY + 3):
                        eye.R[i] =  0
                        eye.G[i] =  0
                        eye.B[i] =  0

        elif not buttons & (1 << BUTTON_X):
            for x in range(8):
                for y in range(8): 
                    i = y + x * 8
                    if ((x < 3 or y < 3) or (x > 3 and y > 3)):
                        eye.R[i] =  0
                        eye.G[i] =  0
                        eye.B[i] =  0
        elif not buttons & (1 << BUTTON_A):
            for y in range(8):
                for x in range(8): 
                    i = y + x * 8
                    j = x + y
                    if j < 8:
                            eye.R[i] =  0
                            eye.G[i] =  0
                            eye.B[i] =  0
        elif not buttons & (1 << BUTTON_B):
            for y in range(8):
                for x in range(8): 
                    i = y + x * 8
                    j = x + y
                    if eye.isRight:
                        if y < 4:
                            eye.R[i] =  0
                            eye.G[i] =  0
                            eye.B[i] =  0
                            continue
                    else:
                        if x < 4:
                            eye.R[i] =  0
                            eye.G[i] =  0
                            eye.B[i] =  0
                            continue

    def applyRotation(eye):
        i: int = 0
        i2: int = 0
        i3: int = 0
        i4: int = 0

        tempR: int = 0
        tempG: int = 0
        tempB: int = 0

        for r in range(eye.orientation):
            for x in range(4):
                for y in range(x,7-x):
                    #x,y
                    i = y + x * 8
                    
                    #y,7-x
                    i2 = (7-x) + y * 8

                    #7-x,7-y
                    i3 = (7-y) + (7-x) * 8

                    #7-y,x
                    i4 = x + (7-y) * 8

                    tempR = eye.R[i]
                    tempG = eye.G[i]
                    tempB = eye.B[i]

                    eye.R[i] = eye.R[i2]
                    eye.G[i] = eye.G[i2]
                    eye.B[i] = eye.B[i2]

                    eye.R[i2] = eye.R[i3]
                    eye.G[i2] = eye.G[i3]
                    eye.B[i2] = eye.B[i3]

                    eye.R[i3] = eye.R[i4]
                    eye.G[i3] = eye.G[i4]
                    eye.B[i3] = eye.B[i4]

                    eye.R[i4] = tempR
                    eye.G[i4] = tempG
                    eye.B[i4] = tempB

    def applyChromaticAbberation(eye):
        rSample: list[int] = []
        gSample: list[int] = []
        bSample: list[int] = []

        i: int = 0
        l: int = 0
        r: int = 0
        u: int = 0
        d: int = 0

        for i in range(64):
            rSample.append(eye.R[i])
            gSample.append(eye.G[i])
            bSample.append(eye.B[i])
        
        for x in range(1,7,1):
            for y in range(1,7,1):
                i = x + y * 8
                l = i - 1
                r = i + 1
                u = i - 8
                d = i + 8
                eye.R[i] = int(rSample[i] * 0.8 + rSample[l] * 0.1 + rSample[u] * 0.1)
                eye.G[i] = gSample[i]
                eye.B[i] = int(bSample[i] * 0.8 + bSample[r] * 0.1 + bSample[d] * 0.1)

    def applyGhost(eye):
        for i in range(64):
            eye.R[i] = int(eye.R[i] * 0.7 + eye.R_prev[i] * 0.3)
            eye.G[i] = int(eye.G[i] * 0.9 + eye.G_prev[i] * 0.1)
            eye.B[i] = int(eye.B[i] * 0.8 + eye.B_prev[i] * 0.2)

eye_L: Eye = Eye((4,4),3,3,3,False,eye_right)
eye_R: Eye = Eye((4,4),3,3,0,True,eye_right)

gradT: int = 0
h: int = random.randrange(0,359)

isRed: bool = False

while True:
    buttons = seesaw.digital_read_bulk(button_mask)
    xyIn: tuple[int, int] = GetJoyIn()

    eye_L.updateGradientPattern_HSV(h,xyIn, buttons)
    eye_R.updateGradientPattern_HSV(h,xyIn, buttons)
    
    eye_L.applyBlink(t_blink)
    t_blink =  eye_R.applyBlink(t_blink)
    
    eye_L.applyEyeMask(xyIn, buttons)
    eye_R.applyEyeMask(xyIn, buttons)

    eye_L.applyRotation()
    eye_R.applyRotation()

    eye_L.applyGhost()
    eye_R.applyGhost()

    eye_L.applyChromaticAbberation()
    eye_R.applyChromaticAbberation()
    
    eye_R.blit()
    eye_L.blit()
    eye_R.show()

    if isRed:
        debugLight.fill((0,0,255))
        isRed = False
    else:
        debugLight.fill((0,255,0))
        isRed = True

    h = h + 2
    if h >= 360:
        h = 0

    if t_blink == 0:
        blink_eyes_open_duration = random.randrange(155,200)
        if blink_eyes_open_duration < 160:
            blink_eyes_open_duration = 0
        blink_start = blink_eyes_open_duration
        blink_eyes_close_start = blink_start + blink_eyelid_frames
        blink_eyes_close_end = blink_eyes_close_start + blink_eyes_closed_duration
        blink_end = blink_eyes_close_end + blink_eyelid_frames