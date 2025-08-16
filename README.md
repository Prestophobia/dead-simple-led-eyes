# Dead simple LED eyes

Code for cheap, easy, and low energy robot eyes.

![A robot bee costume using these eyes](https://cdn.bsky.app/img/feed_thumbnail/plain/did:plc:elcafocfgtspqnpmrervz4uz/bafkreibbmkmlehboj7h7xoyarrug2pwthyoixxv3qztmvadyz5loewbqba@jpeg)

## Goal

The goal of this project was to make animated eyes for a robot costume that:

* Is relatively cheap.
* Can be powered off a phone charger.
* Can be iterated intermittently in my free time.
* Does not require an sensor equipment like cameras, accelerometers, etc.
* Could be replicated by others with little to no experience with soldering or electronics.

## Required Electronics

### Microcontroller

![Adafruit SCORPIO](https://cdn-shop.adafruit.com/970x728/5650-00.jpg)

This project was written for the [Adafruit Feather RP2040 SCORPIO](https://www.adafruit.com/product/5650), since its circuitpython compatibility make it very plug-and play.

### Input

![Adafruit Mini I2C Gamepad with seesaw - STEMMA QT / Qwiic](https://cdn-shop.adafruit.com/970x728/5743-05.jpg)

To control the direction and expression of the eyes, a [Adafruit Mini I2C Gamepad](https://www.adafruit.com/product/5743) connected to the scorpio's 4-pin JST connector per [Adafruit's STEMMA QT standard](https://learn.adafruit.com/introducing-adafruit-stemma-qt/what-is-stemma). Again, this makes the set up relatively plug-and-play.

### Display

![Generic $10 8x8 array off amazon.](https://m.media-amazon.com/images/I/61TG8pjXY+L._AC_SL1000_.jpg)

Any 8x8 array of WS2812B  individually addressible LEDs, which Adafruit calls "neopixels", should do. You need one per eye.

### Power Supply

Any portable USB phone battery will fit the bill.

## Usage

### Step 1: Assembly

Solder either the wires or the JST connector of one eye to D13, the USB power pin, and ground. Then daisy chain the second eye from the first eye. FInally, connect the controller to the SCORPIO's STEMMA QT port.

### Step 2: Installation

Flash circuitpython onto the SCORPIO. After that it's just a matter of putting code.py and its dependencies onto the SCORPIO.

### Step 3: Configuration

Not all LED arrays are wired in the same orientation. Edit the orientation and isRight arguments until the eyes are in the right direction.
