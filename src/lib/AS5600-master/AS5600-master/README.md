# AS5600

Arduino library for the AS5600 12-bit Magnetic Encoder. This library was written for the [NEMA-17 AS5600 Board](https://github.com/kanestoboi/AS5600-Nema-17-Board).

Features
* Read 12 bit value (0-4095)
* Read angle value in degrees (0-360 degrees)
* Get magnetic strength 
* Get gain

## Example 
The following example reads the angle (0-360) of the magnet relative to the AS5600 encoder in increments of 360/4096
```c
#include <AS5600.h>

AS5600 encoder;
float output;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // get the angle in degrees of the encoder
  output = encoder.getAngle();
  Serial.println(output);
}
````

## More Examples
See [`examples`](https://github.com/kanestoboi/AS5600/tree/master/examples) directory in this repository for more examples including continuous rotation.
