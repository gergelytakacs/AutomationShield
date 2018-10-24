Please stick to the Arduino API style guide that can be found here when writing the shield API's.

Namely:

Use begin() to initialize a library instance, usually with some settings. Use end() to stop it.
Use read() to read inputs, and write() to write to outputs, e.g. digitalRead(), analogWrite(), etc.
According to this (subject to debate, say your thoughts) we will use:

begin() to initialize the board.
sensorRead() to read the outputs "y",
referenceRead() to read reference "r" (if there is one) and
actuatorWrite() to send input ''u".
All boards should use these basic functions so that the library remains consistent. Determine which type of input/output will be likely most used by the user and use those in these functions. If you want to use more types of sensor readings (voltage, physical units etc.) then it is suggested to get creative based on this. Such as

sensorReadVoltage() to read the outputs "y" in V.
sensorReadCelsius() to read the outputs "y" in degrees Celsius.