import MagnetoShield
import time

MagnetoShield.begin()
MagnetoShield.calibration()
print(MagnetoShield.voltageRef)
print(MagnetoShield.minCalibrated)
print(MagnetoShield.maxCalibrated)
print(MagnetoShield.calibrated)
print("Picking up the magnet")
MagnetoShield.actuatorWrite(10.0)
time.sleep(2.0)
MagnetoShield.actuatorWrite(0.0)

while True:
    print((MagnetoShield.referenceRead(), MagnetoShield.auxReadCurrent(), MagnetoShield.sensorRead()))
    time.sleep(0.05)