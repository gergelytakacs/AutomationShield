

ADCREF = 3.3                    # ADC reference voltage is only 3.3 V with Python compatible boards
ADCRES = 65536                  # Analog resolution of the Metro M4 is 16 bits
ARES3V3 = ADCREF / ADCRES       # Voltage per analog resolution level

def mapFloat(x, in_min, in_max, out_min, out_max):                                      # same as Arudino map() but with floating point numbers
    return ((x - in_min) * (out_max - out_min)) / ((in_max - in_min) + out_min)             # linear mapping, same as Arduino map()

def constrain(x, xmin, xmax):
    if x <= xmin:
        x = xmin
    elif x >= xmax:
        x = xmax
    return x