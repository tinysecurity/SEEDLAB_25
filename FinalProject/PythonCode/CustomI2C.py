import numpy as np
from smbus2 import SMBus, i2c_msg

def floatToInts(data, precision):
    # change no. bytes to decrease precision
    # valid: 2, 4, 8
    match precision:
        case 2:
            precisionType = "float16"
        case 4:
            precisionType = "float32"
        case 8:
            precisionType = "float64"
        case _:
            precisionType = "float32"
            precision = 4
    dataAsInt = np.array(data).astype(precisionType).view(np.uint32)
    ints = []
    # big endian :)
    for i in range(precision-1,-1,-1):
        shift = i * 8
        mask = 0xFF << shift
        ints.append((dataAsInt & mask) >> shift)
    return ints

def sendMessage(cameraDict, address, precision, busIdx):
    data = None
    if len(cameraDict) == 0:
        data = [0] * (precision*2+1)
    else:
        arrow = cameraDict["arrowColor"] # get if detected and direction
        # the other variable is if data is detected, which is always yes at this point
        data = [(0 | arrow)] # template for bitmasking later
        data.extend(floatToInts(cameraDict["distance"], precision))
        data.extend(floatToInts(cameraDict["angle"], precision))
    #SMBus(busIdx).i2c_rdwr(i2c_msg.write(address, data))
    try:
        SMBus(busIdx).write_block_data(address, 0, data)
    except:
        print("Communication issue, I2C threw an exception...")
    #print(data)
