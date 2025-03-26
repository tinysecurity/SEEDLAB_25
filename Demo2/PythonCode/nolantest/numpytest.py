import numpy as np

pival = np.array(np.pi).astype('float32')
dec_pival = pival.view(np.int32)
print(pival.dtype)
print(pival)
#print(bin_pival.dtype)
print(dec_pival)

print(dec_pival & 0xFF)
print((dec_pival & 0xFF00) >> 8)
print((dec_pival & 0xFF0000) >> 16)
# write this one first, big endian
print((dec_pival & 0xFF000000) >> 24)

flag1 = 1
flag2 = 1
flag3 = 1
flags = (0 | (flag1 << 2)) + (0 | (flag2 << 1)) + (0 | flag3)

print(flags)
