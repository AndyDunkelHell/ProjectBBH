#!/usr/bin/env python3
import os

tfpath = 'Python\model2Dv10flat.tflite'
hpath  = 'model2Dv10flat.h'

data = open(tfpath, 'rb').read()
n = len(data)

with open(hpath, 'w') as f:
    f.write(f'// Auto-generated from {tfpath}, {n} bytes\n')
    f.write('const unsigned char model2Dv10flat_tiny_tflite[] = {')
    for i, b in enumerate(data):
        if i % 12 == 0:
            f.write('\n  ')
        f.write(f'0x{b:02x},')
    f.write('\n};\n')
    f.write(f'const unsigned int model2Dv10flat_tiny_tflite_len = {n};\n')

print(f'Wrote {hpath} with {n} bytes.')
