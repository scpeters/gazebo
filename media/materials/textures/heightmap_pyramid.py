#!/usr/bin/env python3

from PIL import Image

N = 64
im = Image.new("L", [N+1, N+1])

for i in range(N + 1):
    for j in range(N + 1):
        im.putpixel((i,j), int(256/N * (N/2 - max((abs(i - N/2), abs(j - N/2))))))

im.save("heightmap_pyramid_%ix%i.png" % (N + 1, N + 1))
