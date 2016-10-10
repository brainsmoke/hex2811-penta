
import sys

frame_size = int(sys.argv[1])

gamma = 2.5
cut_off = 0x18

factor = 0xff00 / (255.**gamma)
gamma_map = [ int(x**gamma * factor) for x in range(256) ]

for i,v in enumerate(gamma_map):
    if v < cut_off:
        gamma_map[i] = 0

gamma_map = [ chr(x&0xff)+chr(x>>8) for x in gamma_map ]

while True:
    d = sys.stdin.read(frame_size)
    if len(d) == 0:
        break
    sys.stdout.write(''.join( gamma_map[ord(c)] for c in d )+"\xff\xff\xff\xf0")
    sys.stdout.flush()

