
import sys

frame_size = int(sys.argv[1])

END_OF_FRAME = "\xff\xff\xff\xf0"

def calc_gamma(gamma=2.5, cut_off=0x18):
    factor = 0xff00 / (255.**gamma)
    gamma_values = [ int(x**gamma * factor) for x in range(256) ]

    for i,v in enumerate(gamma_values):
        if v < cut_off:
            gamma_values[i] = 0

    return [ chr(x&0xff)+chr(x>>8) for x in gamma_values ]

def read_blocking(f, n):
    d = ''
    while len(d) < n:
       new = f.read(n-len(d))
       if len(new) == 0:
           break
       d += new
    return d

def translate(frame, gamma_map):
    return ''.join( gamma_map[ord(c)] for c in frame )+END_OF_FRAME

if __name__ == '__main__':

    gamma_map = calc_gamma()

    while True:
        frame = read_blocking(sys.stdin, frame_size)

        if len(frame) == 0:
            break

        sys.stdout.write(translate(frame, gamma_map))
        sys.stdout.flush()

