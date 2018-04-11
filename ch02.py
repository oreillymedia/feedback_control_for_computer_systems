
import sys
import math

r = 0.6
k = 50 # float( sys.argv[1] ) # 50..175

print ( "r=%f\tk=%f\n" % ( r, k ))
t=0

print ( r, 0, 0, 0, 0)

def cache( size ):
    if size < 0:
        hitrate = 0
    elif size > 100:
        hitrate = 1
    else:
        hitrate = size/100.0
    return hitrate

y, c = 0, 0
for _ in range( 200 ):
    e = r - y      # tracking error
    c += e         # cumulative error
    u = k*c        # control action
    y = cache(u)   # process output

    print ( r, e, c, u, y)
