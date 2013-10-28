
import sys

r = float(sys.argv[1]) # Reference or "setpoint"
k = float(sys.argv[2]) # Controller gain

u = 0        # "Previous" output
for _ in range( 200 ):
    y = u        # One-step delay: previous output
    
    e = r - y    # Tracking error
    u = k*e      # Controller ouput

    print r, e, 0, u, y
