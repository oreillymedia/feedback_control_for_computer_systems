
import random

class Buffer:
    def __init__( self, max_wip, max_flow ):
        self.queued = 0
        self.wip = 0             # work-in-progress ("ready pool")

        self.max_wip = max_wip
        self.max_flow = max_flow # avg outflow is max_flow/2

    def work( self, u ):
        # Add to ready pool
        u = max( 0, int(round(u)) )
        u = min( u, self.max_wip )
        self.wip += u

        # Transfer from ready pool to queue
        r = int( round( random.uniform( 0, self.wip ) ) )
        self.wip -= r
        self.queued += r

        # Release from queue to downstream process
        r = int( round( random.uniform( 0, self.max_flow ) ) )
        r = min( r, self.queued )
        self.queued -= r

        return self.queued

class Controller:
    def __init__( self, kp, ki ):
        self.kp, self.ki = kp, ki
        self.i = 0       # Cumulative error ("integral")

    def work( self, e ):
        self.i += e

        return self.kp*e + self.ki*self.i

# ============================================================

def open_loop( p, tm=5000 ):
    def target( t ):
        return 5.0  # 5.1
    
    for t in range( tm ):
        u = target(t)
        y = p.work( u )

        print t, u, 0, u, y

def closed_loop( c, p, tm=5000 ):
    def setpoint( t ):
        if t < 100: return 0
        if t < 300: return 50
        return 10
    
    y = 0 
    for t in range( tm ):
        r = setpoint(t)
        e = r - y
        u = c.work(e)
        y = p.work(u)

        print t, r, e, u, y

# ============================================================

c = Controller( 1.25, 0.01 )
p = Buffer( 50, 10 )

# open_loop( p, 1000 )
closed_loop( c, p, 1000 )




