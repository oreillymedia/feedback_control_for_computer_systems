
import math
import random
import feedback as fb

class AbstractServerPool( fb.Component ):
    def __init__( self, n, server, load ):
        self.n = n           # number of server instances
        self.queue = 0       # number of items in queue

        self.server = server # server work function
        self.load = load     # queue-loading work function


    def work( self, u ):
        self.n = max(0, int(round(u))) # server count: non-negative integer

        completed = 0
        for _ in range(self.n):
            completed += self.server() # each server does some amount of work

            if completed >= self.queue:
                completed = self.queue # "trim" completed to queue length
                break                  # stop if queue is empty 

        self.queue -= completed        # reduce queue by work completed

        return completed


    def monitoring( self ):
        return "%d %d" % ( self.n, self.queue )


class ServerPool( AbstractServerPool ):
    def work( self, u ):
        load = self.load()        # additions to the queue
        self.queue = load         # new load replaces old load

        if load == 0: return 1    # no work: 100 percent completion rate 

        completed = AbstractServerPool.work( self, u )

        return completed/load     # completion rate
        

class QueueingServerPool( AbstractServerPool ):
    def work( self, u ):
        load = self.load()        # additions to the queue
        self.queue += load        # new load is added to old load

        completed = AbstractServerPool.work( self, u )

        return load - completed   # net change in queue length

        
class ServerPoolWithLatency( ServerPool ):
    def __init__( self, n, server, load, latency ):
        ServerPool.__init__( self, n, server, load )

        self.latency = latency  # time steps before server becomes active
        self.pending = []       # list of pending servers


    def work( self, u ):
        u = max(0, int(round(u)))  # server count: non-negative integer

        if u <= self.n:            # no increase in servers: no latency
            return ServerPool.work( self, u )

        # for servers already pending: decrement waiting time
        for i in range( len(self.pending) ):
            self.pending[i] -= 1

        newly_active = self.pending.count( 0 ) # how many are done waiting?
        del self.pending[0:newly_active]       # remove from pending...
        self.n += newly_active                 # ... and add to active

        # now add to list of pending servers if requested by input
        self.pending.extend( [self.latency]*int(u-self.n) )

        return ServerPool.work( self, self.n )

    
# --------------------------------------------------

# Load and work functions (unless defined in local scope)

def load_queue():
    # This is only used by closedloop2() and closedloop3()
    
    global global_time
    global_time += 1

    if global_time > 2500:
        return random.gauss( 1200, 5 )

    if global_time > 2200:
        return random.gauss( 800, 5 )

    return random.gauss( 1000, 5 )

def consume_queue():
    a, b = 20, 2
    return 100*random.betavariate( a, b ) # mean: a/(a+b); var: ~b/a^2

# ============================================================

# Server Pool

def statictest( traffic ):
    def loadqueue():
        return random.gauss( traffic, traffic/200 )
    
    fb.static_test( ServerPool, ( 0, consume_queue, loadqueue ),
                    20, 20, 5, 1000 ) # max u, steps, trials, timesteps

def closedloop1():
    # Closed loop, setpoint 0.6-0.8, PID Controller
    
    def loadqueue():
        global global_time
        global_time += 1

        if global_time > 2100:
            return random.gauss( 1200, 5 )

        return random.gauss( 1000, 5 )


    def setpoint( t ):
        if t > 2000:
            return 0.6
        else:
            return 0.8


    p = ServerPool( 8, consume_queue, loadqueue )
    c = fb.PidController( 1, 5 )
    fb.closed_loop( setpoint, c, p, 10000 )

    
def closedloop2():
    # Closed loop, setpoint 0.999x, Asymm (!) Controller
    
    def setpoint( t ):
        if t < 1000:          # Switch on slowly, to avoid initial overshoot
            return t/1000.0
        return 0.9995

    class AsymmController( fb.PidController ):
        def work( self, e ):
            if e > 0:
                e /= 20.0
        
            self.i += fb.DT*e
            self.d = ( self.prev - e )/fb.DT
            self.prev = e

            return self.kp*e + self.ki*self.i + self.kd*self.d

    p = ServerPool( 0, consume_queue, load_queue )
    c = AsymmController( 10, 200 )
    fb.closed_loop( setpoint, c, p )


def closedloop3():
    # Closed loop, setpoint 1.0, incremental controller (non-PID)

    def setpoint( t ):
        return 1.0

    class SpecialController( fb.Component ):
        def __init__( self, period1, period2 ):
            self.period1 = period1
            self.period2 = period2
            self.t = 0

        def work( self, u ):
            if u > 0:
                self.t = self.period1
                return +1
        
            self.t -= 1          # At this point: u <= 0 guaranteed!

            if self.t == 0:
                self.t = self.period2
                return -1

            return 0

    p = ServerPool( 0, consume_queue, load_queue )
    c = SpecialController( 100, 10 )
    fb.closed_loop( setpoint, c, p, actuator=fb.Integrator() )

# ============================================================

# Queue Control

class InnerLoop( fb.Component ):
    def __init__( self, kp, ki, loader ):
        k = 1/100.
        
        self.c = fb.PidController( kp*k, ki*k )
        self.p = QueueingServerPool( 0, consume_queue, loader )

        self.y = 0

    def work( self, u ):
        e = u - self.y       # u is setpoint from outer loop
        e = -e               # inverted dynamics
        v = self.c.work( e )
        self.y = self.p.work( v ) # y is net change
        return self.p.queue

    def monitoring( self ):
        return "%s %d" % ( self.p.monitoring(), self.y ) # servers, queue, diff


def innerloop_steptest():
    def loadqueue():
        return 1000

    def setpoint( t ):
        if t < 1000 or t >= 1500:
            return 25
        else:
            return -25
    
    p = InnerLoop( 0.5, 0.25, loadqueue )
    fb.step_response( setpoint, p, tm=2000 )


def nestedloops():
    def setpoint( t ):
        return 200
        
        if t < 2000:
            return 100
        elif t < 3000:
            return 125
        else:
            return 25

    p = InnerLoop(0.5, 0.25, load_queue) # InnerLoop is "plant" for outer loop

#    c = fb.PidController( 0.06, 0.001 )
    c = fb.AdvController( 0.35, 0.0025, 4.5, smooth=0.15 )
    
#    fb.closed_loop( setpoint, c, p )
    fb.closed_loop( setpoint, c, p, actuator=fb.RecursiveFilter(0.5) )



# ============================================================

if __name__ == '__main__':

    fb.DT = 1

    global_time = 0 # To communicate with queue_load functions

#    statictest( 1000 )
#    closedloop1()
#    closedloop2()
#    closedloop3()

#    innerloop_steptest()

    nestedloops()
