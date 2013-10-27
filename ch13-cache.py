
import random
import feedback as fb

class Cache( fb.Component ):
    def __init__( self, size, demand ):
        self.t = 0        # internal time counter, needed for last access time
        
        self.size = size  # size limit of cache
        self.cache = {}   # actual cache: cache[key] = last_accessed_time

        self.demand = demand # demand function

    def work( self, u ):
        self.t += 1

        self.size = max( 0, int(u) ) # non-negative integer

        i = self.demand( self.t )    # this is the "requested item"

        if i in self.cache:
            self.cache[i] = self.t   # update last access time
            return 1

        if len(self.cache) >= self.size: # must make room
            m = 1 + len(self.cache) - self.size # number of elements to delete

            tmp = {}
            for k in self.cache.keys():    # key by last_access_time
                tmp[ self.cache[k] ] = k
                
            for t in sorted( tmp.keys() ): # delete the oldest elements
                del self.cache[ tmp[t] ]
                m -= 1
                if m == 0:
                    break

        self.cache[i] = self.t       # insert into cache
        return 0


class SmoothedCache( Cache ):
    def __init__( self, size, demand, avg ):
        Cache.__init__( self, size,  demand );
        self.f = fb.FixedFilter( avg )
        
    def work( self, u ):
        y = Cache.work( self, u )
        return self.f.work(y)

# ============================================================

def statictest(demand_width):
    def demand( t ):
        return int( random.gauss( 0, demand_width ) )

    fb.static_test( SmoothedCache, (0, demand, 100), 150, 100, 5, 3000 )
    

def stepresponse():
    def demand( t ): 
        return int( random.gauss( 0, 15 ) )

    def setpoint( t ):
        return 40

    p = SmoothedCache( 0, demand, 100 )

    fb.step_response( setpoint, p )


def closedloop():
    def demand( t ): 
        return int( random.gauss( 0, 15 ) )
    
    def setpoint( t ):
        if t > 5000:
            return 0.5
        return 0.7

    p = SmoothedCache( 0, demand, 100 )
    c = fb.PidController( 100, 250 )

    fb.closed_loop( setpoint, c, p, 10000 )


def closedloop_jumps():
    def demand( t ):
        if t < 3000:
            return int( random.gauss( 0, 15 ) )
        elif t < 5000:
            return int( random.gauss( 0, 35 ) )
        else:
            return int( random.gauss( 100, 15 ) )
    
    def setpoint( t ):
        return 0.7

    p = SmoothedCache( 0, demand, 100 )
    c = fb.PidController( 270, 7.5 ) # Ziegler-Nichols - closedloop1
#   c = fb.PidController( 100, 4.3 ) # Cohen-Coon - 2
#   c = fb.PidController( 80, 2.0 )  # AMIGO - 3
#   c = fb.PidController( 150, 2 )   # 4

    fb.closed_loop( setpoint, c, p, 10000 )


# ============================================================    
        
if __name__ == '__main__':

    fb.DT = 1

    # statictest(35)  # 5, 15, 35  

    # stepresponse()

    # closedloop()
    
    closedloop_jumps()
