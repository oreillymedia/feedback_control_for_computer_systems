
import math
import random
import feedback as fb

class AdPublisher( fb.Component ):

    def __init__( self, scale, min_price, relative_width=0.1 ):
        self.scale = scale
        self.min = min_price
        self.width = relative_width

    def work( self, u ):
        if u <= self.min:       # Price below min: no impressions
            return 0

        # "demand" is the number of impressions served per day
        # The demand is modeled (!) as Gaussian distribution with
        # a mean that depends logarithmically on the price u.
        
        mean = self.scale*math.log( u/self.min )        
        demand = int( random.gauss( mean, self.width*mean ) )

        return max( 0, demand ) # Impression demand is greater than zero


class AdPublisherWithWeekend( AdPublisher ):
    
    def __init__( self, weekday, weekend, min_price, relative_width=0.1 ):
        AdPublisher.__init__( self, None, min_price, relative_width )

        self.weekday = weekday
        self.weekend = weekend        

        self.t = 0      # Internal day counter

    def work( self, u ):
        self.t += 1

        if self.t%7 < 2: # Weekend
            self.scale = self.weekend
        else:
            self.scale = self.weekday

        return AdPublisher.work( self, u )
    
# ------------------------------------------------------------

def statictest():
    fb.static_test( AdPublisher, (100,2), 20, 100, 10, 5000 )


def closedloop( kp, ki, f=fb.Identity() ):
    def setpoint( t ):
        if t > 1000:
            return 125
        return 100

    k = 1.0/20.0

    p = AdPublisher( 100, 2 )
    c = fb.PidController( k*kp, k*ki )

    fb.closed_loop( setpoint, c, p, returnfilter=f )


accumul_goal = 0
def closedloop_accumul( kp, ki ):
    def setpoint( t ):
        global accumul_goal
        
        if t > 1000:
            accumul_goal += 125
        else:
            accumul_goal += 100
        return accumul_goal
    
    k = 1.0/20.0

    p = AdPublisher( 100, 2 )
    c = fb.PidController( k*kp, k*ki )

    fb.closed_loop( setpoint, c, p, returnfilter=fb.Integrator() )


def specialsteptest():
    p = AdPublisher( 100, 2 )
    f = fb.RecursiveFilter(0.05)
    
    for t in range( 500 ):
        r = 5.50
        u = r
        y = p.work( u )
        z = f.work( y )

        print t, t*fb.DT, r, 0, u, u, y, z, p.monitoring()
        
    quit()
    
# ------------------------------------------------------------

if __name__ == '__main__':

    fb.DT = 1

#   statictest()

#    closedloop( 0.5, 0.25 ) # default
#    closedloop( 0.0, 0.25 ) # w/o prop ctrl
#    closedloop( 0.0, 1.75 )  # ringing
    
#    closedloop( 1.0, 0.125, fb.RecursiveFilter(0.125) ) # 
        
#    closedloop_accumul( 0.5, 0.125 )
