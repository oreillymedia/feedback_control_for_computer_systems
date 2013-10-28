
import math
import random
import feedback as fb

class GameEngine( fb.Component ):
    def __init__( self ):
        self.n = 0    # Number of game objects
        self.t = 0    # Steps since last change

        self.resolutions = [ 100, 200, 400, 800, 1600 ] # memory per game obj

    def work( self, u ):
        self.t += 1

        if self.t > random.expovariate( 0.1 ):   # 1 chg every 10 steps on avg
            self.t = 0
            self.n += random.choice( [-1,1] ) 
            self.n = max( 1, min( self.n, 50 ) ) # 1 <= n <= 50
        
        crr = self.resolutions[u] # current resolution
        return crr*self.n         # current memory consumption

    def monitoring( self ):
        return "%d" % (self.n,)


class DeadzoneController( fb.Component ):
    def __init__( self, deadzone ):
        self.deadzone = deadzone
    
    def work( self, u ):
        if abs( u ) < self.deadzone:
            return 0
        
        if u < 0: return -1
        else:     return 1


class ConstrainingIntegrator( fb.Component ):
    def __init__( self ):
        self.state = 0

    def work( self, u ):
        self.state += u
        self.state = max(0, min( self.state, 4 ) ) # Constrain to 0..4
        return self.state
    

class Logarithm( fb.Component ):
    def work( self, u ):
        if u <= 0: return 0
        return math.log(u)


if __name__ == '__main__':

    fb.DT = 1

    def setpoint(t):
        return 3.5*math.log( 10.0 )

    c = DeadzoneController( 0.5*math.log(8.0) )
    p = GameEngine()

    fb.closed_loop( setpoint, c, p,actuator=ConstrainingIntegrator(),
                    returnfilter=Logarithm() )

