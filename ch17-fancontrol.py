
import random
import feedback as fb

class CpuWithCooler( fb.Component ):
    def __init__( self, jumps=False, drift=False ):
        self.ambient = 20             # ambient temperature (in Celsius)
        self.temp    = self.ambient   # initial state: temperature

        self.wattage = 75             # processor heat output in Joule per sec
        self.specific_heat = 1.0/50.0 # specific heat: degree per Joule

        self.loss_factor = 1.0/120.0  # per second

        self.load_wattage_factor = 10 # addtl watts due to load
        self.load_change_seconds = 50 # avg seconds between load changes
        self.current_load = 0
        
        self.ambient_drift = 1.0/3600 # degs per second: 10 degs per 10 hrs

        self.jumps = jumps            # Are there jumps in processor load?
        self.drift = drift            # Is there drift in ambient temp?


    def work( self, u ):
        u = max( 0, min( u, 10 ) )     # Actuator saturation

        self._ambient_drift()          # Drift in ambient temp, if any
        self._load_changes()           # Load changes, if any
        
        diff = self.temp - self.ambient   # Heat loss depends on temp diff
        loss = self.loss_factor*( 1 + u ) # Natural heat loss + fan
        
        flow = self.wattage + self.current_load # Heat inflow to processor

        self.temp += fb.DT*( -loss*diff + self.specific_heat*flow )
        return self.temp


    def _load_changes( self ):
        if self.jumps == False: return

        if random.randint( 0, 2*self.load_change_seconds/fb.DT ) == 0:
            self.current_load = self.load_wattage_factor*random.randint( 0, 5 )

    def _ambient_drift( self ):
        if self.drift == False: return
        
        self.ambient += fb.DT*random.gauss( 0, self.ambient_drift )
        self.ambient = max( 0, min( self.ambient, 40 ) ) # limit drift


    def monitoring( self ):
        return "%f" % ( self.current_load, )

# ============================================================

def no_fan():
    def setpoint(t): return 0
    
    p = CpuWithCooler()
    fb.step_response( setpoint, p, 60000 )    


def min_fan():
    def setpoint(t): return 1
    
    p = CpuWithCooler()
    fb.step_response( setpoint, p, 60000 )    


def measurement( s ):
    def setpoint(t):
        if t<5*60/fb.DT: return 1
        else: return s

    p = CpuWithCooler()
    fb.step_response( setpoint, p, 60000 )    


def production():
    def setpoint(t):
        if t*fb.DT < 6*60: return 50
        else: return 45
#        if t < 40000: return 50
#        else: return 45

    p = CpuWithCooler( True, True ); p.temp = 50 # Initial temp
    c = fb.AdvController( 2, 0.5, 0, clamp=(0,10) )

    fb.closed_loop( setpoint, c, p, 100000, inverted=True,
                    actuator=fb.Limiter( 0, 10 ) )

    
if __name__ == '__main__':

    fb.DT = 0.01

    # no_fan()
    # min_fan()

    # measurement( 5 ) # fan speed: 2, 3, 4, 5
    production()
    

