class PID:

    _dt = 0.025 # min based on deterministic hardware/device of 9600baud
    _error = 0
    _error_previous = 0
    _error_integrated_previous = 0
    p_gain = 1
    i_gain = 0
    d_gain = 0

    def set_pid(self, dt, p, i, d):
        self._dt = dt
        self.p_gain = p
        self.i_gain = i
        self.d_gain = d
        
    def tick(self, set_point, process_value):

        self._error = set_point - process_value
        error_integrated = self._error_integrated_previous + self._error * self._dt # Forward Euler integration
        error_derivative = (self._error - self._error_previous) / self._dt # Finite difference
        
        self._error_previous = self._error
        self._error_integrated_previous = error_integrated

        #print ("error=" + str(self._error))
        
        return self.p_gain * self._error + self.i_gain * error_integrated + self.d_gain * error_derivative

    def get_error(self):
        return self._error
