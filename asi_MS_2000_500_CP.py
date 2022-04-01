import serial

class Controller:
    '''
    Basic device adaptor for ASI MS-2000-500-CP multi-axis stage controller.
    See 'ASI_controller_labels.pdf' for explanation of model name. Many more
    commands are available and have not been implemented.
    '''
    def __init__(self,
                 which_port,                # COM port for controller
                 axes=None,                 # ('X','Y'), ('Z',), ('X','Y','Z')
                 lead_screws=None,          # 'UC', 'SC', 'S', 'F', 'XF' (tuple)
                 axes_min_mm=None,          # min range (tuple)
                 axes_max_mm=None,          # max range (tuple)
                 encoder_counts_per_um=None,# optional -> expert only (tuple)
                 use_pwm=False,             # optional pwm output for LED
                 name='MS-2000-500-CP',     # optional name
                 verbose=True,              # False for max speed
                 very_verbose=False):       # True for debug
        self.name = name
        self.verbose = verbose
        self.very_verbose = very_verbose
        if self.verbose: print('%s: Opening...'%name, end='')
        try:
            self.port = serial.Serial(
                port=which_port, baudrate=9600, timeout=5)
        except serial.serialutil.SerialException:
            raise IOError('%s: No connection on port %s'%(name, which_port))
        if self.verbose: print(" done.")
        self.version = self._send('V').strip(':A \r\n')
        assert self.version == 'Version: USB-9.2k', (
            '%s: controller version not supported'%name)
        self._set_ttl_in_mode('disabled')
        self._set_ttl_out_mode('low')
        self.state = None
        if axes is not None:
            assert axes == ('X','Y') or axes == ('Z',) or axes == ('X','Y','Z')
            self.axes = axes
            assert lead_screws is not None, 'please choose lead screw options'
            assert len(lead_screws) == len(axes)
            screw2value = { # pitch (mm), res (nm), speed (mm/s)
                'UC':(25.40, 88.0, 28.0),   # 'ultra-course'
                'SC':(12.70, 44.0, 14.0),   # 'super-course'
                'S' :(6.350, 22.0, 7.00),   # 'standard'
                'F' :(1.590, 5.50, 1.75),   # 'fine'
                'XF':(0.653, 2.20, 0.70)}   # 'extra-fine'
            pitch_mm, resolution_nm, max_velocity_mmps = [], [], []
            for a in range(len(axes)):
                pitch_mm.append(screw2value[lead_screws[a]][0])
                resolution_nm.append(screw2value[lead_screws[a]][1])
                max_velocity_mmps.append(screw2value[lead_screws[a]][2])
            self.pitch_mm = tuple(pitch_mm)
            self.resolution_nm = tuple(resolution_nm)
            self.max_velocity_mmps = tuple(max_velocity_mmps)
            assert axes_min_mm is not None, 'please specify min range of axes'
            assert axes_max_mm is not None, 'please specify max range of axes'
            assert len(axes_min_mm) == len(axes)
            assert len(axes_max_mm) == len(axes)
            for v in axes_min_mm: assert type(v) is int or type(v) is float
            for v in axes_max_mm: assert type(v) is int or type(v) is float
            self.min_position_um = tuple(1e3 * v for v in axes_min_mm)
            self.max_position_um = tuple(1e3 * v for v in axes_max_mm)
            self.encoder_counts_per_um = len(axes)*(10,)# default value
            if encoder_counts_per_um is not None:
                assert len(encoder_counts_per_um) == len(axes)
                for v in encoder_counts_per_um: assert type(v) is int
                self.encoder_counts_per_um = encoder_counts_per_um
            self.min_acceleration_ms = len(axes)*(25,)  # min acc/dec ramp
            self.max_acceleration_ms = len(axes)*(1e3,) # max acc/dec ramp
            self.max_settle_time_ms = len(axes)*(1e3,)  # max pause after move
            self.tol_settle_time_ms = len(axes)*(1,)    # tolerance
            self.min_precision_um = len(axes)*(1,)      # high precision limit
            self.max_precision_um = len(axes)*(1e6,)    # low precision limit
            self._set_velocity(tuple(0.67 * v for v in self.max_velocity_mmps))
            self._set_acceleration(self.min_acceleration_ms)
            self._set_settle_time(len(axes)*(0,))
            self._set_precision(self.min_precision_um)
            self._get_position()
            self._moving = False
        if use_pwm:
            self.set_pwm_state('off')
            self.set_pwm_intensity(1)
        return None

    def _send(self, cmd, respond=True, parse_axes=False):
        if self.very_verbose:
            print("%s: sending cmd = "%self.name, cmd)
        assert type(cmd) is str, 'command should be a string'
        cmd = bytes(cmd, encoding='ascii')
        self.port.write(cmd + b'\r')
        response = self.port.readline().decode('ascii').strip(':A \r\n')
        if respond:
            assert response != '', '%s: No response'%self.name
            if parse_axes:
                axes, values = [], []
                for a in response.split():
                    axis, value = a.split('=')
                    axes.append(axis), values.append(float(value))
                assert tuple(axes) == self.axes
                response = tuple(values)
        else:
            response = None
        if self.very_verbose:
            print("%s: -> response = "%self.name, response)
        assert self.port.in_waiting == 0
        return response

    def _get_ttl_in_mode(self):
        if self.verbose:
            print("%s: getting ttl in mode"%self.name)
        code2mode = {'0': 'disabled', '10': 'toggle_ttl_out'}
        code = self._send('TTL X?').rstrip().split('=')[1]
        self._ttl_in_mode = code2mode[code]
        if self.verbose:
            print("%s: -> ttl in mode = %s"%(self.name, self._ttl_in_mode))
        return self._ttl_in_mode

    def _set_ttl_in_mode(self, mode):
        if self.verbose:
            print("%s: setting ttl in mode = %s"%(self.name, mode))
        mode2code = {'disabled':'0', 'toggle_ttl_out':'10'}
        assert mode in mode2code, "mode '%s' not allowed"%mode
        self._ttl_in_mode = self._send(
            'TTL X=%s'%mode2code[mode], respond=False)
        assert self._get_ttl_in_mode() == mode
        if self.verbose:
            print("%s: -> done setting ttl in mode."%self.name)
        return None

    def _get_ttl_out_mode(self):
        if self.verbose:
            print("%s: getting ttl out mode"%self.name)
        code2mode = {'0': 'low', '1': 'high', '9': 'pwm'}
        code = self._send('TTL Y?').rstrip().split('=')[1]
        self._ttl_out_mode = code2mode[code]
        if self.verbose:
            print("%s: -> ttl out mode = %s"%(self.name, self._ttl_out_mode))
        return self._ttl_out_mode

    def _set_ttl_out_mode(self, mode):
        if self.verbose:
            print("%s: setting ttl out mode = %s"%(self.name, mode))
        mode2code = {'low':'0', 'high':'1', 'pwm':'9'}
        assert mode in mode2code, "mode '%s' not allowed"%mode
        self._ttl_out_mode = self._send(
            'TTL Y=%s'%mode2code[mode], respond=False)
        assert self._get_ttl_out_mode() == mode
        if self.verbose:
            print("%s: -> done setting ttl out mode."%self.name)
        return None

    def _get_velocity(self):
        if self.verbose:
            print("%s: getting velocity"%self.name)
        self.velocity_mmps = self._send(
            'S '+'? '.join(self.axes)+'?', parse_axes=True)
        if self.verbose:
            print("%s: -> velocity (mm/s) = %s"%(self.name, self.velocity_mmps))
        return self.velocity_mmps

    def _set_velocity(self, velocity_mmps): # tuple i.e. (2, 5, None)
        if self.verbose:
            print("%s: setting velocity = %s"%(self.name, velocity_mmps))
        assert len(velocity_mmps) == len(self.axes)
        for v in velocity_mmps: assert type(v) is int or type(v) is float
        cmd_string = ['S ']
        velocity_mmps = list(velocity_mmps)
        for i, v in enumerate(velocity_mmps):
            if v is None:
                velocity_mmps[i] = self.velocity_mmps[i]
            assert 0 <= velocity_mmps[i] <= self.max_velocity_mmps[i]
            velocity_mmps[i] = round(velocity_mmps[i], 6)
            cmd_string.append('V%s=%0.6f '%(self.axes[i], velocity_mmps[i]))
        self._send(''.join(cmd_string), respond=False)
        assert self._get_velocity() == tuple(velocity_mmps)
        if self.verbose:
            print("%s: -> done setting velocity."%self.name)
        return None

    def _get_acceleration(self): # acceleration or deceleration ramp time (ms)
        if self.verbose:
            print("%s: getting acceleration"%self.name)
        self.acceleration_ms = self._send(
            'AC '+'? '.join(self.axes)+'?', parse_axes=True)
        if self.verbose:
            print("%s: -> acceleration (ms) = %s"%(
                self.name, self.acceleration_ms))
        return self.acceleration_ms

    def _set_acceleration(self, acceleration_ms): # tuple i.e. (2, 5, None)
        if self.verbose:
            print("%s: setting acceleration = %s"%(self.name, acceleration_ms))
        assert len(acceleration_ms) == len(self.axes)
        for v in acceleration_ms: assert type(v) is int or type(v) is float
        cmd_string = ['AC ']
        acceleration_ms = list(acceleration_ms)
        for i, a in enumerate(acceleration_ms):
            if a is None:
                acceleration_ms[i] = self.acceleration_ms[i]
            acceleration_ms[i] = round(acceleration_ms[i])
            assert acceleration_ms[i] >= self.min_acceleration_ms[i]
            assert acceleration_ms[i] <= self.max_acceleration_ms[i]
            cmd_string.append('AC%s=%0.6f '%(self.axes[i], acceleration_ms[i]))
        self._send(''.join(cmd_string), respond=False)
        assert self._get_acceleration() == tuple(acceleration_ms)
        if self.verbose:
            print("%s: -> done setting acceleration."%self.name)
        return None

    def _get_settle_time(self): # time spent at end of move (ms)
        if self.verbose:
            print("%s: getting settle time"%self.name)
        self.settle_time_ms = self._send(
            'WT '+'? '.join(self.axes)+'?', parse_axes=True)
        if self.verbose:
            print("%s: -> settle time (ms) = %s"%(
                self.name, self.settle_time_ms))
        return self.settle_time_ms

    def _set_settle_time(self, settle_time_ms): # tuple i.e. (2, 5, None)
        if self.verbose:
            print("%s: setting settle time = %s"%(self.name, settle_time_ms))
        assert len(settle_time_ms) == len(self.axes)
        for v in settle_time_ms: assert type(v) is int or type(v) is float
        cmd_string = ['WT ']
        settle_time_ms = list(settle_time_ms)
        for i, t in enumerate(settle_time_ms):
            if t is None:
                settle_time_ms[i] = self.settle_time_ms[i]
            settle_time_ms[i] = round(settle_time_ms[i])
            assert 0 <= settle_time_ms[i] <= self.max_settle_time_ms[i]
            cmd_string.append('WT%s=%0.6f '%(self.axes[i], settle_time_ms[i]))
        self._send(''.join(cmd_string), respond=False)
        self._get_settle_time()
        for i, (ti, tf) in enumerate(zip(settle_time_ms, self.settle_time_ms)):
            assert tf >= ti - self.tol_settle_time_ms[i]
            assert tf <= ti + self.tol_settle_time_ms[i]
        if self.verbose:
            print("%s: -> done setting settle_time."%self.name)
        return None

    def _get_precision(self): # acceptable error between target and actual (um)
        if self.verbose:
            print("%s: getting precision"%self.name)
        precision_mm = self._send(
            'PC '+'? '.join(self.axes)+'?', parse_axes=True)
        self.precision_um = tuple(round(1e6 * p) for p in precision_mm)
        if self.verbose:
            print("%s: -> precision (um) = %s"%(
                self.name, self.precision_um))
        return self.precision_um

    def _set_precision(self, precision_um): # tuple i.e. (2, 5, None)
        if self.verbose:
            print("%s: setting precision = %s"%(self.name, precision_um))
        assert len(precision_um) == len(self.axes)
        for v in precision_um: assert type(v) is int or type(v) is float
        cmd_string = ['PC ']
        precision_um = list(precision_um)
        for i, p in enumerate(precision_um):
            if p is None:
                precision_um[i] = self.precision_um[i]
            precision_um[i] = round(precision_um[i])
            assert precision_um[i] >= self.min_precision_um[i]
            assert precision_um[i] <= self.max_precision_um[i]
            cmd_string.append(
                'PC%s=%0.6f '%(self.axes[i], 1e-6 * precision_um[i]))
        self._send(''.join(cmd_string), respond=False)
        assert self._get_precision() == tuple(precision_um)
        if self.verbose:
            print("%s: -> done setting precision."%self.name)
        return None

    def _counts2position(self, counts):
        position_um = []
        for count, factor in zip(counts, self.encoder_counts_per_um):
            position_um.append(float(count) / factor)
        return tuple(position_um)

    def _position2counts(self, position_um):
        counts = []
        for position, factor in zip(position_um, self.encoder_counts_per_um):
            counts.append(round(position * factor))
        return tuple(counts)

    def _get_position(self):
        if self.verbose:
            print("%s: getting position"%self.name)
        response = self._send('W '+' '.join(self.axes)).strip(':A \r\n').split()
        self.position_um = self._counts2position(response)
        if self.verbose:
            print("%s: -> position (um) = %s"%(self.name, self.position_um))
        return self.position_um

    def _finish_moving(self):
        if not self._moving:
            return None
        while True:
            status = self._send('/')
            if status == 'N':
                break
        self._get_position()
        for i, p in enumerate(self.position_um):
            assert p >= self._target_move_um[i] - self.precision_um[i]
            assert p <= self._target_move_um[i] + self.precision_um[i]
        self._moving = False
        if self.verbose: print('%s: -> finished moving'%self.name)
        return None

    def move_um(self, move_um, relative=True, block=True):
        self._finish_moving()
        assert len(move_um) == len(self.axes)
        for v in move_um: assert type(v) is int or type(v) is float or v is None
        move_um = list(move_um)
        for i, m in enumerate(move_um):
            if m is None:
                move_um[i] = self.position_um[i]
            if m is not None and relative:
                move_um[i] = self.position_um[i] + move_um[i]
            assert move_um[i] >= self.min_position_um[i]
            assert move_um[i] <= self.max_position_um[i]
        move_um = tuple(round(v, 3) for v in move_um) # round to nm
        if self.verbose:
            print("%s: moving to (um) = %s"%(self.name, move_um))
        cmd_string = ['M ']
        for i in range(len(self.axes)):
            cmd_string.append('%s=%0.6f '%(self.axes[i],
                                           self._position2counts(move_um)[i]))
        self._send(''.join(cmd_string), respond=False)
        self._moving = True
        self._target_move_um = move_um
        if block:
            self._finish_moving()
        return None

    def get_pwm_intensity(self):
        if self.verbose:
            print("%s: getting pwm intensity"%self.name)
        response = self._send('LED X?') # non-standard answer: 'X=val :A'
        self.pwm_intensity = int(response.split('=')[1])
        if self.verbose:
            print("%s: -> pwm intensity (%%) = %s"%(
                self.name, self.pwm_intensity))
        return self.pwm_intensity

    def set_pwm_intensity(self, intensity):
        if self.verbose:
            print("%s: setting pwm intensity = %s"%(self.name, intensity))
        assert type(intensity) is int or type(intensity) is float
        intensity = int(intensity)
        assert 1 <= intensity <= 99
        self._send('LED X=%d'%intensity, respond=False)
        assert self.get_pwm_intensity() == intensity
        if self.verbose:
            print("%s: -> done setting pwm intensity."%self.name)
        return None

    def set_pwm_state(self, state):
        if self.verbose:
            print("%s: setting pwm state = %s"%(self.name, state))
        assert state in ('off', 'on', 'pwm', 'external')
        if state == 'off':
            self._set_ttl_in_mode('disabled')
            self._set_ttl_out_mode('low')
        if state == 'on':
            self._set_ttl_in_mode('disabled')
            self._set_ttl_out_mode('high')
        if state == 'pwm':
            self._set_ttl_in_mode('disabled')
            self._set_ttl_out_mode('pwm')
        if state == 'external':
            self._set_ttl_in_mode('toggle_ttl_out')
            self._set_ttl_out_mode('low')
        self.state = state
        if self.verbose:
            print("%s: -> done setting pwm state."%self.name)
        return None

    def close(self):
        if self.verbose: print("%s: closing..."%self.name)
        if self.state != 'off':
            self.set_pwm_state('off')
        self.port.close()
        if self.verbose: print("%s: closed."%self.name)
        return None

if __name__ == '__main__':
    port = 'COM3'
    # controller only:
    ms = Controller(which_port=port, verbose=True, very_verbose=True)
    ms.close()

    # controller + xy stage:
    ms = Controller(which_port=port,
                    axes=('X', 'Y'),
                    lead_screws=('S','S'),
                    axes_min_mm=(-50,-25),  # recommended to check and use!
                    axes_max_mm=( 50, 25),  # recommended to check and use!
                    )
    ms.move_um((0, 0), relative=False)      # home
    ms.close()

    # controller + z drive:
    ms = Controller(which_port=port,
                    axes=('Z',),
                    lead_screws=('F',),
                    axes_min_mm=(-10,),     # recommended to check and use!
                    axes_max_mm=( 10,),     # recommended to check and use!
                    )
    ms.move_um((0,), relative=False)        # home
    ms.close()

    # controller + pwm:
    ms = Controller(which_port=port, use_pwm=True)
    ms.set_pwm_state('on')
##    input('hit enter to continue')
    ms.close()

    # controller + xys stage + pwm:
    ms = Controller(which_port=port,
                    axes=('X', 'Y', 'Z'),
                    lead_screws=('S','S','F'),
                    axes_min_mm=(-50,-25,-10), # recommended to check and use!
                    axes_max_mm=( 50, 25, 10), # recommended to check and use!
                    use_pwm=True,
                    verbose=True,
                    very_verbose=False)
    # move stage:
    ms.move_um((0, 0, 0), relative=False)       # home
    for moves in range(3):
        ms.move_um((2000, 1000, None))          # relative moves
    ms.move_um((-1000, None, -50), block=False) # relative non-blocking
    print('do something else...')
    ms.move_um((0, 0, 0), relative=False)       # re-home (absolute)
    # use pwm:
    ms.set_pwm_state('on')
    ms.set_pwm_state('pwm')
    for intensity in range(1, 100, 10):
        ms.set_pwm_intensity(intensity)
    ms.set_pwm_state('external') # -> supply external pwm on ttl in
    ms.close()

    # random input testing:
    ms = Controller(which_port=port,
                    axes=('X', 'Y', 'Z'),
                    lead_screws=('S','S','F'),
                    axes_min_mm=(-50,-25,-10), # recommended to check and use!
                    axes_max_mm=( 50, 25, 10), # recommended to check and use!
                    use_pwm=True,
                    verbose=True,
                    very_verbose=False)

    from random import uniform as r
    iterations = 1 # tested to 10,000 without crash/fail
    min_vel = 1
    factor = 10
    min_pos = tuple(p / factor for p in ms.min_position_um)
    max_pos = tuple(p / factor for p in ms.max_position_um)
    for i in range(iterations):
        print('Test_A:',i)
        vel, acc, stl, pre, pos = [], [], [], [], []
        for a in range(len(ms.axes)):
            vel.append(r(min_vel, ms.max_velocity_mmps[a]))
            acc.append(r(ms.min_acceleration_ms[a], ms.max_acceleration_ms[a]))
            stl.append(r(0, ms.max_settle_time_ms[a]))
            pre.append(r(ms.min_precision_um[a], ms.max_precision_um[a]))
            pos.append(r(min_pos[a], max_pos[a]))
        ms._set_velocity(tuple(vel))
        ms._set_acceleration(tuple(acc))
        ms._set_settle_time(tuple(stl))
        ms._set_precision(tuple(pre))
        ms.move_um(tuple(pos), relative=False)

    ms._set_velocity(ms.max_velocity_mmps)          # max speed
    ms._set_acceleration(ms.min_acceleration_ms)    # fast acceration
    ms._set_settle_time(len(ms.axes)*(0,))          # no settle time
    ms._set_precision(ms.min_precision_um)          # max precision
    for i in range(iterations):
        print('Test_B:',i)
        pos = []
        for a in range(len(ms.axes)):
            pos.append(r(min_pos[a], max_pos[a]))
        ms.move_um(tuple(pos), relative=False)
    ms.move_um((0, 0, 0), relative=False)
    ms.close()
