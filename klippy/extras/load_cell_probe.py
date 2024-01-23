# Load Cell Probe 
#
# Copyright (C) 2023 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math
import chelper
from .load_cell import WebhooksApiDumpRepeater
from . import probe
from mcu import TRSYNC_SINGLE_MCU_TIMEOUT, TRSYNC_TIMEOUT, MCU_trsync

######## Types
class BadTapModule(object):
    def is_bad_tap(self, tap_analysis):
        return False

class NozzleCleanerModule(object):
    def clean_nozzle(self):
        pass

class TrapezoidalMove(object):
    def __init__(self, move):
        # copy c data to python memory
        self.print_time = float(move.print_time)
        self.move_t = float(move.move_t)
        self.start_v = float(move.start_v)
        self.accel = float(move.accel)
        self.start_x = float(move.start_x)
        self.start_y = float(move.start_y)
        self.start_z = float(move.start_z)
        self.x_r = float(move.x_r)
        self.y_r = float(move.y_r)
        self.z_r = float(move.z_r)
    def to_dict(self):
        return {'print_time': float(self.print_time),
                'move_t': float(self.move_t),
                'start_v': float(self.start_v),
                'accel': float(self.accel),
                'start_x': float(self.start_x),
                'start_y': float(self.start_y),
                'start_z': float(self.start_z),
                'x_r': float(self.x_r),
                'y_r': float(self.y_r),
                'z_r': float(self.z_r)
        }

# point on a time/force graph
class ForcePoint(object):
    def __init__(self, time, force):
        self.time = time
        self.force = force
    def to_dict(self):
        return {'time': self.time, 'force': self.force}

class ForceLine(object):
    def __init__(self, slope, intercept):
        self.slope = slope
        self. intercept = intercept
    # measure angles between lines at the 1g == 1ms scale
    # returns +/- 0-180. Positive values represent clockwise rotation
    def angle(self, line, time_scale=0.001):
        radians = (math.atan2(self.slope * time_scale, 1) - 
                   math.atan2(line.slope * time_scale, 1))
        return math.degrees(radians)
    def find_force(self, time):
        return self.slope * time + self.intercept
    def find_time(self, force):
        return (force - self.intercept) / self.slope
    def intersection(self, line):
        numerator = -self.intercept + line.intercept
        denominator = self.slope - line.slope
        intersection_time = numerator / denominator
        intersection_force = self.find_force(intersection_time)
        return ForcePoint(intersection_time, intersection_force)
    def to_dict(self):
        return {'slope': self.slope, 'intercept': self.intercept} 

#########################
# Math Support Functions

# compute the index in the time array when some value was surpassed
def index_near(time, instant):
    import numpy as np
    return int(np.argmax(np.asarray(time) >= instant) or len(time) -1)

# Least Squares on x[] y[] points, returns ForceLine
def lstsq_line(x, y):
    import numpy as np
    x = np.asarray(x)
    y = np.asarray(y)
    x_stacked = np.vstack([x, np.ones(len(x))]).T
    mx, b = np.linalg.lstsq(x_stacked, y, rcond=None)[0]
    return ForceLine(mx, b)

# Kneedle algorithm, finds absolute max elbow points
def find_kneedle(x, y):
    import numpy as np
    x_coords = [x[0], x[-1]]
    y_coords = [y[0], y[-1]]
    line = lstsq_line(x_coords, y_coords)
    # now compute the Y of the line value for every x
    fit_y = []
    for time in x:
        fit_y.append(line.find_force(time))
    fit_y = np.asarray(fit_y)
    delta_y = np.absolute(fit_y - y)
    return np.argmax(delta_y)

def lstsq_error(x, y):
    import numpy as np
    x = np.asarray(x)
    y = np.asarray(y)
    x_stacked = np.vstack([x, np.ones(len(x))]).T
    residual = np.linalg.lstsq(x_stacked, y, rcond=None)[1][0]
    return residual

def find_two_lines_best_fit(x, y, search_direction=-1):
    sweep = range(2, len(x) - 2)
    if (search_direction == -1):
        sweep = reversed(sweep)
    min_error = float('inf')
    for i in sweep:
        r1 = lstsq_error(x[0:i], y[0:i])
        r2 = lstsq_error(x[i:], y[i:])
        error = r1 + r2
        if error < min_error:
            min_error = error
        else:
            return i

# split a group of points into 2 lines using 1 elbow point
def split_to_lines(x, y, elbow_index, discard=[0, 0, 0, 0]):
    l1 = lstsq_line(x[0 + discard[0]: elbow_index - discard[1]],
                    y[0 + discard[0]: elbow_index - discard[1]])
    l2 = lstsq_line(x[elbow_index + discard[2]: -1 - discard[3]],
                    y[elbow_index + discard[2]: -1 - discard[3]])
    return (l1, l2)

# split a line into 3 parts by elbows. Return lines and intersection points
def elbow_split(x, y, discard=[0, 0, 0, 0]):
    elbow_index = find_kneedle(x, y)
    l1, l2 = split_to_lines(x, y, elbow_index, discard)
    elbow_point = l1.intersection(l2)
    # return (line, point, line, point, line)
    return l1, elbow_point, l2

# break a tap event down into 6 points and 5 lines:
#    *-----*|       /*-----*
#           |      /
#           *----*/
def tap_decompose(time, force, homing_end_idx, pullback_start_idx,
                  discard=0):
    default_discard = [discard, discard, discard, discard]
    compression_discard = [discard, discard, 0, 0]
    # compression elbow
    start_time = time[0: homing_end_idx]
    start_force = force[0: homing_end_idx]
    # discard=0 because there are so few points in the compression line
    # this technique works better there than kneedle when the probe is long
    contact_elbow_idx = find_two_lines_best_fit(start_time, start_force, -1)
    l1, l2 = split_to_lines(start_time, start_force, contact_elbow_idx,
                            compression_discard)
    p1 = l1.intersection(l2)
    # pullback elbow
    pullback_time = time[pullback_start_idx: -1]
    pullback_force = force[pullback_start_idx: -1]
    l4, p4, l5 = elbow_split(pullback_time, pullback_force, default_discard)
    # dwell line:
    # discard the first 1/5th of the line because it contains ringing
    dwell_start = homing_end_idx + ((pullback_start_idx - homing_end_idx) // 5)
    dwell_time = time[dwell_start: pullback_start_idx]
    dwell_force = force[dwell_start: pullback_start_idx]
    l3 = lstsq_line(dwell_time, dwell_force)
    # dwell line intersections:
    p2 = l3.intersection(l2)
    p3 = l3.intersection(l4)
    p0 = ForcePoint(time[0], l1.find_force(time[0]))
    p5 = ForcePoint(time[-1], l5.find_force(time[-1]))
    return [p0, p1, p2, p3, p4, p5], [l1, l2, l3, l4, l5]

# calculate variance between a ForceLine and a region of force data
def segment_variance(force, time, start, end, line):
    import numpy as np
    mean = np.average(force[start:end])
    total_var = 0
    delta_var = 0
    for i in range(start, end):
        load = force[i]
        instant = time[i]
        total_var += pow(load - mean, 2)
        delta_var += pow(line.find_force(instant) - load, 2)
    return (total_var, delta_var)

# decide how well the ForceLines predict force near an elbow
# bad predictions == blunt elbow, good predictions == sharp elbow
def elbow_r_squared(force, time, elbow_idx, widths, left_line, right_line):
    r_squared = []
    for width in widths:
        l_tv, l_dv = segment_variance(force, time,
                                elbow_idx - width, elbow_idx, left_line)
        r_tv, r_dv = segment_variance(force, time,
                                elbow_idx, elbow_idx + width, right_line)
        r2 = 1 - ((l_dv + r_dv) / (l_tv + r_tv))
        # returns r squared as a percentage. -350% is bad. 80% is good!
        r_squared.append(round((r2 * 100.), 1))
    return r_squared

# TODO: maybe discard points can scale with sample rate from 1 to 3
DEFAULT_DISCARD_POINTS = 3
class TapAnalysis(object):
    def __init__(self, printer, samples, discard=DEFAULT_DISCARD_POINTS):
        import numpy as np
        self.discard = discard
        np_samples = np.array(samples)
        self.time = np_samples[:, 0]
        self.force = np_samples[:, 1]
        self.sample_time = np.average(np.diff(self.time))
        self.r_squared_widths = [int((n * 0.01) // self.sample_time)
                                 for n in range(2, 7)]
        trapq = printer.lookup_object('motion_report').trapqs['toolhead']
        self.moves = self._extract_trapq(trapq)
        self.home_end_time = self._recalculate_homing_end()
        self.pullback_start_time = self.moves[-3].print_time
        self.pullback_end_time = self.moves[-1].print_time + self.moves[-1].move_t
        self.position = self._extract_pos_history()
        self.is_valid = False
        self.tap_pos = None
        self.tap_points = []
        self.tap_lines = []
        self.tap_angles = []
        self.tap_r_squared = None
    # build toolhead position history for the time/force graph
    def _extract_pos_history(self):
        z_pos = []
        for time in self.time:
            z_pos.append(self.get_toolhead_position(time))
        return z_pos
    def get_toolhead_position(self, print_time):
        for i, move in enumerate(self.moves):
            start_time = move.print_time
            # time before first move, printer was stationary
            if i == 0 and print_time < start_time:
                return (move.start_x, move.start_y, move.start_z)
            end_time = float('inf')
            if i < (len(self.moves) - 1):
                end_time = self.moves[i + 1].print_time
            if print_time >= start_time and print_time < end_time:
                # we have found the move
                move_t = move.move_t
                move_time = max(0.,
                        min(move_t, print_time - move.print_time))
                dist = ((move.start_v + .5 * move.accel * move_time)
                            * move_time)
                pos = ((move.start_x + move.x_r * dist,
                        move.start_y + move.y_r * dist,
                        move.start_z + move.z_r * dist))
                return pos
            else:
                continue
        raise Exception("Move not found, that is impossible!")
    # adjust move_t of move 1 to match the toolhead position of move 2
    def _recalculate_homing_end(self):
        # REVIEW: This takes some logical shortcuts, does it need to be more
        # generalized? e.g. to all 3 axes?
        homing_move = self.moves[-5]
        halt_move = self.moves[-4]
        # acceleration should be 0! This is the 'coasting' move:
        accel = homing_move.accel
        if (accel != 0.):
            raise Exception('Unexpected acceleration in coasting move')
        # how long did it take to get to end_z?
        homing_move.move_t = abs((halt_move.start_z - homing_move.start_z)
                                 / homing_move.start_v)
        return homing_move.print_time + homing_move.move_t
    def _extract_trapq(self, trapq):
        moves, _ = trapq.extract_trapq(self.time[0], self.time[-1])
        moves_out = []
        for move in moves:
            moves_out.append(TrapezoidalMove(move))
        if (len(moves_out) < 5 or len(moves_out) > 6):
            raise Exception("Expected tap to be 5 to 6 moves long")
        return moves_out
    def analyze(self):
        import numpy as np
        discard = self.discard
        force = self.force
        time = self.time
        # find peak local maximum force after homing ends:
        home_end_index = index_near(time, self.home_end_time)
        pullback_start_index = index_near(time, self.pullback_start_time)
        # look forward for the next index where force decreases:
        # REVIEW: On my printer it is always true that the calculated 
        # home_end_time is before peak force. Could this not be true for other
        # printers?
        max_force = abs(force[home_end_index])
        peak_force_index = home_end_index
        for i in range(home_end_index + 1, pullback_start_index):
            next_force = abs(force[i])
            if next_force > max_force:
                max_force = next_force
                peak_force_index += 1
            else:
                break
        if not self.validate_peak_force(peak_force_index, home_end_index):
           logging.info('Peak force not close to endstop trigger time')
           return
        pullback_start_index = index_near(time, self.pullback_start_time)
        points, lines = tap_decompose(time, force, peak_force_index,
                            pullback_start_index, discard)
        self.tap_points = points
        self.tap_lines = lines
        if not self.validate_order():
            logging.info('Tap failed chronology check')
            return
        self.tap_angles = self.calculate_angles()
        if not self.validate_elbow_rotation():
            logging.info('Tap failed elbow rotation check')
            return
        break_contact_time = points[4].time
        if not self.validate_break_contact_time(break_contact_time):
            logging.info('Tap break-contact time is invalid')
            return
        self.tap_pos = self.get_toolhead_position(break_contact_time)
        if not self.validate_elbow_clearance():
            logging.info('Elbow too near tap ends')
            return
        self.tap_r_squared = self.calculate_r_squared()
        self.is_valid = True
    # validate peak force within 50ms of homing end
    def validate_peak_force(self, peak_force_index, home_end_index):
        delta = peak_force_index - home_end_index
        delta_t = abs(self.sample_time * delta)
        return delta <= 1 or delta_t < 0.05
    # validate that a set of ForcePoint objects are in chronological order
    def validate_order(self):
        p = self.tap_points
        return (p[0].time < p[1].time < p[2].time
                < p[3].time < p[4].time < p[5].time)
    # Validate that the rotations between lines form a tap shape
    def validate_elbow_rotation(self):
        a1, a2, a3, a4 = self.tap_angles
        # with two polarities there are 2 valid tap shapes:
        return ((a1 > 0 and a2 < 0 and a3 < 0 and a4 > 0) or
                (a1 < 0 and a2 > 0 and a3 > 0 and a4 < 0))
    # check for space around elbows to calculate r_squared
    def validate_elbow_clearance(self):
        width = self.r_squared_widths[-1]
        start_idx = index_near(self.time, self.tap_points[1]) + width
        end_idx = index_near(self.time, self.tap_points[4]) - width
        return start_idx > 0 and end_idx < len(self.time)
    # the propose break contact point must fall inside the pullback move
    def validate_break_contact_time(self, break_contact_time):
        return (self.pullback_start_time < break_contact_time
                < self.pullback_end_time)
    def calculate_angles(self):
        l1, l2, l3, l4, l5 = self.tap_lines
        return [l1.angle(l2), l2.angle(l3), l3.angle(l4), l4.angle(l5)]
    def calculate_r_squared(self):
        r_squared = []
        for i, elbow in enumerate(self.tap_points[1 : -1]):
            elbow_idx = index_near(self.time, elbow.time)
            r_squared.append(elbow_r_squared(self.force, self.time, elbow_idx,
                            self.r_squared_widths,
                            self.tap_lines[i], self.tap_lines[i + 1]))
        return r_squared
    # convert to dictionary for JSON encoder
    def to_dict(self):
        return {
            'time': self.time.tolist(),
            'force': self.force.tolist(),
            'position': self.position,
            'points': [point.to_dict() for point in self.tap_points],
            'lines': [line.to_dict() for line in self.tap_lines],
            'tap_pos': self.tap_pos,
            'moves': [move.to_dict() for move in self.moves],
            'home_end_time': self.home_end_time,
            'pullback_start_time': self.pullback_start_time,
            'pullback_end_time': self.pullback_end_time,
            'tap_angles': self.tap_angles,
            'tap_r_squared': self.tap_r_squared,
            'is_valid': self.is_valid,
        }

WATCHDOG_MAX = 3
MIN_MSG_TIME = 0.100
#LoadCellEndstop implements both McuEndstop and EndstopWrapper
class LoadCellEndstop:
    def __init__(self, config, load_cell):
        self._config = config
        self._config_name = config.get_name()
        self._printer = printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        printer.register_event_handler('klippy:mcu_identify',
                                        self.handle_mcu_identify)
        self._load_cell = load_cell
        self._sensor = sensor = load_cell.get_sensor()
        self._mcu = mcu = sensor.get_mcu()
        self._oid = self._mcu.create_oid()
        sensor.attach_load_cell_endstop(self._oid)
        self.settling_time = config.getfloat('settling_time', default=0.375,
                                             minval=0, maxval=1)
        # Collect 4x60hz power cycles of data to average across power noise
        sps = sensor.get_samples_per_second()
        default_tare_count = max(2, round(sps * ((1 / 60) * 4)))
        self.tare_count = config.getfloat('tare_count',
                            default=default_tare_count, minval=2, maxval=sps)
        # activate/deactivate gcode
        gcode_macro = printer.load_object(config, 'gcode_macro')
        self.position_endstop = config.getfloat('z_offset')
        self.activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')
        # multi probes state
        self.multi = 'OFF'
        self.deactivate_on_each_sample = config.getboolean(
            'deactivate_on_each_sample', True)
        self._home_cmd = self._query_cmd = self._set_range_cmd = None
        self._trigger_completion = None
        ffi_main, ffi_lib = chelper.get_ffi()
        self._trdispatch = ffi_main.gc(ffi_lib.trdispatch_alloc(), ffi_lib.free)
        self._trsyncs = [MCU_trsync(mcu, self._trdispatch)]
        self.trigger_counts = 0
        self.tare_counts = 0
        self.last_trigger_time = 0
        self.trigger_force_grams = config.getfloat('trigger_force_grams'
                    , minval=10., maxval=250, default=50.)
        self.sample_count = config.getint("trigger_count"
            , default=1, minval=1, maxval=5)
        self._mcu.add_config_cmd("config_load_cell_endstop oid=%d"
                                  % (self._oid))
        self._mcu.add_config_cmd("load_cell_endstop_home oid=%d trsync_oid=0" \
            " trigger_reason=0 clock=0 sample_count=0 rest_ticks=0 timeout=0"
            % (self._oid), on_restart=True)
        self._mcu.register_config_callback(self._build_config)
    def _build_config(self):
        # Lookup commands
        cmd_queue = self._trsyncs[0].get_command_queue()
        self._query_cmd = self._mcu.lookup_query_command(
            "load_cell_endstop_query_state oid=%c",
            "load_cell_endstop_state oid=%c homing=%c homing_triggered=%c" \
            " is_triggered=%c trigger_ticks=%u sample=%i sample_ticks=%u",
            oid=self._oid, cq=cmd_queue)
        self._set_range_cmd = self._mcu.lookup_command(
            "set_range_load_cell_endstop oid=%c trigger_counts=%u" \
                " tare_counts=%i"
            , cq=cmd_queue)
        self._home_cmd = self._mcu.lookup_command(
            "load_cell_endstop_home oid=%c trsync_oid=%c trigger_reason=%c" \
            " clock=%u sample_count=%c rest_ticks=%u timeout=%u", cq=cmd_queue)
    def get_status(self, eventtime):
        return {
            'trigger_counts': self.trigger_counts,
            'tare_counts': self.tare_counts,
            'sample_count': self.sample_count,
            'last_trigger_time': self.last_trigger_time
        }
    def _set_endstop_range(self):
        self.tare_counts = int(self._load_cell.get_tare_counts())
        counts_per_gram = self._load_cell.get_counts_per_gram()
        self.trigger_counts = abs(int(self.trigger_force_grams
                                      * counts_per_gram))
        self._set_range_cmd.send([self._oid, self.trigger_counts,
                                self.tare_counts])
    def get_mcu(self):
        return self._mcu
    def get_oid(self):
        return self._oid
    def handle_mcu_identify(self):
        kinematics = self._printer.lookup_object('toolhead').get_kinematics()
        for stepper in kinematics.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)
    def add_stepper(self, stepper):
        trsyncs = {trsync.get_mcu(): trsync for trsync in self._trsyncs}
        trsync = trsyncs.get(stepper.get_mcu())
        if trsync is None:
            trsync = MCU_trsync(stepper.get_mcu(), self._trdispatch)
            self._trsyncs.append(trsync)
        trsync.add_stepper(stepper)
        # Check for unsupported multi-mcu shared stepper rails
        sname = stepper.get_name()
        if sname.startswith('stepper_'):
            for ot in self._trsyncs:
                for s in ot.get_steppers():
                    if ot is not trsync and s.get_name().startswith(sname[:9]):
                        cerror = self._mcu.get_printer().config_error
                        raise cerror("Multi-mcu homing not supported on"
                                     " multi-mcu shared axis")
    def get_steppers(self):
        return [s for trsync in self._trsyncs for s in trsync.get_steppers()]
    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        # not used:
        sample_time = rest_time = triggered = sample_count = None
        # do not permit homing if the load cell is not calibrated
        if not self._load_cell.is_calibrated():
            raise self._printer.command_error("Load Cell not calibrated")
        # re-compute rest_time based on the sample rate
        rest_ticks = self._sensor.get_clock_ticks_per_sample()
        # tare the sensor just before probing
        # this uses pause(), requiring a print_time update
        self.pause_and_tare()
        reactor = self._mcu.get_printer().get_reactor()
        now = reactor.monotonic()
        print_time = self._mcu.estimated_print_time(now) + MIN_MSG_TIME
        clock = self._mcu.print_time_to_clock(print_time)
        self._printer.send_event("load_cell_endstop:homing_start_time"
                                , print_time)
        # copied from endstop.py
        self._trigger_completion = reactor.completion()
        expire_timeout = TRSYNC_TIMEOUT
        if len(self._trsyncs) == 1:
            expire_timeout = TRSYNC_SINGLE_MCU_TIMEOUT
        for trsync in self._trsyncs:
            trsync.start(print_time, self._trigger_completion, expire_timeout)
        etrsync = self._trsyncs[0]
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_start(self._trdispatch, etrsync.REASON_HOST_REQUEST)
        # end copy
        self._home_cmd.send([self._oid, etrsync.get_oid()
            , etrsync.REASON_ENDSTOP_HIT, clock
            , self.sample_count, rest_ticks, WATCHDOG_MAX]
            , reqclock=clock)
        return self._trigger_completion
    def home_wait(self, home_end_time):
        self.home_end_time = home_end_time
        etrsync = self._trsyncs[0]
        etrsync.set_home_end_time(home_end_time)
        if self._mcu.is_fileoutput():
            self._trigger_completion.complete(True)
        self._trigger_completion.wait()
        # trigger has happened, now to find out why...
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trdispatch_stop(self._trdispatch)
        res = [trsync.stop() for trsync in self._trsyncs]
        if any([r == etrsync.REASON_COMMS_TIMEOUT for r in res]):
            return -1.
        if res[0] != etrsync.REASON_ENDSTOP_HIT:
            return 0.
        if self._mcu.is_fileoutput():
            return home_end_time
        params = self._query_cmd.send([self._oid])
        # clear trsync from load_cell_endstop
        self._home_cmd.send([self._oid, 0, 0, 0, 0, 0, 0])
        # The time of the first sample that triggered is in "trigger_ticks"
        trigger_ticks = self._mcu.clock32_to_clock64(params['trigger_ticks'])
        trigger_time = self._mcu.clock_to_print_time(trigger_ticks)
        self.last_trigger_time = trigger_time
        return trigger_time
    def query_endstop(self, print_time):
        clock = self._mcu.print_time_to_clock(print_time)
        if self._mcu.is_fileoutput():
            return 0
        params = self._query_cmd.send([self._oid], minclock=clock)
        if params['homing'] == 1:
            return params['homing_triggered'] == 1
        else:
            return params['is_triggered'] == 1
    # pauses for the last move to complete and then tares the load_cell
    # returns the last sample record used in taring
    def pause_and_tare(self):
        import numpy as np
        toolhead = self._printer.lookup_object('toolhead')
        collector = self._load_cell.get_collector()
        # collect and discard samples up to the end of the current
        collector.collect_until(toolhead.get_last_move_time()
                                + self.settling_time)
        tare_samples = collector.collect_samples(self.tare_count)
        tare_counts = np.average(np.array(tare_samples)[:,2].astype(float))
        self._load_cell.tare(tare_counts)
        self._set_endstop_range()
    def deactivate_probe(self):
        toolhead = self._printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self._printer.command_error(
                "Toolhead moved during probe activate_gcode script")
    def activate_probe(self):
        toolhead = self._printer.lookup_object('toolhead')
        start_pos = toolhead.get_position()
        self.activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self._printer.command_error(
                "Toolhead moved during probe deactivate_gcode script")
    def multi_probe_begin(self):
        if self.deactivate_on_each_sample:
            return
        self.multi = 'FIRST'
    def multi_probe_end(self):
        if self.deactivate_on_each_sample:
            return
        self.deactivate_probe()
        self.multi = 'OFF'
    def probe_prepare(self, hmove):
        if self.multi == 'OFF' or self.multi == 'FIRST':
            self.activate_probe()
            if self.multi == 'FIRST':
                self.multi = 'ON'
    def probe_finish(self, hmove):
        if self.multi == 'OFF':
            self.deactivate_probe()
    def get_position_endstop(self):
        return self.position_endstop

NOZZLE_CLEANER = "{action_respond_info(\"Bad tap detected, nozzle needs" \
        " cleaning. nozzle_cleaner_gcode not configured!\")}"
class LoadCellPrinterProbe(probe.PrinterProbe):
    def __init__(self, config, load_cell, load_cell_endstop):
        logging.info('in LoadCellPrinterProbe constructor')
        super(LoadCellPrinterProbe, self).__init__(config, load_cell_endstop)
        printer = config.get_printer()
        self._load_cell = load_cell
        self._lc_endstop = load_cell_endstop
        self.pullback_dist = config.getfloat('pullback_dist'
                                        , minval=0.01, maxval=2.0, default=0.1)
        sps = self._load_cell.get_sensor().get_samples_per_second()
        default_pullback_speed = sps * 0.001
        #TODO: Math: set the minimum pullback speed such that at least
        # enough samples will be collected
        # e.g. 5 + 1 + (2 * discard)
        self.pullback_speed = config.getfloat('pullback_speed'
                    , minval=0.01, maxval=1.0, default=default_pullback_speed)
        self.pullback_extra_time = config.getfloat('pullback_extra_time'
                    , minval=0.00, maxval=1.0, default=0.3)
        self.pullback_distance = 0.1
        self.bad_tap_module = self.load_module(config
                            , 'bad_tap_module', BadTapModule())
        self.nozzle_cleaner_module = self.load_module(config
                            , 'nozzle_cleaner_module', None)
        gcode_macro = printer.load_object(config, 'gcode_macro')
        self.nozzle_cleaner_gcode = gcode_macro.load_template(config,
                                'nozzle_cleaner_gcode', NOZZLE_CLEANER)
        self.collector = None
        self.printer.register_event_handler(
            "load_cell_endstop:homing_start_time"
            , self._handle_homing_start_time)
        # webhooks support
        name = config.get_name()
        self.api_dump = WebhooksApiDumpRepeater(printer,
            "load_cell_probe/dump_taps", "load_cell_probe", name,
            {"header": ["probe_tap_event"]})
    def load_module(self, config, name, default):
        module = config.get(name, default=None)
        return default if module is None else self.printer.lookup_object(module)
    def _handle_homing_start_time(self, print_time):
        if self.collector is not None:
            self.collector.start_collecting(print_time)
    def _clean_nozzle(self, retries):
        #TODO: what params to pass to nozzle cleaners?
        # X,Y,Z of the failed probe?
        # original requested probe location
        # how many times this has happened?
        if self.nozzle_cleaner_module is not None:
            self.nozzle_cleaner_module.clean_nozzle()
        else:
            macro = self.nozzle_cleaner_gcode
            context = macro.create_template_context()
            context['params'] = {
                'RETRIES': retries,
            }
            macro.run_gcode_from_command(context)
    def _pullback_move(self):
        toolhead = self.printer.lookup_object('toolhead')
        pullback_pos = toolhead.get_position()
        pullback_pos[2] += self.pullback_distance
        toolhead.move(pullback_pos, self.pullback_speed)
        toolhead.flush_step_generation()
        pullback_end = toolhead.get_last_move_time()
        return pullback_end
    # Override
    def _probe(self, speed):
        self.collector = self._load_cell.get_collector()
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        if 'z' not in toolhead.get_status(curtime)['homed_axes']:
            raise self.printer.command_error("Must home before probe")
        phoming = self.printer.lookup_object('homing')
        pos = toolhead.get_position()
        pos[2] = self.z_position
        try:
            epos = phoming.probing_move(self.mcu_probe, pos, speed)
        except self.printer.command_error as e:
            self.collector.stop_collecting()
            self.collector = None
            reason = str(e)
            if "Timeout during endstop homing" in reason:
                reason += probe.HINT_TIMEOUT
            raise self.printer.command_error(reason)
        pullback_end_time = self._pullback_move()
        pullback_end_pos = toolhead.get_position()
        samples = self.collector.collect_until(pullback_end_time
                                               + self.pullback_extra_time)
        self.collector = None
        ppa = TapAnalysis(self.printer, samples)
        ppa.analyze()
        # broadcast tap event data:
        self.api_dump.send({'tap': ppa.to_dict()})
        self._was_bad_tap = True
        if ppa.is_valid:
            self._was_bad_tap = self.bad_tap_module.is_bad_tap(ppa)
            if not self._was_bad_tap:
                epos[2] = ppa.tap_pos[2]
                self.gcode.respond_info("probe at %.3f,%.3f is z=%.6f"
                                % (epos[0], epos[1], epos[2]))
                return epos[:3]
        return pullback_end_pos[:3]
    # Override
    def run_probe(self, gcmd):
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.)
        lift_speed = self.get_lift_speed(gcmd)
        sample_count = gcmd.get_int("SAMPLES", self.sample_count, minval=1)
        sample_retract_dist = gcmd.get_float("SAMPLE_RETRACT_DIST",
                                             self.sample_retract_dist, above=0.)
        samples_tolerance = gcmd.get_float("SAMPLES_TOLERANCE",
                                           self.samples_tolerance, minval=0.)
        samples_retries = gcmd.get_int("SAMPLES_TOLERANCE_RETRIES",
                                       self.samples_retries, minval=0)
        samples_result = gcmd.get("SAMPLES_RESULT", self.samples_result)
        must_notify_multi_probe = not self.multi_probe_pending
        if must_notify_multi_probe:
            self.multi_probe_begin()
        probe_xy = self.printer.lookup_object('toolhead').get_position()[:2]
        retries = 0
        positions = []
        while len(positions) < sample_count:
            # Probe position
            logging.info("probe start")
            probe_pos = self._probe(speed)
            logging.info("probe complete")
            # handle bad taps by cleaning and retry:
            if self._was_bad_tap:
                if retries >= samples_retries:
                    raise gcmd.error("Probe had too many retries")
                gcmd.respond_info("Bad tap detected. Cleaning Nozzle...")
                # tap was invalid or bad, clean nozzle
                self._clean_nozzle(retries)
                gcmd.respond_info("Retrying Probe...")
                retries += 1
            else:
                positions.append(probe_pos)
                # Check samples tolerance
                z_positions = [p[2] for p in positions]
                if max(z_positions) - min(z_positions) > samples_tolerance:
                    if retries >= samples_retries:
                        raise gcmd.error("Probe samples exceed \
                            samples_tolerance")
                    gcmd.respond_info("Probe samples exceed tolerance. \
                        Retrying...")
                    retries += 1
                    positions = []
            # Retract
            if len(positions) < sample_count:
                logging.info("retract move start")
                self._move(probe_xy + [probe_pos[2] + sample_retract_dist],
                            lift_speed)
                logging.info("retract move complete")
        if must_notify_multi_probe:
            self.multi_probe_end()
        # Calculate and return result
        if samples_result == 'median':
            return self._calc_median(positions)
        return self._calc_mean(positions)
    #TODO: add tap info to status
    #def get_status(self, eventtime):
    #    status = super.get_status(eventtime)
    #    return status

def load_config(config):
    printer = config.get_printer()
    load_cell = printer.lookup_object(config.get('load_cell'))
    load_cell_endstop = LoadCellEndstop(config, load_cell)
    printer.add_object('probe',
                    LoadCellPrinterProbe(config, load_cell, load_cell_endstop))
    return load_cell_endstop
