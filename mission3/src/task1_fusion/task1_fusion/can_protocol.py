"""Configurable, single-frame CAN acquisition decoder; no guessed car protocol."""
import math
import struct
import numpy as np
from .math_utils import se2_exp, wrap

TYPES = {'int16': 'h', 'uint16': 'H', 'int32': 'i', 'uint32': 'I',
         'float32': 'f', 'float64': 'd', 'uint8': 'B', 'int8': 'b'}


def signal(payload, field, byte_order):
    code = TYPES[field['type']]
    offset = int(field['offset'])
    if offset < 0 or offset + struct.calcsize(code) > len(payload):
        raise ValueError('CAN payload shorter than configured field')
    value = struct.unpack_from(('<' if byte_order == 'little' else '>') + code, payload, offset)[0]
    out = value * field.get('scale', 1.0) + field.get('bias', 0.0)
    if not math.isfinite(out):
        raise ValueError('nonfinite CAN signal')
    return out


def modular_delta(current, previous, bits):
    modulus = 1 << bits
    return (int(current)-int(previous)+modulus//2) % modulus - modulus//2


class Decoder:
    def __init__(self, config):
        if config.get('protocol_confirmed') is not True:
            raise ValueError('CAN protocol is unconfirmed; fill real fields and set protocol_confirmed: true')
        self.cfg = config
        self.mode = config['mode']
        if self.mode not in ('body_twist', 'differential_counts'):
            raise ValueError('supported modes: body_twist, differential_counts')
        self.previous_counts = None
        self.previous_stamp = None
        self.previous_counter = None
        self.counter_time = None
        self.previous_boot = None
        self.pose = np.zeros(3)
        if config.get('byte_order', 'little') not in ('little', 'big'):
            raise ValueError('invalid byte order')
        if self.mode == 'differential_counts':
            if min(config['meters_per_count_left'], config['meters_per_count_right'], config['track_width_m']) <= 0:
                raise ValueError('wheel scales and track width must be positive')

    def decode(self, payload, received_ros_sec):
        c = self.cfg
        if 'payload_length' in c and len(payload) != c['payload_length']:
            raise ValueError('CAN payload length differs from confirmed protocol')
        boot = None
        if c.get('boot_id'):
            boot = signal(payload, c['boot_id'], c.get('byte_order', 'little'))
            if self.previous_boot is not None and boot != self.previous_boot:
                raise ValueError('MCU boot ID changed; restart decoder and fusion')
        fields = c['fields']
        read = lambda k: signal(payload, fields[k], c.get('byte_order', 'little'))
        stamp = received_ros_sec - c.get('receive_latency_sec', 0.0)
        tick = c.get('timestamp_counter')
        counter_time = self.counter_time
        if tick:
            counter = signal(payload, tick, c.get('byte_order', 'little'))
            if self.previous_counter is None:
                counter_time = stamp
            else:
                d = modular_delta(counter, self.previous_counter, tick['bits'])
                if d <= 0:
                    raise ValueError('duplicate/reversed MCU timestamp; reboot needs decoder restart')
                counter_time += d * tick['seconds_per_tick']
            stamp = counter_time
        dt = None if self.previous_stamp is None else stamp-self.previous_stamp
        if dt is not None and (dt <= 0 or dt > c.get('max_gap_sec', 0.1)):
            raise ValueError('CAN timestamp gap; restart decoder and fusion')
        if stamp > received_ros_sec + 0.05 or received_ros_sec-stamp > c.get('max_receive_age_sec', 0.1):
            raise ValueError('MCU/ROS clock mapping drift or excessive receive delay')
        if self.mode == 'body_twist':
            velocity = np.array([read('vx'), read('vy'), read('wz')])
            counts = None
        else:
            counts = (read('left_count'), read('right_count'))
            if self.previous_counts is None:
                velocity = np.zeros(3)
            else:
                bits = c.get('count_bits', 32)
                dl = modular_delta(counts[0], self.previous_counts[0], bits)*c['meters_per_count_left']*c.get('left_sign', 1)
                dr = modular_delta(counts[1], self.previous_counts[1], bits)*c['meters_per_count_right']*c.get('right_sign', 1)
                forward = (dl+dr)/2/dt
                wz = (dr-dl)/c['track_width_m']/dt
                axis = c.get('forward_axis', 'y')
                if axis not in ('x', 'y'):
                    raise ValueError('forward_axis must be x or y')
                velocity = np.array([forward if axis == 'x' else 0.0,
                                     forward if axis == 'y' else 0.0, wz])
        if math.hypot(*velocity[:2]) > c.get('max_speed_mps', 5.0) or abs(velocity[2]) > c.get('max_yaw_rate', 10.0):
            raise ValueError('implausible wheel increment; check protocol or MCU reset')
        if dt is not None:
            delta = se2_exp(*velocity, dt)
            co, si = math.cos(self.pose[2]), math.sin(self.pose[2])
            self.pose[:2] += [co*delta[0]-si*delta[1], si*delta[0]+co*delta[1]]
            self.pose[2] = wrap(self.pose[2]+delta[2])
        # Commit only after all validations passed.
        self.previous_counts, self.previous_stamp = counts, stamp
        self.previous_boot = boot
        if tick:
            self.previous_counter = counter
            self.counter_time = counter_time
        return stamp, self.pose.copy(), velocity
