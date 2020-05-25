#!/usr/bin/env python
"""Generates the pins file for the TM4C123."""

from __future__ import print_function

import argparse
import sys
import csv


SUPPORTED_AFS = {
    'UART'	: ('TX', 'RX', 'RTS', 'CTS'), #UART
    'SSI'	: ('CLK', 'TX', 'RX', 'FSS'), #SPI
    'I2C'	: ('SDA', 'SCL'),
    'TIM'	: ('CCP0', 'CCP1'), #16 bit Timer
    'WTIM' 	: ('CCP0', 'CCP1'), #32 bit Wide Timer
    'MTRL' 	: ('PWM0', 'PWM1', 'PWM2', 'PWM3', 'PWM4', 'PWM5','PWM6','PWM7', 'FAULT0'), # Motion Control
    'ADC'	: ( 'AIN0','AIN1','AIN2','AIN3','AIN4','AIN5','AIN6','AIN7','AIN8','AIN9','AIN10', 			'AIN11'),
    'COMP'	: ('NEG','POS', 'OUT' ), # Analog Comparator
    'QEI'	: ('PHA0', 'PHA1', 'PHB0', 'PHB1', 'IDX0', 'IDX1'), # Quadrature Encoder Interface
    'TR'	: ('CLK', 'D0', 'D1'), # Trace
    'CAN'	: ('TX', 'RX'),
    'NMI'	: (''),
    'JTAG'	: ('TDO', 'SWO', 'TDI', 'TMS', 'SWDIO', 'TCK', 'SWCLK'),
    'USB'	: ('DM', 'DP', 'EPEN', 'ID', 'PFLT', 'VBUS')
}
SINGLE_UNIT_AF = ('NMI','TR') # These do not have Unit numbers

NO_PREFIX_UNIT_AF = ('ADC', 'QEI', 'JTAG') # these units dont have the unit type in the af name

AF_SHORT_DICT = {
    'UART'  : 'U',
    'TIM'   : 'T',
    'WTIM'  : 'WT',
    'MTRL'  : 'M',
    'COMP'  : 'C'
}	# some af names are shortened in the datasheet and driverlib


def parse_port_pin(name_str):
    """Parses a string and returns a (port, port_pin) tuple."""
    if len(name_str) < 3:
        raise ValueError("Expecting pin name to be at least 3 characters")
    if name_str[0] != 'P':
        raise ValueError("Expecting pin name to start with P")
    if name_str[1] < 'A' or name_str[1] > 'F':
        raise ValueError("Expecting pin port to be between A and F")
    port = name_str[1]
    pin_str = name_str[2:]
    if not pin_str.isdigit():
        raise ValueError("Expecting numeric pin number.")
    return (port, int(pin_str))


class AF:
    """Holds the description of an alternate function"""
    def __init__(self, name, idx, fn, unit, type, pin_name):
        self.name = name
        self.idx = idx
        """AF from 0 to 9 and 14 to 15"""
        if self.idx > 15 or (10 <= self.idx <= 13):
            self.idx = -1
        self.fn = fn
        self.unit = unit
        self.type = type
        self.pin_name = pin_name
        if fn in AF_SHORT_DICT:
            self.short = AF_SHORT_DICT[fn] + str(unit)
        elif fn in NO_PREFIX_UNIT_AF:
            self.short = ''
        elif unit < 0 :
            self.short = fn
        else:
            self.short = fn + str(unit)

    def print(self):
        if self.idx == 0:
            print ('    AF_AN({:14s}, {:4d}, {:4s}, {:4d}, {:6s}),    // {}'.format(self.name, self.idx, self.fn, self.unit, self.type, self.name))
        elif self.short == '':
            print ('    AF_SL({:14s}, {:4d}, {:4s}, {:4d}, {:12s}, {:3s}),    // {}'.format(self.name, self.idx, self.fn, self.unit, self.type, self.pin_name, self.name))
        else:
            print ('    AF_ML({:14s}, {:4d}, {:4s}, {:4d}, {:6s}, {:4s}, {:3s}),    // {}'.format(self.name, self.idx, self.fn, self.unit, self.type, self.short, self.pin_name, self.name))


class Pin:
    """Holds the information associated with a pin."""
    def __init__(self, name, port, port_pin, def_af, pin_num):
        self.name = name
        self.port = port
        self.port_pin = port_pin
        self.pin_num = pin_num
        self.def_af = def_af
        self.board_pin = False
        self.afs = []

    def add_af(self, af):
        self.afs.append(af)

    def print(self):
        print('// {}'.format(self.name))
        if len(self.afs):
            print('const pin_af_obj_t pin_{}_af[] = {{'.format(self.name))
            for af in self.afs:
                af.print()
            print('};')
            print('const pin_obj_t pin_{:3s}_obj = PIN({:6s}, {}, {:3d}, {:2d}, pin_{}_af, {}, {});\n'.format(
                self.name, self.name, self.port, self.port_pin, self.pin_num, self.name, self.def_af, len(self.afs)))
        else:
            print('const pin_obj_t pin_{:3s}_obj = PIN({:6s}, {}, {:3d}, {:2d}, NULL, 0, 0);\n'.format(
                self.name, self.name, self.port, self.port_pin, self.pin_num))

    def print_header(self, hdr_file):
        hdr_file.write('extern const pin_obj_t pin_{:3s}_obj;\n#define pin_{:3s} (&pin_{:3s}_obj)\n\n'.format(self.name,self.name,self.name))


class Pins:
    def __init__(self):
        self.board_pins = []   # list of pin objects

    def find_pin(self, port, port_pin):
        for pin in self.board_pins:
            if pin.port == port and pin.port_pin == port_pin:
                return pin

    def find_pin_by_num(self, pin_num):
        for pin in self.board_pins:
            if pin.pin_num == pin_num:
                return pin

    def find_pin_by_name(self, name):
        for pin in self.board_pins:
            if pin.name == name:
                return pin

    def parse_af_file(self, filename, pin_col, pinname_col, defaf_col, af_start_col):
        with open(filename, 'r') as csvfile:
            rows = csv.reader(csvfile)
            for row in rows:
                try:
                    (port_num, port_pin) = parse_port_pin(row[pinname_col])
                except:
                    continue
                if not row[pin_col].isdigit():
                    raise ValueError("Invalid pin number {:s} in row {:s}".format(row[pin_col], row))
                pin_num = int(row[pin_col])
                # find the default af
                if row[defaf_col] != '' and row[defaf_col] != row[pinname_col]:
                    for cell in row[af_start_col:]:
                        if cell == row[defaf_col]:
                            def_af = row[af_start_col:].index(cell)
                            break
                else:
                    def_af = 0
                pin = Pin(row[pinname_col], port_num, port_pin, def_af, pin_num)
                self.board_pins.append(pin)
                af_idx = 0
                for af in row[af_start_col:]:
                    af_splitted = af.split('_')
                    fn_name = af_splitted[0].rstrip('0123456789')
                    if  fn_name in SUPPORTED_AFS:
                        try:
                            type_name = af_splitted[1]
                        except:
                            type_name = ''
                        if type_name in SUPPORTED_AFS[fn_name]:
                            if fn_name in SINGLE_UNIT_AF: # Dont have numbers
                                unit_idx = -1
                            elif fn_name in NO_PREFIX_UNIT_AF:
                                unit_idx = -1
                            else:
                                unit_idx = af_splitted[0][-1]
                            pin.add_af(AF(af, af_idx, fn_name, int(unit_idx), type_name, pin.name))
                    af_idx += 1

    def parse_board_file(self, filename, cpu_pin_col):
        with open(filename, 'r') as csvfile:
            rows = csv.reader(csvfile)
            for row in rows:
                if row[cpu_pin_col].isdigit():
                    pin = self.find_pin_by_num(int(row[cpu_pin_col]))
                else:
                    pin = self.find_pin_by_name(row[cpu_pin_col])
                if pin:
                    pin.board_pin = True

    def print_named(self, label, pins):
        print('')
        print('STATIC const mp_rom_map_elem_t pin_{:s}_pins_locals_dict_table[] = {{'.format(label))
        for pin in pins:
            if pin.board_pin:
                print('    {{ MP_ROM_QSTR(MP_QSTR_{:3s}), MP_ROM_PTR(pin_{:3s}) }},'.format(pin.name, pin.name))
        print('};')
        print('MP_DEFINE_CONST_DICT(pin_{:s}_pins_locals_dict, pin_{:s}_pins_locals_dict_table);'.format(label, label))

    def print(self):
        for pin in self.board_pins:
            if pin.board_pin:
                pin.print()
        self.print_named('board', self.board_pins)
        self.print_named('cpu', self.board_pins)
        print('')

    def print_header(self, hdr_filename):
        with open(hdr_filename, 'wt') as hdr_file:
            for pin in self.board_pins:
                if pin.board_pin:
                    pin.print_header(hdr_file)

    def print_qstr(self, qstr_filename):
        with open(qstr_filename, 'wt') as qstr_file:
            pin_qstr_set = set([])
            af_qstr_set = set([])
            for pin in self.board_pins:
                if pin.board_pin:
                    pin_qstr_set |= set([pin.name])
                    for af in pin.afs:
                        af_qstr_set |= set([af.name])
            print('// Board pins', file=qstr_file)
            for qstr in sorted(pin_qstr_set):
                print('Q({})'.format(qstr), file=qstr_file)
            print('\n// Pin AFs', file=qstr_file)
            for qstr in sorted(af_qstr_set):
                print('Q({})'.format(qstr), file=qstr_file)


def main():
    parser = argparse.ArgumentParser(
        prog="make-pins.py",
        usage="%(prog)s [options] [command]",
        description="Generate board specific pin file"
    )
    parser.add_argument(
        "-a", "--af",
        dest="af_filename",
        help="Specifies the alternate function file for the chip",
        default="tm4c123_af.csv"
    )
    parser.add_argument(
        "-b", "--board",
        dest="board_filename",
        help="Specifies the board file",
    )
    parser.add_argument(
        "-p", "--prefix",
        dest="prefix_filename",
        help="Specifies beginning portion of generated pins file",
        default="tm4c123_prefix.c"
    )
    parser.add_argument(
        "-q", "--qstr",
        dest="qstr_filename",
        help="Specifies name of generated qstr header file",
        default="build/pins_qstr.h"
    )
    parser.add_argument(
        "-r", "--hdr",
        dest="hdr_filename",
        help="Specifies name of generated pin header file",
        default="build/pins.h"
    )
    args = parser.parse_args(sys.argv[1:])

    pins = Pins()

    print('// This file was automatically generated by make-pins.py')
    print('//')
    if args.af_filename:
        print('// --af {:s}'.format(args.af_filename))
        pins.parse_af_file(args.af_filename, 0, 1, 2, 3)

    if args.board_filename:
        print('// --board {:s}'.format(args.board_filename))
        pins.parse_board_file(args.board_filename, 1)

    if args.prefix_filename:
        print('// --prefix {:s}'.format(args.prefix_filename))
        print('')
        with open(args.prefix_filename, 'r') as prefix_file:
            print(prefix_file.read())
    pins.print()
    pins.print_qstr(args.qstr_filename)
    pins.print_header(args.hdr_filename)


if __name__ == "__main__":
    main()
