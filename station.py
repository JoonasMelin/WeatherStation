#!/usr/bin/python
# include RPi libraries in to Python code
import plotly.plotly as py
import plotly.graph_objs as go
import json
import datetime

import RPi.GPIO as GPIO
import time

import smbus
import time
from ctypes import c_short

import sys
import Adafruit_DHT

import cPickle as pickle
from collections import deque

import time
import threading
from functools import wraps

import numpy as np

import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.dates as dates

import scipy
from scipy import signal

DEVICE = 0x77 # Default device I2C address

#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1 

# instantiate GPIO as an object
GPIO.setmode(GPIO.BOARD)

# define GPIO pins with variables a_pin and b_pin
pull_up_pin = 7
humidity_pin = 15
light_pin = 16


class RingBuffer(object):
    def __init__(self, size_max, default_value=0.0, dtype=np.float16):
        """initialization"""
        self.size_max = size_max

        self._data = np.empty(size_max, dtype=dtype)
        self._data.fill(default_value)

        self.size = 0

    def append(self, value):
        """append an element"""
        self._data = np.roll(self._data, 1)
        self._data[0] = value 

        self.size += 1

        if self.size == self.size_max:
            self.__class__  = RingBufferFull

    def get_all(self):
        """return a list of elements from the oldest to the newest"""
        return(self._data)

    def get_partial(self):
        return(self.get_all()[0:self.size])

    def get_up_to(self, max_elems):
        elem_to = max_elems
        if max_elems > self.size:
            elem_to = self.size
        return(self.get_all()[0:elem_to])


    def __getitem__(self, key):
        """get element"""
        return(self._data[key])

    def __repr__(self):
        """return string representation"""
        s = self._data.__repr__()
        s = s + '\t' + str(self.size)
        s = s + '\t' + self.get_all()[::-1].__repr__()
        s = s + '\t' + self.get_partial()[::-1].__repr__()
        return(s)

class RingBufferFull(RingBuffer):
    def append(self, value):
        """append an element when buffer is full"""
        self._data = np.roll(self._data, 1)
        self._data[0] = value

# create discharge function for reading capacitor data
def setup():
    GPIO.setup(pull_up_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(humidity_pin, GPIO.IN)
    GPIO.setup(light_pin, GPIO.IN)
    time.sleep(2)

def read_temperature_from(sensor_id):
    # Open the file that we viewed earlier so that python can see what is in it. Replace the serial number as before.
    temperature = 0
    try:
        tfile = open(("/sys/bus/w1/devices/%s/w1_slave"%sensor_id)) 
        # Read all of the text in the file. 
        text = tfile.read() 
        # Close the file now that the text has been read. 
        tfile.close() 
        # Split the text with new lines (\n) and select the second line. 
        secondline = text.split("\n")[1] 
        # Split the line into words, referring to the spaces, and select the 10th word (counting from 0). 
        temperaturedata = secondline.split(" ")[9] 
        # The first two characters are "t=", so get rid of those and convert the temperature from a string to a number. 
        temperature = float(temperaturedata[2:]) 
        # Put the decimal point in the right place and display it. 
        temperature = temperature / 1000
    except:
        print("Could not read sensor:", sys.exc_info()[0])

    #print("temp from %s is %s"% (sensor_id, temperature))
    return temperature

def convertToString(data):
    # Simple function to convert binary data into
    # a string
    return str((data[1] + (256 * data[0])) / 1.2)

def getShort(data, index):
    # return two bytes from data as a signed 16-bit value
    return c_short((data[index]<< 8) + data[index + 1]).value

def getUshort(data, index):
    # return two bytes from data as an unsigned 16-bit value
    return (data[index]<< 8) + data[index+1] 

def readBmp180Id(addr=DEVICE):
    # Register Address
    REG_ID     = 0xD0

    (chip_id, chip_version) = bus.read_i2c_block_data(addr, REG_ID, 2)
    return (chip_id, chip_version)

def readBmp180(addr=DEVICE):
    # Register Addresses
    REG_CALIB  = 0xAA
    REG_MEAS   = 0xF4
    REG_MSB    = 0xF6
    REG_LSB    = 0xF7
    # Control Register Address
    CRV_TEMP   = 0x2E
    CRV_PRES   = 0x34 
    # Oversample setting
    OVERSAMPLE = 3    # 0 - 3

    # Read calibration data
    # Read calibration data from EEPROM
    cal = bus.read_i2c_block_data(addr, REG_CALIB, 22)

    # Convert byte data to word values
    AC1 = getShort(cal, 0)
    AC2 = getShort(cal, 2)
    AC3 = getShort(cal, 4)
    AC4 = getUshort(cal, 6)
    AC5 = getUshort(cal, 8)
    AC6 = getUshort(cal, 10)
    B1  = getShort(cal, 12)
    B2  = getShort(cal, 14)
    MB  = getShort(cal, 16)
    MC  = getShort(cal, 18)
    MD  = getShort(cal, 20)

    # Read temperature
    bus.write_byte_data(addr, REG_MEAS, CRV_TEMP)
    time.sleep(0.005)
    (msb, lsb) = bus.read_i2c_block_data(addr, REG_MSB, 2)
    UT = (msb << 8) + lsb

    # Read pressure
    bus.write_byte_data(addr, REG_MEAS, CRV_PRES + (OVERSAMPLE << 6))
    time.sleep(0.04)
    (msb, lsb, xsb) = bus.read_i2c_block_data(addr, REG_MSB, 3)
    UP = ((msb << 16) + (lsb << 8) + xsb) >> (8 - OVERSAMPLE)

    # Refine temperature
    X1 = ((UT - AC6) * AC5) >> 15
    X2 = (MC << 11) / (X1 + MD)
    B5 = X1 + X2
    temperature = (B5 + 8) >> 4

    # Refine pressure
    B6  = B5 - 4000
    B62 = B6 * B6 >> 12
    X1  = (B2 * B62) >> 11
    X2  = AC2 * B6 >> 11
    X3  = X1 + X2
    B3  = (((AC1 * 4 + X3) << OVERSAMPLE) + 2) >> 2

    X1 = AC3 * B6 >> 13
    X2 = (B1 * B62) >> 16
    X3 = ((X1 + X2) + 2) >> 2
    B4 = (AC4 * (X3 + 32768)) >> 15
    B7 = (UP - B3) * (50000 >> OVERSAMPLE)

    P = (B7 * 2) / B4

    X1 = (P >> 8) * (P >> 8)
    X1 = (X1 * 3038) >> 16
    X2 = (-7357 * P) >> 16
    pressure = P + ((X1 + X2 + 3791) >> 4)

    return (temperature/10.0,pressure/ 100.0)


def make_stream(stamps, y_data, name, token, max_data_points):
    print(token)
    # Plot all the history data as well
    
    url = py.plot([
    {
        'x': stamps, 'y': y_data, 'type': 'scatter',
        'stream': {
            'token': token,
            'maxpoints': max_data_points
        }
    }], filename=name, auto_open=False)

    print("View your streaming graph here: %s "% url)
    print("\n\n")

    return url

def open_streams(plotly_user_config, names, data, max_data_points):
    print("Attempting to open the streams to plotly")
    sys.stdout.flush()

    py.sign_in(plotly_user_config["plotly_username"], plotly_user_config["plotly_api_key"])

    stamps = list(data['stamps'])
    data_len = data['temp1'].get_up_to(max_data_points).size

    if len(stamps) < data_len:
        data_len = len(stamps)

    stamps = stamps[0:data_len]

    tokens = plotly_user_config['plotly_streaming_tokens']

    #print(list(data['temp1'].get_partial()))
    #sys.stdout.flush()

    url_temp1 = make_stream(stamps, list(data['temp1'].get_up_to(data_len)), names[0], tokens[0], max_data_points)
    url_temp2 = make_stream(stamps, list(data['temp2'].get_up_to(data_len)), names[1], tokens[1], max_data_points)
    url_temp3 = make_stream(stamps, list(data['temp3'].get_up_to(data_len)), names[2], tokens[2], max_data_points)
    url_humidity = make_stream(stamps, list(data['humidity'].get_up_to(data_len)), names[3], tokens[3], max_data_points)
    url_pressure = make_stream(stamps, list(data['pressure'].get_up_to(data_len)), names[4], tokens[4], max_data_points)

    stream_list = []
    for token in plotly_user_config['plotly_streaming_tokens']:
        cur_stream = py.Stream(token)
        cur_stream.open()
        stream_list.append(cur_stream)

    return stream_list

def print_data_to_html(data):
    stamps = list(data['stamps'])
    time_ax = dates.date2num(stamps)
    hfmt = dates.DateFormatter('%m/%d %H:%M')

    filt_l = 9

    plt.figure(figsize=(15, 15), dpi=300)
    ax1 = plt.subplot(511)
    ax1.set_title("Temperature 1 and 2")
    ax1.xaxis.set_major_locator(dates.HourLocator())
    ax1.xaxis.set_major_formatter(hfmt)
    plt.plot(time_ax, scipy.signal.medfilt(data['temp1'].get_partial(),filt_l), 'b-', time_ax, scipy.signal.medfilt(data['temp2'].get_partial(),filt_l), 'r-')

       

    ax2 = plt.subplot(512)
    ax2.set_title("Case temperature")
    ax2.xaxis.set_major_locator(dates.HourLocator())
    ax2.xaxis.set_major_formatter(hfmt)
    plt.plot(time_ax, scipy.signal.medfilt(data['temp3'].get_partial(),filt_l), 'b-')

    ax3 = plt.subplot(513)
    ax3.set_title("Humidity")
    ax3.xaxis.set_major_locator(dates.HourLocator())
    ax3.xaxis.set_major_formatter(hfmt)
    plt.plot(time_ax, scipy.signal.medfilt(data['humidity'].get_partial(),filt_l), 'b-')

    ax4 = plt.subplot(514)
    ax4.set_title("Pressure")
    ax4.xaxis.set_major_locator(dates.HourLocator())
    ax4.xaxis.set_major_formatter(hfmt)
    plt.plot(time_ax, scipy.signal.medfilt(data['pressure'].get_partial(),filt_l), 'b-')

    ax5 = plt.subplot(515)
    ax5.set_title("Humidity vs pressure")
    plt.xticks(rotation='vertical')
    plt.plot(data['humidity'].get_partial(), data['pressure'].get_partial(), 'rx')

    plt.savefig("/var/www/html/data.png")

    #with open("/var/www/html/index.html", 'w+') as output:
        #mpld3.save_html(plt, output)


def main():
    print("Setting up the sensors")
    setup()

    print("Starting the weatherstation")

    data_dump_file = "/home/pi/data.dump"

    #max_data_points = 15000000 # Roughly 4 samples/min to keep a years worth of data
    #max_data_points = 40000
    max_data_points = 150000
    max_data_points_plot = 10000

    resolution_secs = 5

    with open('/home/pi/station/.config.json') as config_file:
        plotly_user_config = json.load(config_file)

    # Loading the data from the disk if it exists
    initialise_data = True
    if os.path.isfile(data_dump_file):
        print("Loading the data from the disk dump")
        try:
            with open(data_dump_file, 'rb') as input:
                data = pickle.load(input)
                if data['maxlen'] == max_data_points:
                    print("Data has the same amount of data points, not initializing")
                    initialise_data=False
        except:
            print("Loading data failed, re-initialising")
            initialise_data=True
            
    if initialise_data:
        print("Re-initializing the data")
        data = {}
        data['stamps'] = deque([], maxlen=max_data_points) 
        data['temp1'] = RingBuffer(size_max=max_data_points)
        data['temp2'] = RingBuffer(size_max=max_data_points)
        data['temp3'] = RingBuffer(size_max=max_data_points)
        data['humidity'] = RingBuffer(size_max=max_data_points)
        data['pressure'] = RingBuffer(size_max=max_data_points)
        data['maxlen'] = max_data_points

    names = ['Temperature probe 1(F)', 'Temperature probe 2(F)', 'Temperature case(F)', 'Humidity(%)', 'Pressure(mbar)', 'Pressure vs humidity']
    streams = []
    successfully_opened = False
    last_call = datetime.datetime.fromtimestamp(0)
    last_save_call = datetime.datetime.now()
    last_read_call = datetime.datetime.fromtimestamp(0)


    while True:
        # Checking if we should dump the data to disk
        duration_since_last_read = datetime.datetime.now() - last_read_call

        # Throttling the data read rate
        while duration_since_last_read.total_seconds() < resolution_secs:
            duration_since_last_read = datetime.datetime.now() - last_read_call
            time.sleep(0.5)

        last_read_call = datetime.datetime.now()
            

        temp_1 = read_temperature_from("28-041663688cff")
        temp_2 = read_temperature_from("28-0316643ddcff")
        

        #(chip_id, chip_version) = readBmp180Id()
        (temperature_pres,pressure)=readBmp180()

        humidity, temperature = Adafruit_DHT.read_retry(11, 22)

        print("Temp1: %s, Temp2: %s, Temp pressure sens: %s, Humidity: %s, Pressure: %s"% (temp_1, temp_2, temperature_pres, humidity, pressure))

        # Saving all the data to the queue
        data['stamps'].append(datetime.datetime.now())
        data['temp1'].append(temp_1)
        data['temp2'].append(temp_2)
        data['temp3'].append(temperature)
        data['humidity'].append(humidity)
        data['pressure'].append(pressure)

        if not successfully_opened:
            # Ratelimiting the calls to the service
            duration_since_last = datetime.datetime.now() - last_call

            if duration_since_last.total_seconds() < 60*60:
                print("Not requesting the streams yet")
            else:
                print("%s seconds has elapsed, trying the streams again"%duration_since_last.total_seconds())
                last_call = datetime.datetime.now()

                try:
                    streams = open_streams(plotly_user_config, names, data, max_data_points_plot)
                    successfully_opened = True
                except:
                    print("Could not open the streams:", sys.exc_info()[0])

        else:
            try:
                print("Writing the info to the streams")
                streams[0].write({'x': datetime.datetime.now(), 'y': temp_1})
                streams[1].write({'x': datetime.datetime.now(), 'y': temp_2})
                streams[2].write({'x': datetime.datetime.now(), 'y': temperature})
                streams[3].write({'x': datetime.datetime.now(), 'y': humidity})
                streams[4].write({'x': datetime.datetime.now(), 'y': pressure})
                streams[5].write({'x': temp_1, 'y': humidity})
            except:
                print("Could not print to streams:", sys.exc_info()[0])
                successfully_opened = False

        # Checking if we should dump the data to disk
        duration_since_last_save = datetime.datetime.now() - last_save_call

        if duration_since_last_save.total_seconds() < 60:
            print("Not saving the data yet")
        else:
            print("Saving the data to disk")
            print_data_to_html(data)
            last_save_call = datetime.datetime.now()
            with open(data_dump_file, 'w+') as output:
                pickle.dump(data, output, pickle.HIGHEST_PROTOCOL)


        time.sleep(0.5)

if __name__=="__main__":
    main()


