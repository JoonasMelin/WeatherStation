#!/usr/bin/python
# include RPi libraries in to Python code
import plotly.plotly as py
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

DEVICE = 0x77 # Default device I2C address

#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1 

# instantiate GPIO as an object
GPIO.setmode(GPIO.BOARD)

# define GPIO pins with variables a_pin and b_pin
pull_up_pin = 7
humidity_pin = 15
light_pin = 16

# create discharge function for reading capacitor data
def setup():
    GPIO.setup(pull_up_pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(humidity_pin, GPIO.IN)
    GPIO.setup(light_pin, GPIO.IN)
    time.sleep(0.005)

def read_temperature_from(sensor_id):
    # Open the file that we viewed earlier so that python can see what is in it. Replace the serial number as before. 
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

    print("temp from %s is %s"% (sensor_id, temperature))
    return temperature
    # provide a loop to display analog data count value on the screen




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

def main():
    setup()

    with open('/home/pi/station/.config.json') as config_file:
        plotly_user_config = json.load(config_file)
        py.sign_in(plotly_user_config["plotly_username"], plotly_user_config["plotly_api_key"])

    names = ['Temperature probe 1(F)', 'Temperature probe 2(F)', 'Temperature case(F)', 'Humidity(%)', 'Pressure(mbar)', 'Pressure vs humidity']
    for (token, name) in zip(plotly_user_config['plotly_streaming_tokens'], names):
        print(token)
        url = py.plot([
        {
            'x': [], 'y': [], 'type': 'scatter',
            'stream': {
                'token': token,
                'maxpoints': 20000
            }
        }], filename=name)

        print("View your streaming graph here: %s "% url)
        print("\n\n")

    stream_temp1 = py.Stream(plotly_user_config['plotly_streaming_tokens'][0])
    stream_temp1.open()

    stream_temp2 = py.Stream(plotly_user_config['plotly_streaming_tokens'][1])
    stream_temp2.open()

    stream_temp3 = py.Stream(plotly_user_config['plotly_streaming_tokens'][2])
    stream_temp3.open()

    stream_humidity = py.Stream(plotly_user_config['plotly_streaming_tokens'][3])
    stream_humidity.open()

    stream_pressure = py.Stream(plotly_user_config['plotly_streaming_tokens'][4])
    stream_pressure.open()

    stream_preshum = py.Stream(plotly_user_config['plotly_streaming_tokens'][5])
    stream_preshum.open()

    while True:
        temp_1 = read_temperature_from("28-041663688cff")
        temp_2 = read_temperature_from("28-0316643ddcff")
        

        #(chip_id, chip_version) = readBmp180Id()
        (temperature_pres,pressure)=readBmp180()

        humidity, temperature = Adafruit_DHT.read_retry(11, 22)

        print("Temp1: %s, Temp2: %s, Temp pressure sens: %s, Humidity: %s, Pressure: %s"% (temp_1, temp_2, temperature_pres, humidity, pressure))

        stream_temp1.write({'x': datetime.datetime.now(), 'y': temp_1})
        stream_temp2.write({'x': datetime.datetime.now(), 'y': temp_2})
        stream_temp3.write({'x': datetime.datetime.now(), 'y': temperature})
        stream_humidity.write({'x': datetime.datetime.now(), 'y': humidity})
        stream_pressure.write({'x': datetime.datetime.now(), 'y': pressure})
        stream_preshum.write({'x': temp_1, 'y': humidity})

        time.sleep(2)

if __name__=="__main__":
    main()


