# WeatherStation
Open source weatherstation developed in IoT style. Publishes data to PlotLy and local web server.

# Sensors
The station is based on Rasberry Pi Zero platform, which means that almost any digital sensor can be easily integrated into the platform. With additional ADC modules, analog sensors could also be included.

## Temperature
The current version has two temperature probes (more can be easily added). The probes are based on DS18b20 digital sensor and should be accurate within 0.5C in -10C to 85C range.

## Humidity
Humidity is measured insie the case and utilises DHT11 digital temperature sensor. This sensor is accurate to 5% with in 20%-80% range.

## Pressure
The pressure is measured with BMP180 which is capable of sensing pressure changes as low as 0.02 hPa, which means that altitude changes of 0.17m could be detected. The pressure sensor is also placed inside the case.

# Usage

The weatherstation should be paired with a corresponding mini wifi router so that it automatically connects to it. Once connected, it will automatically pull the latest code version from this GitHub repository and start publishing data to its own webserver and to plotly.

## Viewing the data

