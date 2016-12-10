#!/bin/sh

STATION_FOLDER="/home/pi/WeatherStation"

cd $STATION_FOLDER

MD5_ORIG=$(md5sum $STATION_FOLDER/station.py)

git pull

MD5_PULL=$(md5sum $STATION_FOLDER/station.py)

if [ $MD5_PULL !=  $MD5_ORIG ]; then
    echo "Killing the process"
    pkill -f station.py
    sleep 5
    rm $STATION_FOLDER/station.lock
fi

echo "Attempting to start the process with flock"
flock $STATION_FOLDER/station.lock python $STATION_FOLDER/station.py