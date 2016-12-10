#!/bin/sh

STATION_FOLDER="/home/pi/WeatherStation"

cd $STATION_FOLDER

UPSTREAM=${1:-'@{u}'} #'
LOCAL=$(git rev-parse @)
REMOTE=$(git rev-parse "$UPSTREAM")
BASE=$(git merge-base @ "$UPSTREAM")

RESTART=false

if [ $LOCAL = $REMOTE ]; then
    echo "Up-to-date"
elif [ $LOCAL = $BASE ]; then
    echo "Need to pull"
    RESTART=true
elif [ $REMOTE = $BASE ]; then
    echo "Need to push"
else
    echo "Diverged"
fi

if [ "$RESTART" = true ]; then
    git pull

    kilall station.py
    sleep 5
    rm $STATION_FOLDER/station.lock
fi

flock $STATION_FOLDER/station.lock python $STATION_FOLDER/station.py