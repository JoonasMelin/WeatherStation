#!/bin/sh

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

if [RESTART]; then
    kilall station.py
    sleep 5
    flock /home/pi/station.lock python /home/pi/WeatherStation/station.py
fi