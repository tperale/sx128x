#!/bin/bash
if [ "$2" = "-c" ]; then
    if [ "$1" = "0" ]; then
        make TARGET=zoul BOARD=remote-revb PORT=/dev/ttyUSB$1 coordinator.upload
    else
        make TARGET=zoul BOARD=remote-revb PORT=/dev/ttyUSB$1 node.upload
    fi
else
    if [ "$1" = "0" ]; then
        make TARGET=zoul BOARD=remote-revb PORT=/dev/ttyUSB$1 MAKE_WITH_TSCH=1 coordinator.upload
    else
        make TARGET=zoul BOARD=remote-revb PORT=/dev/ttyUSB$1 MAKE_WITH_TSCH=1 node.upload
    fi
fi

make TARGET=zoul BOARD=remote-revb PORT=/dev/ttyUSB$1 login
