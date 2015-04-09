#!/bin/bash

ADDR="192.168.1.2"
SUBNET="24"
BROADCAST_ADDR="192.168.1.255"
INTERFACE="enp0s25"

# enable the interface
sudo ip link set $INTERFACE up

# cofigure the network interface
sudo ip addr add $ADDR/$SUBNET broadcast $BROADCAST_ADDR dev $INTERFACE
