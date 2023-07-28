#!/bin/bash

sudo bash -c 'echo 1 > /proc/sys/net/ipv4/ip_forward'
sudo nmcli radio wifi on
sudo iptables -t nat -A POSTROUTING -o wlp0s20f3 -j MASQUERADE
sudo iptables -t nat -A POSTROUTING -o eno1 -j MASQUERADE
sudo iptables -A FORWARD -i eno0 -o wlp0s20f3 -j ACCEPT
sudo iptables -A FORWARD -i eno0 -o eno1 -j ACCEPT
sudo iptables -A FORWARD -i wlp0s20f3 -o eno0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i eno1 -o eno0 -m state --state RELATED,ESTABLISHED -j ACCEPT
