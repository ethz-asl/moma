#!/bin/bash
set -o pipefail

# Update Ubuntu packages to latest.
apt-get -qq update && apt-get -qq upgrade

# update git and set to always point to https
apt-get -qq update && apt-get install -y curl apt-utils
apt-get -qq update && apt-get install -y git git-lfs
git config --global url.https://github.com/.insteadOf git@github.com:
git config --global advice.detachedHead false

# get install tools
apt-get -qq update && apt-get install -y python3-catkin-tools python3-vcstool python3-pip python-is-python3

# Install other system deps
apt-get -qq update && apt-get install -y qtbase5-dev bash-completion

# Clear cache to keep layer size down
rm -rf /var/lib/apt/lists/*
