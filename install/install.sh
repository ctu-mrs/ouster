#!/bin/bash

set -e

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

sudo apt-get -y update
sudo apt-get -y install libglew-dev libglfw3-dev httpie libtclap-dev

num=`cat ~/.bashrc | grep "OUSTER_IP" | wc -l`
if [ "$num" -lt "1" ]; then
  echo 'export OUSTER_IP="10.10.20.90"' >> ~/.bashrc
  echo "adding  OUSTER_IP into .bashrc"
else
  echo ".bashrc already has OUSTER_IP"
fi

num=`cat ~/.bashrc | grep "OUSTER_UDP_DEST_IP" | wc -l`
if [ "$num" -lt "1" ]; then
  echo 'export OUSTER_UDP_DEST_IP="10.10.20.100"' >> ~/.bashrc
  echo "adding  OUSTER_UDP_DEST_IP into .bashrc. Change it to your IP address!"
else
  echo ".bashrc already has OUSTER_UDP_DEST_IP"
fi

num=`cat ~/.zshrc | grep "OUSTER_IP" | wc -l`
if [ "$num" -lt "1" ]; then
  echo 'export OUSTER_IP="10.10.20.90"' >> ~/.zshrc
  echo "adding  OUSTER_IP into .zshrc"
else
  echo ".bashrc already has OUSTER_IP"
fi

num=`cat ~/.zshrc | grep "OUSTER_UDP_DEST_IP" | wc -l`
if [ "$num" -lt "1" ]; then
  echo 'export OUSTER_UDP_DEST_IP="10.10.20.100"' >> ~/.zshrc
  echo "adding  OUSTER_UDP_DEST_IP into .zshrc. Change it to your IP address!"
else
  echo ".zshrc already has OUSTER_UDP_DEST_IP"
fi
