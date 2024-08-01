#!/bin/bash

# Define the password
password="odroid"

# Define the command
command="sudo MicroXRCEAgent serial --dev /dev/ttyPixhawk -b 921600"

# Execute the command with sudo, providing the password 
echo $password | sudo -S $command
