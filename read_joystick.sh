#!/bin/bash 

# Exit if we encounter an error
set -e
jstest --event /dev/input/js0 | grep -m 1 "type 1, time .*, number .*, value 1" | read 
