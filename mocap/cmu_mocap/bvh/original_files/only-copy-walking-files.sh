#!/bin/bash
# My first script

cat cmu-mocap-index-text.txt | grep walk | awk '{print $1".bvh"}' | grep -vwE "(Subject)" | xargs -I {} cp {} ../{}
