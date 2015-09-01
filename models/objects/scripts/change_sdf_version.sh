#!/bin/bash  

grep -rl "<sdf version=" ../ | xargs sed -i "s/<sdf version=\"1.4/<sdf version=\"1.3/g"
