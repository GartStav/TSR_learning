#!/bin/bash

function is_a_number( )
{
    re='^[0-9]+([.][0-9]+)?$'
    if ! [[ $1 =~ $re ]] ; then
	echo "error: Not a number" >&2; exit 1
    fi
}

echo "usage ./scale_models.sh $1 $2"
echo " where $1 is the old scaling"
echo " and $2 is the new scaling"

echo "SCALE from $1 to $2"
is_a_number $1
echo "test argument 1 is a number"
is_a_number $2
echo "test argument 2 is a number"

grep -rl "<scale>$1 $1 $1" ../ | xargs sed -i "s/<scale>$1 $1 $1/<scale>$2 $2 $2/g"

echo "exit normal"
