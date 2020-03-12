#!/usr/bin/env bash

echo "Source Path: " $1
echo "Destination Path: "$2

if [ $# -ne 2 ]
then
    echo "Expect 2 Arguments; Needs to run as super-user"
    echo "1: Source Path"
    echo "2: Destination Path"
    echo "This script creates symlinks of all files from source path in the destination path if the file/symlink not already exists in the destination path"
    exit -1
fi

for f in $1*; do
    if [ -d $f ];
    then
        echo $f "is a directory"
    else
        # f is a file
        fname=$(echo $f | awk 'BEGIN {FS="/lib/"}{print $2}')
        if echo $fname | grep -q .so;
        then
            destfile=$2$fname
            if [[ -f $destfile ]]
            then
                echo $fname "exists in destination path"
            else
                sudo ln -s $f $destfile
            fi
        else
            # quickhack or okay ??
            echo $fname "is not a dynamik library"
        fi
    fi
done
