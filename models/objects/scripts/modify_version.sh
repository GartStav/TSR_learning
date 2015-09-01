#!/bin/bash  

while read line  
do
   echo $line
   gzsdf convert "$line"
done < list_of_files.txt