#!/bin/bash 
iter=9;


while [ $iter -lt 10 ] 
do 


for ((  j = 1 ;  j <= 11;  j++  )) 
do 

#echo "At step :" $iter ", sample :" $j ", trial :"$i 
w_sum=$(./waf --run "scratch/test_7_single_channel_1 $iter $j")
#echo $w_sum
 
done 


(( iter++ )) 
done 

exit 0 