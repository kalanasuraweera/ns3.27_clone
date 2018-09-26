#!/bin/bash 
iter=0;


while [ $iter -lt 50 ] 
do 

#echo "At step :" $iter ", sample :" $j ", trial :"$i 
w_sum=$(./waf --run "scratch/test_7_CCA_new $iter")
#echo $w_sum


(( iter++ )) 
done 

exit 0 