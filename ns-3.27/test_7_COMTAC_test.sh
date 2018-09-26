#!/bin/bash 
iter=0;


while [ $iter -lt 50 ] 
do 


for ((  j = 1 ;  j <= 11;  j++  )) 
do 

#echo "At step :" $iter ", sample :" $j ", trial :"$i 
w_sum=$(./waf --run "scratch/test_7_COMTAC_pre_bash $iter $j")
#echo $w_sum

done 


(( iter++ )) 
done 

exit 0 