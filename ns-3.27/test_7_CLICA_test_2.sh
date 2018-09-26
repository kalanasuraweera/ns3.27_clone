#!/bin/bash 
iter=0;


while [ $iter -lt 50 ] 
do 


for ((  j = 1 ;  j <= 11;  j++  )) 
do 

for ((  k = 1 ;  k <= 11;  k++  )) 
do 


if ((k >= j+4 || j >= k+4 ))
then

#echo "At step :" $iter ", sample :" $j ", trial :"$i 
w_sum=$(./waf --run "scratch/test_7_CLICA_bash_2 $iter $j $k")
#echo $w_sum

fi

done 
done 


(( iter++ )) 
done 

exit 0 