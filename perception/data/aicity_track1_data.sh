#!/bin/bash

mkdir track1-data/
cd track1-data 
wget http://www.aicitychallenge.org/wp-content/uploads/Shuo/2021/track1/qjqvbz1/AIC21_Track1_Vehicle_Counting.zip
wget http://www.aicitychallenge.org/wp-content/uploads/Shuo/2021/track1/qjqvbz1/AIC21_Track1_Vehicle_Counting.z01 
wget http://www.aicitychallenge.org/wp-content/uploads/Shuo/2021/track1/qjqvbz1/AIC21_Track1_Vehicle_Counting.z02 
wget http://www.aicitychallenge.org/wp-content/uploads/Shuo/2021/track1/qjqvbz1/AIC21_Track1_Vehicle_Counting.z03 
wget http://www.aicitychallenge.org/wp-content/uploads/Shuo/2021/track1/qjqvbz1/AIC21_Track1_Vehicle_Counting.z04  
wget http://www.aicitychallenge.org/wp-content/uploads/Shuo/2021/track1/qjqvbz1/AIC21_Track1_Vehicle_Counting.z05
wget http://www.aicitychallenge.org/wp-content/uploads/2021/03/efficiency_base.py_.zip 

echo("----- download completed -----")

zip -FF AIC21_Track1_Vehicle_Counting.zip --out AIC21_Track1_Vehicle_Counting_full.zip 
rm AIC21_Track1_Vehicle_Counting.z*

mkdir AIC21_Track1_Vehicle_Counting 
cd AIC21_Track1_Vehicle_Counting 
unzip -FF ../AIC21_Track1_Vehicle_Counting_full.zip 

echo("----- unzip completed -----")
