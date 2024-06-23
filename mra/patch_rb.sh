#!/bin/bash

# Patch script for RoadBlasters
#
# Prereqs: zip/unzip 
# Roms: atarisy1 and roadblst sets, compatible with most recent MAME
#
# Usage: Place romsets in same directory as script. Will generate an
# output archive called "roadblst_patched.zip" which can be used with
# the corresponding .mra file.

# Make a working directory to stage the ROM dependencies
# (roadblst, atarisy1 romsets), and unzip to modify them.
mkdir ./roadblst
cp ./roadblst.zip ./roadblst
cp ./atarisy1.zip ./roadblst
cd ./roadblst
unzip ./roadblst.zip
unzip ./atarisy1.zip
rm ./roadblst.zip
rm ./atarisy1.zip
cd ..

# Generate 0xFF padding files used throughout
tr '\0' '\377' < /dev/zero | dd bs=16K count=1 of=./16k_ones.bin
tr '\0' '\377' < /dev/zero | dd bs=32K count=1 of=./32k_ones.bin

# Append 16K 0xFFs
mv ./roadblst/136032.114.j11 ./roadblst/136032.114.j11.raw
cat ./roadblst/136032.114.j11.raw ./16k_ones.bin > ./roadblst/136032.114.j11x
mv ./roadblst/136032.115.j10 ./roadblst/136032.115.j10.raw
cat ./roadblst/136032.115.j10.raw ./16k_ones.bin > ./roadblst/136032.115.j10x

cp ./16k_ones.bin ./roadblst/
cp ./32k_ones.bin ./roadblst/

rm ./roadblst/*.raw

# Create output archive
cd ./roadblst/
zip -r  ../roadblst_patched.zip .
cd ..

