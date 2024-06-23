#!/bin/bash

# Patch script for Road Runner
#
# Prereqs: zip/unzip 
# Roms: atarisy1 and roadrunn sets, compatible with most recent MAME
#
# Usage: Place romsets in same directory as script. Will generate an
# output archive called "roadrunn_patched.zip" which can be used with
# the corresponding .mra file.

# Make a working directory to stage the ROM dependencies
# (roadrunn, atarisy1 romsets), and unzip to modify them.
mkdir ./roadrunn
cp ./roadrunn.zip ./roadrunn
cp ./atarisy1.zip ./roadrunn
cd ./roadrunn
unzip ./roadrunn.zip
unzip ./atarisy1.zip
rm ./roadrunn.zip
rm ./atarisy1.zip
cd ..

# Generate 0xFF padding files used throughout
tr '\0' '\377' < /dev/zero | dd bs=16K count=1 of=./16k_ones.bin
tr '\0' '\377' < /dev/zero | dd bs=32K count=1 of=./32k_ones.bin

# Append 16K 0xFFs
mv ./roadrunn/136032.114.j11 ./roadrunn/136032.114.j11.raw
cat ./roadrunn/136032.114.j11.raw ./16k_ones.bin > ./roadrunn/136032.114.j11x
mv ./roadrunn/136032.115.j10 ./roadrunn/136032.115.j10.raw
cat ./roadrunn/136032.115.j10.raw ./16k_ones.bin > ./roadrunn/136032.115.j10x

cp ./16k_ones.bin ./roadrunn/
cp ./32k_ones.bin ./roadrunn/

rm ./roadrunn/*.raw

# Create output archive
cd ./roadrunn/
zip -r  ../roadrunn_patched.zip .
cd ..

