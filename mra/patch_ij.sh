#!/bin/bash

# Patch script for Indiana Jones and the Temple of Doom
#
# Prereqs: zip/unzip 
# Roms: atarisy1 and indytemp4 sets, compatible with most recent MAME
#
# Usage: Place romsets in same directory as script. Will generate an
# output archive called "indytemp_patched.zip" which can be used with
# the corresponding .mra file.

# Makes a working directory to stage the ROM dependencies
# (indytemp4, atarisy1 romsets), and unzip to modify them.
mkdir ./indytemp
cp ./indytemp4.zip ./indytemp
cp ./atarisy1.zip ./indytemp
cd ./indytemp
unzip ./indytemp4.zip
unzip ./atarisy1.zip
rm ./indytemp4.zip
rm ./atarisy1.zip
cd ..

# Generate 0xFF padding files used throughout
tr '\0' '\377' < /dev/zero | dd bs=8K count=1 of=./8k_ones.bin
tr '\0' '\377' < /dev/zero | dd bs=16K count=1 of=./16k_ones.bin
tr '\0' '\377' < /dev/zero | dd bs=32K count=1 of=./32k_ones.bin
tr '\0' '\377' < /dev/zero | dd bs=64K count=1 of=./64k_ones.bin

# Files to append 32K 0xFF's
mv ./indytemp/136036.135 ./indytemp/136036.135.raw
cat ./indytemp/136036.135.raw ./32k_ones.bin > ./indytemp/136036.135x
mv ./indytemp/136036.136 ./indytemp/136036.136.raw
cat ./indytemp/136036.136.raw ./32k_ones.bin > ./indytemp/136036.136x
mv ./indytemp/136036.137 ./indytemp/136036.137.raw
cat ./indytemp/136036.137.raw ./32k_ones.bin > ./indytemp/136036.137x
mv ./indytemp/136036.138 ./indytemp/136036.138.raw
cat ./indytemp/136036.138.raw ./32k_ones.bin > ./indytemp/136036.138x
mv ./indytemp/136036.139 ./indytemp/136036.139.raw
cat ./indytemp/136036.139.raw ./32k_ones.bin > ./indytemp/136036.139x
mv ./indytemp/136036.140 ./indytemp/136036.140.raw
cat ./indytemp/136036.140.raw ./32k_ones.bin > ./indytemp/136036.140x
mv ./indytemp/136036.141 ./indytemp/136036.141.raw
cat ./indytemp/136036.141.raw ./32k_ones.bin > ./indytemp/136036.141x
mv ./indytemp/136036.142 ./indytemp/136036.142.raw
cat ./indytemp/136036.142.raw ./32k_ones.bin > ./indytemp/136036.142x
mv ./indytemp/136036.143 ./indytemp/136036.143.raw
cat ./indytemp/136036.143.raw ./32k_ones.bin > ./indytemp/136036.143x
mv ./indytemp/136036.144 ./indytemp/136036.144.raw
cat ./indytemp/136036.144.raw ./32k_ones.bin > ./indytemp/136036.144x
mv ./indytemp/136036.145 ./indytemp/136036.145.raw
cat ./indytemp/136036.145.raw ./32k_ones.bin > ./indytemp/136036.145x
mv ./indytemp/136036.146 ./indytemp/136036.146.raw
cat ./indytemp/136036.146.raw ./32k_ones.bin > ./indytemp/136036.146x
mv ./indytemp/136036.147 ./indytemp/136036.147.raw
cat ./indytemp/136036.147.raw ./32k_ones.bin > ./indytemp/136036.147x
mv ./indytemp/136036.148 ./indytemp/136036.148.raw
cat ./indytemp/136036.148.raw ./32k_ones.bin > ./indytemp/136036.148x
mv ./indytemp/136036.149 ./indytemp/136036.149.raw
cat ./indytemp/136036.149.raw ./32k_ones.bin > ./indytemp/136036.149x
mv ./indytemp/136036.150 ./indytemp/136036.150.raw
cat ./indytemp/136036.150.raw ./32k_ones.bin > ./indytemp/136036.150x

# Append 16K 0xFFs
mv ./indytemp/136032.114.j11 ./indytemp/136032.114.j11.raw
cat ./indytemp/136032.114.j11.raw ./16k_ones.bin > ./indytemp/136032.114.j11x
mv ./indytemp/136032.115.j10 ./indytemp/136032.115.j10.raw
cat ./indytemp/136032.115.j10.raw ./16k_ones.bin > ./indytemp/136032.115.j10x

mv ./indytemp/136036.356 ./indytemp/136036.356.raw
cat ./indytemp/136036.356.raw ./16k_ones.bin > ./indytemp/136036.356x
mv ./indytemp/136036.357 ./indytemp/136036.357.raw
cat ./indytemp/136036.357.raw ./16k_ones.bin > ./indytemp/136036.357x

cp ./8k_ones.bin ./indytemp/
cp ./16k_ones.bin ./indytemp/
cp ./32k_ones.bin ./indytemp/
cp ./64k_ones.bin ./indytemp/

rm ./indytemp/*.raw

# Create output archive
cd ./indytemp/
zip -r  ../indytemp_patched.zip .
cd ..

