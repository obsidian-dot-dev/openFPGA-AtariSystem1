#!/bin/bash

# Patch script for Marble Madness
#
# Prereqs: zip/unzip 
# Roms: atarisy1 and marble5 sets, compatible with most recent MAME
#
# Usage: Place romsets in same directory as script. Will generate an
# output archive called "marble_patched.zip" which can be used with
# the corresponding .mra file.

# Make a working directory to stage the ROM dependencies
# (marble5, atarisy1 romsets), and unzip to modify them.
mkdir ./marble
cp ./marble5.zip ./marble
cp ./atarisy1.zip ./marble
cd ./marble
unzip ./marble5.zip
unzip ./atarisy1.zip
rm ./marble5.zip
rm ./atarisy1.zip
cd ..

# Generate 0xFF padding files used throughout
tr '\0' '\377' < /dev/zero | dd bs=8K count=1 of=./8k_ones.bin
tr '\0' '\377' < /dev/zero | dd bs=16K count=1 of=./16k_ones.bin
tr '\0' '\377' < /dev/zero | dd bs=32K count=1 of=./32k_ones.bin
tr '\0' '\377' < /dev/zero | dd bs=64K count=1 of=./64k_ones.bin

# Files to append 32K 0xFF's
mv ./marble/136033.109 ./marble/136033.109.raw
cat ./marble/136033.109.raw ./32k_ones.bin > ./marble/136033.109x
mv ./marble/136033.110 ./marble/136033.110.raw
cat ./marble/136033.110.raw ./32k_ones.bin > ./marble/136033.110x
mv ./marble/136033.111 ./marble/136033.111.raw
cat ./marble/136033.111.raw ./32k_ones.bin > ./marble/136033.111x
mv ./marble/136033.112 ./marble/136033.112.raw
cat ./marble/136033.112.raw ./32k_ones.bin > ./marble/136033.112x
mv ./marble/136033.113 ./marble/136033.113.raw
cat ./marble/136033.113.raw ./32k_ones.bin > ./marble/136033.113x

# Prepend 16K, append 32K 0xFFs
mv ./marble/136033.115 ./marble/136033.115.raw
cat ./16k_ones.bin ./marble/136033.115.raw ./32k_ones.bin > ./marble/136033.115x
mv ./marble/136033.116 ./marble/136033.116.raw
cat ./16k_ones.bin ./marble/136033.116.raw ./32k_ones.bin > ./marble/136033.116x
mv ./marble/136033.117 ./marble/136033.117.raw
cat ./16k_ones.bin ./marble/136033.117.raw ./32k_ones.bin > ./marble/136033.117x

# Append 16K 0xFFs
mv ./marble/136032.114.j11 ./marble/136032.114.j11.raw
cat ./marble/136032.114.j11.raw ./16k_ones.bin > ./marble/136032.114.j11x
mv ./marble/136032.115.j10 ./marble/136032.115.j10.raw
cat ./marble/136032.115.j10.raw ./16k_ones.bin > ./marble/136032.115.j10x

cp ./8k_ones.bin ./marble/
cp ./16k_ones.bin ./marble/
cp ./32k_ones.bin ./marble/
cp ./64k_ones.bin ./marble/

rm ./marble/*.raw

# Create output archive
cd ./marble/
zip -r  ../marble_patched.zip .
cd ..

