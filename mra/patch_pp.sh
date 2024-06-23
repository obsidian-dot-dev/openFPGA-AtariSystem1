#!/bin/bash

# Patch script for Peter Pack Rat
#
# Prereqs: zip/unzip 
# Roms: atarisy1 and peterpak sets, compatible with most recent MAME
#
# Usage: Place romsets in same directory as script. Will generate an
# output archive called "peterpak_patched.zip" which can be used with
# the corresponding .mra file.

# Make a working directory to stage the ROM dependencies
# (peterpak, atarisy1 romsets), and unzip to modify them.
mkdir ./peterpak
cp ./peterpak.zip ./peterpak
cp ./atarisy1.zip ./peterpak
cd ./peterpak
unzip ./peterpak.zip
unzip ./atarisy1.zip
rm ./peterpak.zip
rm ./atarisy1.zip
cd ..

# Generate 0xFF padding files used throughout
tr '\0' '\377' < /dev/zero | dd bs=8K count=1 of=./8k_ones.bin
tr '\0' '\377' < /dev/zero | dd bs=16K count=1 of=./16k_ones.bin
tr '\0' '\377' < /dev/zero | dd bs=32K count=1 of=./32k_ones.bin
tr '\0' '\377' < /dev/zero | dd bs=64K count=1 of=./64k_ones.bin

# Files to append 32K 0xFF's
mv ./peterpak/136028.138 ./peterpak/136028.138.raw
cat ./peterpak/136028.138.raw ./32k_ones.bin > ./peterpak/136028.138x
mv ./peterpak/136028.139 ./peterpak/136028.139.raw
cat ./peterpak/136028.139.raw ./32k_ones.bin > ./peterpak/136028.139x
mv ./peterpak/136028.140 ./peterpak/136028.140.raw
cat ./peterpak/136028.140.raw ./32k_ones.bin > ./peterpak/136028.140x
mv ./peterpak/136028.141 ./peterpak/136028.141.raw
cat ./peterpak/136028.141.raw ./32k_ones.bin > ./peterpak/136028.141x

mv ./peterpak/136028.150 ./peterpak/136028.150.raw
cat ./peterpak/136028.150.raw ./32k_ones.bin > ./peterpak/136028.150x
mv ./peterpak/136028.151 ./peterpak/136028.151.raw
cat ./peterpak/136028.151.raw ./32k_ones.bin > ./peterpak/136028.151x
mv ./peterpak/136028.152 ./peterpak/136028.152.raw
cat ./peterpak/136028.152.raw ./32k_ones.bin > ./peterpak/136028.152x
mv ./peterpak/136028.153 ./peterpak/136028.153.raw
cat ./peterpak/136028.153.raw ./32k_ones.bin > ./peterpak/136028.153x

# Prepend 16K, append 32K 0xFFs
mv ./peterpak/136028.105 ./peterpak/136028.105.raw
cat ./16k_ones.bin ./peterpak/136028.105.raw ./32k_ones.bin > ./peterpak/136028.105x
mv ./peterpak/136028.108 ./peterpak/136028.108.raw
cat ./16k_ones.bin ./peterpak/136028.108.raw ./32k_ones.bin > ./peterpak/136028.108x
mv ./peterpak/136028.111 ./peterpak/136028.111.raw
cat ./16k_ones.bin ./peterpak/136028.111.raw ./32k_ones.bin > ./peterpak/136028.111x
mv ./peterpak/136028.114 ./peterpak/136028.114.raw
cat ./16k_ones.bin ./peterpak/136028.114.raw ./32k_ones.bin > ./peterpak/136028.114x

# Append 16K 0xFFs
mv ./peterpak/136032.114.j11 ./peterpak/136032.114.j11.raw
cat ./peterpak/136032.114.j11.raw ./16k_ones.bin > ./peterpak/136032.114.j11x
mv ./peterpak/136032.115.j10 ./peterpak/136032.115.j10.raw
cat ./peterpak/136032.115.j10.raw ./16k_ones.bin > ./peterpak/136032.115.j10x

mv ./peterpak/136028.146 ./peterpak/136028.146.raw
cat ./peterpak/136028.146.raw ./16k_ones.bin > ./peterpak/136028.146x
mv ./peterpak/136028.147 ./peterpak/136028.147.raw
cat ./peterpak/136028.147.raw ./16k_ones.bin > ./peterpak/136028.147x

# Append roms together
mv ./peterpak/136028.142 ./peterpak/136028.142.raw
mv ./peterpak/136028.144 ./peterpak/136028.144.raw
cat ./peterpak/136028.142.raw ./peterpak/136028.144.raw > ./peterpak/136028.142.144x

mv ./peterpak/136028.143 ./peterpak/136028.143.raw
mv ./peterpak/136028.145 ./peterpak/136028.145.raw
cat ./peterpak/136028.143.raw ./peterpak/136028.145.raw > ./peterpak/136028.143.145x


rm ./peterpak/*.raw

cp ./8k_ones.bin ./peterpak/
cp ./16k_ones.bin ./peterpak/
cp ./32k_ones.bin ./peterpak/
cp ./64k_ones.bin ./peterpak/

# Create output archive
cd ./peterpak/
zip -r  ../peterpak_patched.zip .
cd ..

