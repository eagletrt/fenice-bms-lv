cd Core/Lib/bin2srec
cp ../../../build/fenice-bms-lv.bin .
./bin2srec -a 0x8004000 -i fenice-bms-lv.bin -o ../../../fenice-bms-lv.srec
rm fenice-bms-lv.bin