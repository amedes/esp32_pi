Pi calculation program for ESP32
====

This program can calculate Pi more then 10,000 under decimal digits.
Using assembler and dual core feature of ESP32, it takes about 150 seconds.
The calculation method are various arctangent formula like Machin and Euler, etc.

# How to compile and execute

```
idf.py -p serial flash monitor 
```

# Sample output
![sample output](./es32_pi_output.png "output")
