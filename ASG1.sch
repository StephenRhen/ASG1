EESchema Schematic File Version 4
LIBS:ASG1-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Audio Signal Generator"
Date "2019-11-12"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_Module:Arduino_Nano_v3.x A1
U 1 1 5D891AF7
P 4200 2550
F 0 "A1" V 4250 2650 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" V 4150 2650 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 4350 1600 50  0001 L CNN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 4200 1550 50  0001 C CNN
	1    4200 2550
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5D8959AE
P 5750 3050
F 0 "#PWR014" H 5750 2800 50  0001 C CNN
F 1 "GND" H 5755 2877 50  0001 C CNN
F 2 "" H 5750 3050 50  0001 C CNN
F 3 "" H 5750 3050 50  0001 C CNN
	1    5750 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 5D8BD65F
P 6650 2600
F 0 "#PWR018" H 6650 2350 50  0001 C CNN
F 1 "GND" H 6655 2427 50  0001 C CNN
F 2 "" H 6650 2600 50  0001 C CNN
F 3 "" H 6650 2600 50  0001 C CNN
	1    6650 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 2650 5150 2650
Wire Wire Line
	4100 3650 4100 3600
$Comp
L power:+5V #PWR08
U 1 1 5DA869B9
P 4000 1400
F 0 "#PWR08" H 4000 1250 50  0001 C CNN
F 1 "+5V" H 4015 1573 50  0000 C CNN
F 2 "" H 4000 1400 50  0001 C CNN
F 3 "" H 4000 1400 50  0001 C CNN
	1    4000 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 1400 4000 1550
$Comp
L local:1602-I2C U1
U 1 1 5DA920F8
P 2650 3250
F 0 "U1" H 2220 3296 50  0000 R CNN
F 1 "1602-I2C" H 2220 3205 50  0000 R CNN
F 2 "Display:1602-I2C" H 2650 2650 50  0001 C CNN
F 3 "http://www.lcd-module.de/pdf/doma/t123-i2c.pdf" H 2650 2750 50  0001 C CNN
	1    2650 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5DA97352
P 2650 3750
F 0 "#PWR04" H 2650 3500 50  0001 C CNN
F 1 "GND" H 2655 3577 50  0001 C CNN
F 2 "" H 2650 3750 50  0001 C CNN
F 3 "" H 2650 3750 50  0001 C CNN
	1    2650 3750
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR03
U 1 1 5DA97842
P 2650 2750
F 0 "#PWR03" H 2650 2600 50  0001 C CNN
F 1 "+5V" H 2665 2923 50  0000 C CNN
F 2 "" H 2650 2750 50  0001 C CNN
F 3 "" H 2650 2750 50  0001 C CNN
	1    2650 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 4250 7300 4200
$Comp
L power:GND #PWR09
U 1 1 5D8963D6
P 4100 3650
F 0 "#PWR09" H 4100 3400 50  0001 C CNN
F 1 "GND" H 4105 3477 50  0001 C CNN
F 2 "" H 4100 3650 50  0001 C CNN
F 3 "" H 4100 3650 50  0001 C CNN
	1    4100 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:Rotary_Encoder_Switch SW1
U 1 1 5DCEA549
P 2350 5650
F 0 "SW1" V 2950 5700 50  0000 R CNN
F 1 "Rotary_Encoder_Switch" V 2850 6000 50  0000 R CNN
F 2 "" H 2200 5810 50  0001 C CNN
F 3 "~" H 2350 5910 50  0001 C CNN
	1    2350 5650
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 5DCEA54F
P 2000 6350
F 0 "R3" V 1800 6350 50  0000 C CNN
F 1 "10K" V 1900 6350 50  0000 C CNN
F 2 "" V 1930 6350 50  0001 C CNN
F 3 "~" H 2000 6350 50  0001 C CNN
	1    2000 6350
	0    -1   1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 5DCEA555
P 2000 6000
F 0 "R2" V 1793 6000 50  0000 C CNN
F 1 "10K" V 1884 6000 50  0000 C CNN
F 2 "" V 1930 6000 50  0001 C CNN
F 3 "~" H 2000 6000 50  0001 C CNN
	1    2000 6000
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5DCEA55B
P 2350 6100
F 0 "#PWR02" H 2350 5850 50  0001 C CNN
F 1 "GND" H 2355 5927 50  0001 C CNN
F 2 "" H 2350 6100 50  0001 C CNN
F 3 "" H 2350 6100 50  0001 C CNN
	1    2350 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5DCEA562
P 2750 6000
F 0 "R5" V 2543 6000 50  0000 C CNN
F 1 "10K" V 2634 6000 50  0000 C CNN
F 2 "" V 2680 6000 50  0001 C CNN
F 3 "~" H 2750 6000 50  0001 C CNN
	1    2750 6000
	0    -1   1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 5DCEA568
P 2750 6350
F 0 "R6" V 2550 6350 50  0000 C CNN
F 1 "10K" V 2650 6350 50  0000 C CNN
F 2 "" V 2680 6350 50  0001 C CNN
F 3 "~" H 2750 6350 50  0001 C CNN
	1    2750 6350
	0    -1   1    0   
$EndComp
$Comp
L Device:C C2
U 1 1 5DCEA57F
P 3350 6550
F 0 "C2" H 3236 6596 50  0000 R CNN
F 1 "0.01uF" H 3236 6505 50  0000 R CNN
F 2 "" H 3388 6400 50  0001 C CNN
F 3 "~" H 3350 6550 50  0001 C CNN
	1    3350 6550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5DCEA585
P 3650 6550
F 0 "C3" H 3765 6596 50  0000 L CNN
F 1 "0.01uF" H 3765 6505 50  0000 L CNN
F 2 "" H 3688 6400 50  0001 C CNN
F 3 "~" H 3650 6550 50  0001 C CNN
	1    3650 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 6000 3350 6000
Wire Wire Line
	2250 5950 2250 6000
Wire Wire Line
	2250 6000 2600 6000
$Comp
L Device:R R4
U 1 1 5DCEA5A3
P 2750 5300
F 0 "R4" V 2635 5300 50  0000 C CNN
F 1 "10K" V 2544 5300 50  0000 C CNN
F 2 "" V 2680 5300 50  0001 C CNN
F 3 "~" H 2750 5300 50  0001 C CNN
	1    2750 5300
	0    1    -1   0   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5DCEA5AF
P 3050 5650
F 0 "#PWR05" H 3050 5400 50  0001 C CNN
F 1 "GND" H 3055 5477 50  0001 C CNN
F 2 "" H 3050 5650 50  0001 C CNN
F 3 "" H 3050 5650 50  0001 C CNN
	1    3050 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 5350 2450 5300
Wire Wire Line
	4700 2550 5150 2550
Wire Wire Line
	4700 2450 5150 2450
Wire Wire Line
	4700 2350 5150 2350
Wire Wire Line
	4700 2250 5150 2250
Wire Wire Line
	4700 2150 5150 2150
Wire Wire Line
	4700 2050 5150 2050
Wire Wire Line
	4700 1950 5150 1950
$Comp
L Analog_DAC:AD7533JN U2
U 1 1 5D8908ED
P 5750 2250
F 0 "U2" H 5750 2450 50  0000 C CNN
F 1 "AD7523" H 5750 2300 50  0000 C CNN
F 2 "" H 5750 2250 50  0001 C CIN
F 3 "https://www.analog.com/static/imported-files/data_sheets/AD7533.pdf" H 5750 2250 50  0001 C CNN
	1    5750 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 2750 5200 2750
Wire Wire Line
	5200 2750 5200 3500
Wire Wire Line
	4700 2850 5100 2850
Wire Wire Line
	5100 2850 5100 3600
Wire Wire Line
	4700 2950 5000 2950
Wire Wire Line
	4700 3050 4900 3050
Wire Wire Line
	2150 6000 2250 6000
Connection ~ 2250 6000
Wire Wire Line
	1750 6000 1850 6000
$Comp
L power:+5V #PWR01
U 1 1 5DDED91A
P 1750 5050
F 0 "#PWR01" H 1750 4900 50  0001 C CNN
F 1 "+5V" H 1765 5223 50  0000 C CNN
F 2 "" H 1750 5050 50  0001 C CNN
F 3 "" H 1750 5050 50  0001 C CNN
	1    1750 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 5950 2350 6100
Wire Wire Line
	2600 6350 2450 6350
Wire Wire Line
	2450 5950 2450 6350
Connection ~ 2450 6350
Wire Wire Line
	2450 6350 2150 6350
Wire Wire Line
	1750 6000 1750 6350
Wire Wire Line
	1750 6350 1850 6350
Connection ~ 1750 6000
$Comp
L power:GND #PWR06
U 1 1 5DE78692
P 3350 6750
F 0 "#PWR06" H 3350 6500 50  0001 C CNN
F 1 "GND" H 3355 6577 50  0001 C CNN
F 2 "" H 3350 6750 50  0001 C CNN
F 3 "" H 3350 6750 50  0001 C CNN
	1    3350 6750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 5DE78B83
P 3650 6750
F 0 "#PWR07" H 3650 6500 50  0001 C CNN
F 1 "GND" H 3655 6577 50  0001 C CNN
F 2 "" H 3650 6750 50  0001 C CNN
F 3 "" H 3650 6750 50  0001 C CNN
	1    3650 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 6700 3650 6750
Wire Wire Line
	3350 6700 3350 6750
Wire Wire Line
	3350 4450 3350 6000
$Comp
L Device:C C1
U 1 1 5DCEA5A9
P 3050 5500
F 0 "C1" H 3165 5546 50  0000 L CNN
F 1 "0.01uF" H 3165 5455 50  0000 L CNN
F 2 "" H 3088 5350 50  0001 C CNN
F 3 "~" H 3050 5500 50  0001 C CNN
	1    3050 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 5300 2600 5300
Wire Wire Line
	2900 5300 3050 5300
Wire Wire Line
	3050 5300 3050 5350
Wire Wire Line
	1750 5050 1750 5300
Wire Wire Line
	1850 5300 1750 5300
Connection ~ 1750 5300
Wire Wire Line
	1750 5300 1750 6000
Wire Wire Line
	2150 5300 2250 5300
Wire Wire Line
	2250 5300 2250 5350
$Comp
L Device:R R1
U 1 1 5DCEA56E
P 2000 5300
F 0 "R1" V 2100 5350 50  0000 C CNN
F 1 "10K" V 2200 5350 50  0000 C CNN
F 2 "" V 1930 5300 50  0001 C CNN
F 3 "~" H 2000 5300 50  0001 C CNN
	1    2000 5300
	0    -1   1    0   
$EndComp
Connection ~ 3350 6000
Wire Wire Line
	3350 6000 3350 6400
Wire Wire Line
	2900 6350 3650 6350
Wire Wire Line
	3650 4550 3650 6350
Connection ~ 3650 6350
Wire Wire Line
	3650 6350 3650 6400
Text Notes 2100 4900 0    50   ~ 0
Pulsewidth/Waveform
Wire Wire Line
	3050 5300 3050 4350
Wire Wire Line
	3050 4350 4800 4350
Wire Wire Line
	4800 4350 4800 3150
Wire Wire Line
	4800 3150 4700 3150
Connection ~ 3050 5300
$Comp
L power:GND #PWR015
U 1 1 5DF41D7E
P 5800 6750
F 0 "#PWR015" H 5800 6500 50  0001 C CNN
F 1 "GND" H 5805 6577 50  0001 C CNN
F 2 "" H 5800 6750 50  0001 C CNN
F 3 "" H 5800 6750 50  0001 C CNN
	1    5800 6750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5DF41D84
P 6100 6750
F 0 "#PWR017" H 6100 6500 50  0001 C CNN
F 1 "GND" H 6105 6577 50  0001 C CNN
F 2 "" H 6100 6750 50  0001 C CNN
F 3 "" H 6100 6750 50  0001 C CNN
	1    6100 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 6700 6100 6750
Wire Wire Line
	5800 6700 5800 6750
Wire Wire Line
	5200 3500 6100 3500
Wire Wire Line
	5100 3600 5800 3600
Wire Wire Line
	6350 2650 6400 2650
Wire Wire Line
	3150 2950 3700 2950
Wire Wire Line
	3150 3050 3700 3050
Wire Wire Line
	4200 3550 4200 3600
Wire Wire Line
	4200 3600 4100 3600
Connection ~ 4100 3600
Wire Wire Line
	4100 3600 4100 3550
Wire Wire Line
	3700 2850 3600 2850
Wire Wire Line
	3600 2850 3600 3850
Wire Wire Line
	3600 3850 5500 3850
Wire Wire Line
	6400 4150 3300 4150
Wire Wire Line
	3300 4150 3300 2550
Wire Wire Line
	3300 2550 3700 2550
$Comp
L Device:R R16
U 1 1 5DC3B19E
P 7800 2500
F 0 "R16" V 8007 2500 50  0000 C CNN
F 1 "7.5K" V 7916 2500 50  0000 C CNN
F 2 "" V 7730 2500 50  0001 C CNN
F 3 "~" H 7800 2500 50  0001 C CNN
	1    7800 2500
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R18
U 1 1 5DC3B7EF
P 7900 3700
F 0 "R18" H 7830 3654 50  0000 R CNN
F 1 "4.3K" H 7830 3745 50  0000 R CNN
F 2 "" V 7830 3700 50  0001 C CNN
F 3 "~" H 7900 3700 50  0001 C CNN
	1    7900 3700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR026
U 1 1 5DC3CB53
P 7900 4050
F 0 "#PWR026" H 7900 3800 50  0001 C CNN
F 1 "GND" H 7905 3877 50  0001 C CNN
F 2 "" H 7900 4050 50  0001 C CNN
F 3 "" H 7900 4050 50  0001 C CNN
	1    7900 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C12
U 1 1 5DC3CF6E
P 8550 2150
F 0 "C12" V 8298 2150 50  0000 C CNN
F 1 "100pF" V 8389 2150 50  0000 C CNN
F 2 "" H 8588 2000 50  0001 C CNN
F 3 "~" H 8550 2150 50  0001 C CNN
	1    8550 2150
	0    1    1    0   
$EndComp
$Comp
L Device:R R19
U 1 1 5DD0A2F6
P 8550 2500
F 0 "R19" V 8343 2500 50  0000 C CNN
F 1 "30K" V 8434 2500 50  0000 C CNN
F 2 "" V 8480 2500 50  0001 C CNN
F 3 "~" H 8550 2500 50  0001 C CNN
	1    8550 2500
	0    1    1    0   
$EndComp
$Comp
L Device:R R17
U 1 1 5DD28696
P 7800 3300
F 0 "R17" V 7593 3300 50  0000 C CNN
F 1 "15K" V 7684 3300 50  0000 C CNN
F 2 "" V 7730 3300 50  0001 C CNN
F 3 "~" H 7800 3300 50  0001 C CNN
	1    7800 3300
	0    1    1    0   
$EndComp
$Comp
L local:LM741alt U3
U 1 1 5DCC2706
P 7150 2500
F 0 "U3" H 7350 2350 50  0000 L CNN
F 1 "LM741" H 7350 2250 50  0000 L CNN
F 2 "" H 7200 2550 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm741.pdf" H 7300 2650 50  0001 C CNN
	1    7150 2500
	1    0    0    -1  
$EndComp
$Comp
L local:LM741alt U5
U 1 1 5DD04E95
P 8400 3400
F 0 "U5" H 8650 3250 50  0000 L CNN
F 1 "LM741" H 8650 3150 50  0000 L CNN
F 2 "" H 8450 3450 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm741.pdf" H 8550 3550 50  0001 C CNN
	1    8400 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 3300 8050 3300
Connection ~ 8050 3300
Wire Wire Line
	8050 3300 8100 3300
Wire Wire Line
	8050 2150 8400 2150
Wire Wire Line
	8100 3500 7900 3500
Wire Wire Line
	7900 3500 7900 3550
Wire Wire Line
	7900 4050 7900 3850
$Comp
L power:+12V #PWR027
U 1 1 5DD4E586
P 8300 2800
F 0 "#PWR027" H 8300 2650 50  0001 C CNN
F 1 "+12V" H 8315 2973 50  0000 C CNN
F 2 "" H 8300 2800 50  0001 C CNN
F 3 "" H 8300 2800 50  0001 C CNN
	1    8300 2800
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR021
U 1 1 5DD4D793
P 7050 1950
F 0 "#PWR021" H 7050 1800 50  0001 C CNN
F 1 "+12V" H 7100 2100 50  0000 C CNN
F 2 "" H 7050 1950 50  0001 C CNN
F 3 "" H 7050 1950 50  0001 C CNN
	1    7050 1950
	1    0    0    -1  
$EndComp
$Comp
L power:-12V #PWR022
U 1 1 5DD6970C
P 7050 3050
F 0 "#PWR022" H 7050 3150 50  0001 C CNN
F 1 "-12V" H 7065 3223 50  0000 C CNN
F 2 "" H 7050 3050 50  0001 C CNN
F 3 "" H 7050 3050 50  0001 C CNN
	1    7050 3050
	1    0    0    1   
$EndComp
$Comp
L power:-12V #PWR028
U 1 1 5DD6FAAF
P 8300 4150
F 0 "#PWR028" H 8300 4250 50  0001 C CNN
F 1 "-12V" H 8315 4323 50  0000 C CNN
F 2 "" H 8300 4150 50  0001 C CNN
F 3 "" H 8300 4150 50  0001 C CNN
	1    8300 4150
	1    0    0    1   
$EndComp
$Comp
L power:+12V #PWR013
U 1 1 5DD904CA
P 5750 1100
F 0 "#PWR013" H 5750 950 50  0001 C CNN
F 1 "+12V" H 5800 1250 50  0000 C CNN
F 2 "" H 5750 1100 50  0001 C CNN
F 3 "" H 5750 1100 50  0001 C CNN
	1    5750 1100
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR024
U 1 1 5DD908F8
P 7300 4200
F 0 "#PWR024" H 7300 4050 50  0001 C CNN
F 1 "+12V" H 7350 4350 50  0000 C CNN
F 2 "" H 7300 4200 50  0001 C CNN
F 3 "" H 7300 4200 50  0001 C CNN
	1    7300 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 3400 9050 3400
$Comp
L Device:C C11
U 1 1 5DCB21E0
P 8500 4000
F 0 "C11" H 8385 3954 50  0000 R CNN
F 1 "0.1uF" H 8385 4045 50  0000 R CNN
F 2 "" H 8538 3850 50  0001 C CNN
F 3 "~" H 8500 4000 50  0001 C CNN
	1    8500 4000
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR029
U 1 1 5DCB7A34
P 8500 4150
F 0 "#PWR029" H 8500 3900 50  0001 C CNN
F 1 "GND" H 8505 3977 50  0001 C CNN
F 2 "" H 8500 4150 50  0001 C CNN
F 3 "" H 8500 4150 50  0001 C CNN
	1    8500 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 3700 8300 3850
Wire Wire Line
	8500 3850 8300 3850
Connection ~ 8300 3850
Wire Wire Line
	8300 3850 8300 4150
$Comp
L Device:C C13
U 1 1 5DCC898D
P 8550 2950
F 0 "C13" H 8435 2904 50  0000 R CNN
F 1 "0.1uF" H 8435 2995 50  0000 R CNN
F 2 "" H 8588 2800 50  0001 C CNN
F 3 "~" H 8550 2950 50  0001 C CNN
	1    8550 2950
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR030
U 1 1 5DCC9AAD
P 8550 3100
F 0 "#PWR030" H 8550 2850 50  0001 C CNN
F 1 "GND" H 8555 2927 50  0001 C CNN
F 2 "" H 8550 3100 50  0001 C CNN
F 3 "" H 8550 3100 50  0001 C CNN
	1    8550 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 3100 8300 2800
Wire Wire Line
	8550 2800 8300 2800
Connection ~ 8300 2800
Wire Wire Line
	9050 3400 9050 2500
Wire Wire Line
	8700 2150 9050 2150
Connection ~ 9050 3400
Wire Wire Line
	9050 3400 9150 3400
Wire Wire Line
	8700 2500 9050 2500
Connection ~ 9050 2500
Wire Wire Line
	9050 2500 9050 2150
Wire Wire Line
	8400 2500 8050 2500
Connection ~ 8050 2500
Wire Wire Line
	8050 2500 8050 2150
Wire Wire Line
	7650 3300 6400 3300
Wire Wire Line
	6400 2650 6400 3300
Connection ~ 6400 3300
Wire Wire Line
	6400 3300 6400 4150
Wire Wire Line
	8050 2500 8050 3300
Wire Wire Line
	7950 2500 8050 2500
Wire Wire Line
	7450 2500 7600 2500
$Comp
L Device:C C9
U 1 1 5DD10847
P 7300 2100
F 0 "C9" H 7185 2054 50  0000 R CNN
F 1 "0.1uF" H 7185 2145 50  0000 R CNN
F 2 "" H 7338 1950 50  0001 C CNN
F 3 "~" H 7300 2100 50  0001 C CNN
	1    7300 2100
	-1   0    0    1   
$EndComp
Wire Wire Line
	6350 2050 6800 2050
$Comp
L power:GND #PWR023
U 1 1 5DD16BDC
P 7300 2250
F 0 "#PWR023" H 7300 2000 50  0001 C CNN
F 1 "GND" H 7305 2077 50  0001 C CNN
F 2 "" H 7300 2250 50  0001 C CNN
F 3 "" H 7300 2250 50  0001 C CNN
	1    7300 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 2200 7050 1950
Wire Wire Line
	7300 1950 7050 1950
Connection ~ 7050 1950
Wire Wire Line
	7600 2500 7600 1750
Wire Wire Line
	6350 1750 7600 1750
Connection ~ 7600 2500
Wire Wire Line
	7600 2500 7650 2500
Wire Wire Line
	6650 2600 6850 2600
Wire Wire Line
	6650 2600 6650 2350
Wire Wire Line
	6350 2350 6650 2350
Wire Wire Line
	6800 2400 6850 2400
Wire Wire Line
	6800 2050 6800 2400
$Comp
L Device:C C8
U 1 1 5DD3A49E
P 6800 3000
F 0 "C8" H 6914 2954 50  0000 L CNN
F 1 "0.1uF" H 6914 3045 50  0000 L CNN
F 2 "" H 6838 2850 50  0001 C CNN
F 3 "~" H 6800 3000 50  0001 C CNN
	1    6800 3000
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5DD3AE57
P 6800 3150
F 0 "#PWR020" H 6800 2900 50  0001 C CNN
F 1 "GND" H 6805 2977 50  0001 C CNN
F 2 "" H 6800 3150 50  0001 C CNN
F 3 "" H 6800 3150 50  0001 C CNN
	1    6800 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 2800 7050 2850
Connection ~ 7050 2850
Wire Wire Line
	7050 2850 7050 3050
Connection ~ 6650 2600
Wire Wire Line
	6800 2850 7050 2850
$Comp
L Device:C C6
U 1 1 5DD4EA2C
P 6050 1250
F 0 "C6" H 5935 1204 50  0000 R CNN
F 1 "0.1uF" H 5935 1295 50  0000 R CNN
F 2 "" H 6088 1100 50  0001 C CNN
F 3 "~" H 6050 1250 50  0001 C CNN
	1    6050 1250
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5DD4F235
P 6050 1400
F 0 "#PWR016" H 6050 1150 50  0001 C CNN
F 1 "GND" H 6055 1227 50  0001 C CNN
F 2 "" H 6050 1400 50  0001 C CNN
F 3 "" H 6050 1400 50  0001 C CNN
	1    6050 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 1450 5750 1100
Wire Wire Line
	6050 1100 5750 1100
Connection ~ 5750 1100
Connection ~ 6400 4150
Wire Wire Line
	6400 4150 6400 4800
Wire Wire Line
	5500 3850 5500 5300
Wire Wire Line
	6400 4800 6500 4800
Wire Wire Line
	5800 3600 5800 6000
Wire Wire Line
	6100 3500 6100 6350
Connection ~ 5500 5300
Text Notes 4550 4900 0    50   ~ 0
Frequency/Range
Wire Wire Line
	6100 6350 6100 6400
Connection ~ 6100 6350
Wire Wire Line
	5350 6350 6100 6350
Wire Wire Line
	5800 6000 5800 6400
Connection ~ 5800 6000
$Comp
L Device:R R7
U 1 1 5DF41D9C
P 4450 5300
F 0 "R7" V 4550 5350 50  0000 C CNN
F 1 "10K" V 4650 5350 50  0000 C CNN
F 2 "" V 4380 5300 50  0001 C CNN
F 3 "~" H 4450 5300 50  0001 C CNN
	1    4450 5300
	0    -1   1    0   
$EndComp
Wire Wire Line
	4700 5300 4700 5350
Wire Wire Line
	4600 5300 4700 5300
Wire Wire Line
	4200 5300 4200 6000
Connection ~ 4200 5300
Wire Wire Line
	4300 5300 4200 5300
Wire Wire Line
	4200 5050 4200 5300
Wire Wire Line
	5500 5300 5500 5350
Wire Wire Line
	5350 5300 5500 5300
Wire Wire Line
	4900 5300 5050 5300
$Comp
L Device:C C4
U 1 1 5DF41D8D
P 5500 5500
F 0 "C4" H 5615 5546 50  0000 L CNN
F 1 "0.01uF" H 5615 5455 50  0000 L CNN
F 2 "" H 5538 5350 50  0001 C CNN
F 3 "~" H 5500 5500 50  0001 C CNN
	1    5500 5500
	1    0    0    -1  
$EndComp
Connection ~ 4200 6000
Wire Wire Line
	4200 6350 4300 6350
Wire Wire Line
	4200 6000 4200 6350
Wire Wire Line
	4900 6350 4600 6350
Connection ~ 4900 6350
Wire Wire Line
	4900 5950 4900 6350
Wire Wire Line
	5050 6350 4900 6350
Wire Wire Line
	4800 5950 4800 6100
$Comp
L power:+5V #PWR010
U 1 1 5DF41D70
P 4200 5050
F 0 "#PWR010" H 4200 4900 50  0001 C CNN
F 1 "+5V" H 4215 5223 50  0000 C CNN
F 2 "" H 4200 5050 50  0001 C CNN
F 3 "" H 4200 5050 50  0001 C CNN
	1    4200 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 6000 4300 6000
Connection ~ 4700 6000
Wire Wire Line
	4600 6000 4700 6000
Wire Wire Line
	4900 5350 4900 5300
$Comp
L power:GND #PWR012
U 1 1 5DF41D66
P 5500 5650
F 0 "#PWR012" H 5500 5400 50  0001 C CNN
F 1 "GND" H 5505 5477 50  0001 C CNN
F 2 "" H 5500 5650 50  0001 C CNN
F 3 "" H 5500 5650 50  0001 C CNN
	1    5500 5650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 5DF41D60
P 5200 5300
F 0 "R10" V 5085 5300 50  0000 C CNN
F 1 "10K" V 4994 5300 50  0000 C CNN
F 2 "" V 5130 5300 50  0001 C CNN
F 3 "~" H 5200 5300 50  0001 C CNN
	1    5200 5300
	0    1    -1   0   
$EndComp
Wire Wire Line
	4700 6000 5050 6000
Wire Wire Line
	4700 5950 4700 6000
Wire Wire Line
	5350 6000 5800 6000
$Comp
L Device:C C7
U 1 1 5DF41D57
P 6100 6550
F 0 "C7" H 6215 6596 50  0000 L CNN
F 1 "0.01uF" H 6215 6505 50  0000 L CNN
F 2 "" H 6138 6400 50  0001 C CNN
F 3 "~" H 6100 6550 50  0001 C CNN
	1    6100 6550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5DF41D51
P 5800 6550
F 0 "C5" H 5686 6596 50  0000 R CNN
F 1 "0.01uF" H 5686 6505 50  0000 R CNN
F 2 "" H 5838 6400 50  0001 C CNN
F 3 "~" H 5800 6550 50  0001 C CNN
	1    5800 6550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 5DF41D4B
P 5200 6350
F 0 "R12" V 5000 6350 50  0000 C CNN
F 1 "10K" V 5100 6350 50  0000 C CNN
F 2 "" V 5130 6350 50  0001 C CNN
F 3 "~" H 5200 6350 50  0001 C CNN
	1    5200 6350
	0    -1   1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 5DF41D45
P 5200 6000
F 0 "R11" V 4993 6000 50  0000 C CNN
F 1 "10K" V 5084 6000 50  0000 C CNN
F 2 "" V 5130 6000 50  0001 C CNN
F 3 "~" H 5200 6000 50  0001 C CNN
	1    5200 6000
	0    -1   1    0   
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5DF41D3F
P 4800 6100
F 0 "#PWR011" H 4800 5850 50  0001 C CNN
F 1 "GND" H 4805 5927 50  0001 C CNN
F 2 "" H 4800 6100 50  0001 C CNN
F 3 "" H 4800 6100 50  0001 C CNN
	1    4800 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 5DF41D39
P 4450 6000
F 0 "R8" V 4243 6000 50  0000 C CNN
F 1 "10K" V 4334 6000 50  0000 C CNN
F 2 "" V 4380 6000 50  0001 C CNN
F 3 "~" H 4450 6000 50  0001 C CNN
	1    4450 6000
	0    -1   1    0   
$EndComp
$Comp
L Device:R R9
U 1 1 5DF41D33
P 4450 6350
F 0 "R9" V 4250 6350 50  0000 C CNN
F 1 "10K" V 4350 6350 50  0000 C CNN
F 2 "" V 4380 6350 50  0001 C CNN
F 3 "~" H 4450 6350 50  0001 C CNN
	1    4450 6350
	0    -1   1    0   
$EndComp
$Comp
L Device:Rotary_Encoder_Switch SW2
U 1 1 5DF41D2D
P 4800 5650
F 0 "SW2" V 5400 5700 50  0000 R CNN
F 1 "Rotary_Encoder_Switch" V 5300 6000 50  0000 R CNN
F 2 "" H 4650 5810 50  0001 C CNN
F 3 "~" H 4800 5910 50  0001 C CNN
	1    4800 5650
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4900 4450 3350 4450
Wire Wire Line
	4900 3050 4900 4450
Wire Wire Line
	5000 4550 3650 4550
Wire Wire Line
	5000 2950 5000 4550
Text GLabel 7650 4600 2    50   Input ~ 0
5VREF
Wire Wire Line
	7650 5350 7300 5350
Wire Wire Line
	7650 5100 7650 5350
Wire Wire Line
	7650 4600 7300 4600
Wire Wire Line
	7650 4800 7650 4600
$Comp
L Device:CP C10
U 1 1 5DA709DA
P 7650 4950
F 0 "C10" H 7532 4996 50  0000 R CNN
F 1 "10u" H 7532 4905 50  0000 R CNN
F 2 "" H 7688 4800 50  0001 C CNN
F 3 "~" H 7650 4950 50  0001 C CNN
	1    7650 4950
	-1   0    0    -1  
$EndComp
Connection ~ 7300 5350
Wire Wire Line
	7300 5400 7300 5350
$Comp
L power:GND #PWR025
U 1 1 5DA6A5D2
P 7300 5400
F 0 "#PWR025" H 7300 5150 50  0001 C CNN
F 1 "GND" H 7305 5227 50  0001 C CNN
F 2 "" H 7300 5400 50  0001 C CNN
F 3 "" H 7300 5400 50  0001 C CNN
	1    7300 5400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7150 5350 7150 5100
Wire Wire Line
	7300 5350 7150 5350
Wire Wire Line
	7150 4600 6650 4600
Connection ~ 7150 4600
Wire Wire Line
	7150 4900 7150 4600
Wire Wire Line
	7300 5000 7300 4950
Connection ~ 7300 5000
Wire Wire Line
	7250 5000 7300 5000
Wire Wire Line
	7300 5050 7300 5000
Connection ~ 7300 4600
Wire Wire Line
	7300 4600 7300 4550
Wire Wire Line
	7300 4650 7300 4600
$Comp
L Device:R R13
U 1 1 5DA535C2
P 7300 4400
F 0 "R13" H 7370 4446 50  0000 L CNN
F 1 "390" H 7370 4355 50  0000 L CNN
F 2 "" V 7230 4400 50  0001 C CNN
F 3 "~" H 7300 4400 50  0001 C CNN
	1    7300 4400
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R15
U 1 1 5DA5322B
P 7300 5200
F 0 "R15" H 7231 5246 50  0000 R CNN
F 1 "4.7K" H 7231 5155 50  0000 R CNN
F 2 "" V 7230 5200 50  0001 C CNN
F 3 "~" H 7300 5200 50  0001 C CNN
	1    7300 5200
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 5DA52AFA
P 7300 4800
F 0 "R14" H 7369 4846 50  0000 L CNN
F 1 "4.7K" H 7369 4755 50  0000 L CNN
F 2 "" V 7230 4800 50  0001 C CNN
F 3 "~" H 7300 4800 50  0001 C CNN
	1    7300 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 4600 7150 4600
$Comp
L Reference_Voltage:TL431LP U4
U 1 1 5DA48428
P 7150 5000
F 0 "U4" V 7196 4930 50  0000 R CNN
F 1 "TL431LP" V 7105 4930 50  0000 R CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 7150 4850 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/tl431.pdf" H 7150 5000 50  0001 C CIN
	1    7150 5000
	0    1    -1   0   
$EndComp
Text Notes 6850 4500 2    50   ~ 0
Amplitude
Wire Wire Line
	6650 5000 6650 4950
Wire Wire Line
	6650 4650 6650 4600
$Comp
L power:GND #PWR019
U 1 1 5DA5821C
P 6650 5000
F 0 "#PWR019" H 6650 4750 50  0001 C CNN
F 1 "GND" H 6655 4827 50  0001 C CNN
F 2 "" H 6650 5000 50  0001 C CNN
F 3 "" H 6650 5000 50  0001 C CNN
	1    6650 5000
	-1   0    0    -1  
$EndComp
$Comp
L Device:R_POT RV1
U 1 1 5DA42FEB
P 6650 4800
F 0 "RV1" H 6581 4846 50  0000 R CNN
F 1 "2K" H 6581 4755 50  0000 R CNN
F 2 "" H 6650 4800 50  0001 C CNN
F 3 "~" H 6650 4800 50  0001 C CNN
	1    6650 4800
	-1   0    0    -1  
$EndComp
Text GLabel 9150 3400 2    50   Input ~ 0
Output
$EndSCHEMATC
