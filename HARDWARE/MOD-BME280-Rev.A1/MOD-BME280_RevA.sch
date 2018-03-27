EESchema Schematic File Version 4
LIBS:MOD-BME280_RevA-cache
EELAYER 26 0
EELAYER END
$Descr User 8268 5827
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	2450 1700 2450 1800
$Comp
L MOD-BME280_RevA:GND #PWR01
U 1 1 5874A8EB
P 2450 1800
F 0 "#PWR01" H 2450 1550 50  0001 C CNN
F 1 "GND" H 2455 1627 50  0000 C CNN
F 2 "" H 2450 1800 50  0000 C CNN
F 3 "" H 2450 1800 50  0000 C CNN
	1    2450 1800
	1    0    0    -1  
$EndComp
$Comp
L MOD-BME280_RevA:C_Small C1
U 1 1 5874A903
P 2100 2800
F 0 "C1" H 2192 2846 50  0000 L CNN
F 1 "100nF" H 2192 2755 50  0000 L CNN
F 2 "OLIMEX_RLC-FP:C_0603_5MIL_DWS" H 2100 2800 50  0001 C CNN
F 3 "" H 2100 2800 50  0000 C CNN
	1    2100 2800
	1    0    0    -1  
$EndComp
$Comp
L MOD-BME280_RevA:R R1
U 1 1 5874A929
P 3950 1800
F 0 "R1" V 4100 1800 50  0000 L CNN
F 1 "4.7K" V 4200 1800 50  0000 L CNN
F 2 "OLIMEX_RLC-FP:R_0603_5MIL_DWS" V 3880 1800 50  0001 C CNN
F 3 "" H 3950 1800 50  0000 C CNN
	1    3950 1800
	0    1    1    0   
$EndComp
$Comp
L MOD-BME280_RevA:R R2
U 1 1 5874A969
P 4200 1800
F 0 "R2" V 4350 1800 50  0000 L CNN
F 1 "4.7K" V 4450 1800 50  0000 L CNN
F 2 "OLIMEX_RLC-FP:R_0603_5MIL_DWS" V 4130 1800 50  0001 C CNN
F 3 "" H 4200 1800 50  0000 C CNN
	1    4200 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	4900 1600 4900 2450
Wire Wire Line
	3800 2650 2600 2650
Wire Wire Line
	3800 2450 4600 2450
Wire Wire Line
	2100 2900 2100 2950
$Comp
L MOD-BME280_RevA:GND #PWR02
U 1 1 5874AA33
P 2100 2950
F 0 "#PWR02" H 2100 2700 50  0001 C CNN
F 1 "GND" H 2105 2777 50  0000 C CNN
F 2 "" H 2100 2950 50  0000 C CNN
F 3 "" H 2100 2950 50  0000 C CNN
	1    2100 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 2450 5750 2450
$Comp
L MOD-BME280_RevA:GND #PWR03
U 1 1 5874AA61
P 5750 2450
F 0 "#PWR03" H 5750 2200 50  0001 C CNN
F 1 "GND" V 5755 2322 50  0000 R CNN
F 2 "" H 5750 2450 50  0000 C CNN
F 3 "" H 5750 2450 50  0000 C CNN
	1    5750 2450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3700 2200 4200 2200
Wire Wire Line
	5700 2200 5700 2650
Wire Wire Line
	5700 2650 5600 2650
Wire Wire Line
	3700 2300 3950 2300
Wire Wire Line
	4800 2650 5000 2650
Wire Wire Line
	3950 1950 3950 2300
Connection ~ 3950 2300
Wire Wire Line
	4200 1950 4200 2200
Connection ~ 4200 2200
Wire Wire Line
	3950 1650 3950 1600
Wire Wire Line
	3950 1600 4200 1600
Connection ~ 4900 2450
Wire Wire Line
	4200 1650 4200 1600
Connection ~ 4200 1600
Connection ~ 4800 2650
Wire Wire Line
	4600 2450 4600 3000
Connection ~ 4600 2450
Wire Wire Line
	4500 3000 4500 2950
$Comp
L MOD-BME280_RevA:GND #PWR04
U 1 1 5874AC92
P 4500 2950
F 0 "#PWR04" H 4500 2700 50  0001 C CNN
F 1 "GND" H 4505 2777 50  0000 C CNN
F 2 "" H 4500 2950 50  0000 C CNN
F 3 "" H 4500 2950 50  0000 C CNN
	1    4500 2950
	-1   0    0    1   
$EndComp
$Comp
L MOD-BME280_RevA:BME280 U1
U 1 1 59648E20
P 3150 2150
F 0 "U1" H 3200 2547 60  0000 C CNN
F 1 "BME280" H 3200 2441 60  0000 C CNN
F 2 "OLIMEX_IC-FP:BME280" H 3150 1950 60  0001 C CNN
F 3 "" H 3150 1950 60  0001 C CNN
	1    3150 2150
	1    0    0    -1  
$EndComp
Text Label 4800 2950 1    60   ~ 0
SCL
Text Label 4700 2950 1    60   ~ 0
SDA
Wire Wire Line
	4800 2300 4800 2650
Wire Wire Line
	4700 3000 4700 2200
Connection ~ 4700 2200
Wire Wire Line
	3700 2100 3800 2100
Wire Wire Line
	3800 2100 3800 2450
Wire Wire Line
	2700 2000 2100 2000
Wire Wire Line
	2100 2000 2100 2650
Connection ~ 3800 2450
Wire Wire Line
	2250 2100 2600 2100
Wire Wire Line
	2600 2100 2600 1700
Wire Wire Line
	2450 1700 2600 1700
Wire Wire Line
	3800 1700 3800 2000
Wire Wire Line
	3800 2000 3700 2000
Connection ~ 2100 2650
Connection ~ 2600 1700
$Comp
L MOD-BME280_RevA:SJ2W_Closed(1-2) SJ1
U 1 1 59649171
P 2250 2300
F 0 "SJ1" H 2450 2350 50  0000 L CNN
F 1 "SJ2W_Closed(1-2)" H 2450 2250 50  0000 L CNN
F 2 "OLIMEX_Jumpers-FP:SJ_2_SMALL_12_TIED" H 2280 2450 20  0001 C CNN
F 3 "" H 2250 2300 60  0000 C CNN
	1    2250 2300
	-1   0    0    1   
$EndComp
Connection ~ 2600 2100
Wire Wire Line
	2450 2300 2700 2300
Wire Wire Line
	2250 2500 2250 2650
Connection ~ 2250 2650
Wire Wire Line
	2700 2200 2600 2200
Wire Wire Line
	2600 2200 2600 2650
Connection ~ 2600 2650
Text Notes 3000 3000 0    60   ~ 0
I2C ADDRESS SELECT:\n0x77 - GND \n0x76 - VDDIO (default)
$Comp
L MOD-BME280_RevA:C_Small C2
U 1 1 5964984B
P 2600 2800
F 0 "C2" H 2692 2846 50  0000 L CNN
F 1 "100nF" H 2692 2755 50  0000 L CNN
F 2 "OLIMEX_RLC-FP:C_0603_5MIL_DWS" H 2600 2800 50  0001 C CNN
F 3 "" H 2600 2800 50  0000 C CNN
	1    2600 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 2900 2600 2950
$Comp
L MOD-BME280_RevA:GND #PWR05
U 1 1 59649852
P 2600 2950
F 0 "#PWR05" H 2600 2700 50  0001 C CNN
F 1 "GND" H 2605 2777 50  0000 C CNN
F 2 "" H 2600 2950 50  0000 C CNN
F 3 "" H 2600 2950 50  0000 C CNN
	1    2600 2950
	1    0    0    -1  
$EndComp
NoConn ~ 5000 2550
NoConn ~ 5000 2750
NoConn ~ 5000 2850
NoConn ~ 5600 2550
NoConn ~ 5600 2750
NoConn ~ 5600 2850
$Comp
L MOD-BME280_RevA:BH10S P1
U 1 1 59B630E2
P 5300 2650
F 0 "P1" H 5350 3067 50  0000 C CNN
F 1 "BH10" H 5350 2976 50  0000 C CNN
F 2 "OLIMEX_Connectors-FP:UEXTM-SMD" H 5300 2650 50  0001 C CNN
F 3 "" H 5300 2650 50  0001 C CNN
	1    5300 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 2300 4800 2300
Wire Wire Line
	4200 2200 4700 2200
Wire Wire Line
	4900 2450 5000 2450
Wire Wire Line
	4200 1600 4900 1600
Wire Wire Line
	4800 2650 4800 3000
Wire Wire Line
	4600 2450 4900 2450
Wire Wire Line
	4700 2200 5700 2200
Wire Wire Line
	3800 2450 3800 2650
Wire Wire Line
	2100 2650 2100 2700
Wire Wire Line
	2600 1700 3800 1700
Wire Wire Line
	2600 2100 2700 2100
Wire Wire Line
	2250 2650 2100 2650
Wire Wire Line
	2600 2650 2250 2650
Wire Wire Line
	2600 2650 2600 2700
$Comp
L MOD-BME280_RevA:CON4 P2
U 1 1 59B63183
P 4600 3100
F 0 "P2" V 4473 2812 50  0000 R CNN
F 1 "Conn_01x04" V 4564 2812 50  0000 R CNN
F 2 "OLIMEX_Connectors-FP:SIP4_SMD" H 4600 3100 50  0001 C CNN
F 3 "" H 4600 3100 50  0001 C CNN
	1    4600 3100
	0    -1   -1   0   
$EndComp
$EndSCHEMATC