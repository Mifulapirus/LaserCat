EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 1
Title "LaserCat"
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Transistor_FET:BUZ11 Q?
U 1 1 5F283BDA
P 5550 4850
F 0 "Q?" H 5755 4896 50  0001 L CNN
F 1 "BUZ100" H 5755 4850 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 5800 4775 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BU/BUZ11.pdf" H 5550 4850 50  0001 L CNN
	1    5550 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:Laserdiode_1A3C LD?
U 1 1 5F284C29
P 5650 4250
F 0 "LD?" H 5600 4025 50  0001 C CNN
F 1 "Laser" V 5600 4162 50  0000 R CNN
F 2 "" H 5550 4225 50  0001 C CNN
F 3 "~" H 5680 4050 50  0001 C CNN
	1    5650 4250
	0    -1   -1   0   
$EndComp
$Comp
L Motor:Motor_Servo M?
U 1 1 5F288036
P 6200 3850
F 0 "M?" H 6532 3915 50  0001 L CNN
F 1 "Servo Tilt" H 6532 3869 50  0000 L CNN
F 2 "" H 6200 3660 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 6200 3660 50  0001 C CNN
	1    6200 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F28A5F3
P 4400 4600
F 0 "#PWR?" H 4400 4350 50  0001 C CNN
F 1 "GND" H 4405 4427 50  0000 C CNN
F 2 "" H 4400 4600 50  0001 C CNN
F 3 "" H 4400 4600 50  0001 C CNN
	1    4400 4600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F28AB45
P 5400 4100
F 0 "#PWR?" H 5400 3850 50  0001 C CNN
F 1 "GND" H 5405 3927 50  0000 C CNN
F 2 "" H 5400 4100 50  0001 C CNN
F 3 "" H 5400 4100 50  0001 C CNN
	1    5400 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5F28F2C9
P 5650 5200
F 0 "#PWR?" H 5650 4950 50  0001 C CNN
F 1 "GND" H 5655 5027 50  0000 C CNN
F 2 "" H 5650 5200 50  0001 C CNN
F 3 "" H 5650 5200 50  0001 C CNN
	1    5650 5200
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_Servo M?
U 1 1 5F286F96
P 6200 3400
F 0 "M?" H 6532 3465 50  0001 L CNN
F 1 "Servo Pan" H 6532 3419 50  0000 L CNN
F 2 "" H 6200 3210 50  0001 C CNN
F 3 "http://forums.parallax.com/uploads/attachments/46831/74481.png" H 6200 3210 50  0001 C CNN
	1    6200 3400
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW?
U 1 1 5F286730
P 5150 3450
F 0 "SW?" H 5150 3735 50  0001 C CNN
F 1 "Push Button" H 5150 3643 50  0000 C CNN
F 2 "" H 5150 3650 50  0001 C CNN
F 3 "~" H 5150 3650 50  0001 C CNN
	1    5150 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 3400 5650 3150
Wire Wire Line
	5650 3850 5650 3400
Connection ~ 5650 3400
Wire Wire Line
	5750 3650 4800 3650
Wire Wire Line
	4800 3450 4950 3450
Wire Wire Line
	5750 3300 5750 3650
Wire Wire Line
	5350 3450 5400 3450
Wire Wire Line
	5400 3450 5400 3500
Connection ~ 5400 3500
Wire Wire Line
	4800 3850 5250 3850
Wire Wire Line
	4400 4600 4400 4350
Wire Wire Line
	4300 2600 4300 2750
$Comp
L MCU_Module:WeMos_D1_mini U?
U 1 1 5F2826F7
P 4400 3550
F 0 "U?" H 4400 2661 50  0001 C CNN
F 1 "Wemos D1 Mini" H 4400 2661 50  0000 C CNN
F 2 "Module:WEMOS_D1_mini_light" H 4400 2400 50  0001 C CNN
F 3 "https://wiki.wemos.cc/products:d1:d1_mini#documentation" H 2550 2400 50  0001 C CNN
	1    4400 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 4850 5250 4850
Wire Wire Line
	5250 4850 5250 3850
Wire Wire Line
	5400 3500 5400 3950
Wire Wire Line
	5650 3850 5900 3850
Wire Wire Line
	4800 3750 5900 3750
Wire Wire Line
	5400 3500 5900 3500
Wire Wire Line
	5650 3400 5900 3400
Wire Wire Line
	5750 3300 5900 3300
Wire Wire Line
	5650 3850 5650 4050
Connection ~ 5650 3850
Wire Wire Line
	5900 3950 5400 3950
Connection ~ 5400 3950
Wire Wire Line
	5400 3950 5400 4100
Wire Wire Line
	5650 4550 5650 4650
Wire Wire Line
	5650 5050 5650 5200
$Comp
L power:+5V #PWR?
U 1 1 5F2DD878
P 5650 3150
F 0 "#PWR?" H 5650 3000 50  0001 C CNN
F 1 "+5V" H 5665 3323 50  0000 C CNN
F 2 "" H 5650 3150 50  0001 C CNN
F 3 "" H 5650 3150 50  0001 C CNN
	1    5650 3150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5F2DD278
P 4300 2600
F 0 "#PWR?" H 4300 2450 50  0001 C CNN
F 1 "+5V" H 4315 2773 50  0000 C CNN
F 2 "" H 4300 2600 50  0001 C CNN
F 3 "" H 4300 2600 50  0001 C CNN
	1    4300 2600
	1    0    0    -1  
$EndComp
$EndSCHEMATC
