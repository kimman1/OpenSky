MAIN CPU: STM32F103C8
www.st.com/resource/en/datasheet/stm32f103c8.pdf

EEPROM: Some knock-off of 24C02
http://www.onsemi.com/pub_link/Collateral/CAT24C01-D.PDF

RF Chip: CC2500
www.ti.com/lit/ds/swrs040c/swrs040c.pdf

RF Frontend SiGe2431L
www.rfmw.com/datasheets/skyworks/SE2431L_DS.pdf

4 Pin Thru-Hole unpopulated header  --------------------- 
                                        	G Tx Rx +|--- S 
							 |--- +
							 |--- G
							 |--- G 
							 |--- +
							 |--- S 
				    ---------------------
						     ||||
						     A-+S


STM32F103
```
1	VBat
2	PC13	Bind
3	PC14	CC2500.3  (GDO2)
4	PC15	NC
5	PD0		12MHz
6	PD1		12MHz
7	NRST	EEPROM.8 (RST)
8	VSSA
9	VDDA	EEPROM.7 (WP)
10	PA0 	EEPROM.6 (SCL)
11	PA1 	EEPROM.5 (SDA)
12			NC

13 	PA3		Divider to supply
14			NC
15	PA5		Ain 
16	PA6		PU?
17			NC
18			NC
19			NC
20	PB2		Resistor to VSS1
21	PB10	"S" port on 4-pin socket
22	PB11	Transistor switching "S" port???
23			VSS1
24			VDD1

25	PB12	Servo 4 (with 0_Ohm pads)
26	PB13	Servo 3
27	PB14	Servo 2 (I pulled the res on this one)
28	PB15	Servo 1
29	PA8		CC2500.7 (CSn)
30	PA9		Servo 4 (with 0_Ohm to inverter (also uninverted to 4 pin thru-hole Tx)
31	PA10	CC2500.1 (SCLK) (and 4 pin thru-hole Rx?)
32	PA11	CC2500.2 (GDO1)
33	PA12	CC2500.20 (SI)
34	PA13	SWDIO (PAD!!)
35			VSS2
36			VDD2

37	PA14	SWDCLK (PAD!!)
38	PA15	SiGe.24  (CTX)
39	PB3		SiGe.21  (CPS)
40	PB4		SiGe.20  (CSD)
41	PB5		SiGe.16  (Ant.Sel)
42			NC
43			NC
44	BOOT0	(R20 unpopulated)
45	PB8		LED
46	PB9		LED
47 			VSS3
48			VDD3
```

EEPROM
```
1	PU?
2	NC
3	NC
4	NC
5	SDA		STM.11  (PA1)
6	SCL		STM.10	(PA0)
7	WP		STM.9	(VDDA)
8	VCC		STM.7	(NRST)
```

CC2500
```
1	SCLK	STM.31	(PA10)
2	GDO1	STM.32	(PA11)
3	GDO2	STM.3	(PC14)
4	
5
6
7	CSn		STM.29	(PA8)
8	26MHz
9
10	26MHz
11	
12	RF_P	SiGe.6	(TR)
13	RF_N	SiGe.6	(TR)
14
15
16
17
18
19
20	SI		STM.33	(PA12)
```


SiGe2431L
```
6	TR		CC2500.12/13
16	AntSel	STM.41	(PB5)
20	CSD		STM.40	(PB4)
21	CPS		STM.39	(PB3)
24	CTX		STM.38	(PA15)
```



