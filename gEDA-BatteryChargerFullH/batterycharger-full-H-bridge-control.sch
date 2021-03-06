v 20121123 2
C 40000 40000 0 0 0 title-B.sym
T 51300 41000 9 10 1 0 0 0 1
Battery Charger using H-Bridge Buck-Boost
T 54000 40100 9 10 1 0 0 0 1
Ken Sarkies
T 54000 40400 9 10 1 0 0 0 1
25/10/2010
T 50600 40100 9 10 1 0 0 0 1
1
T 52100 40100 9 10 1 0 0 0 1
1
N 41200 49200 42300 49200 4
N 43700 47000 42900 47000 4
N 41200 46400 41200 48300 4
N 41200 48300 41800 48300 4
C 43800 50200 1 270 0 capacitor-4.sym
{
T 44900 50000 5 10 0 0 270 0 1
device=POLARIZED_CAPACITOR
T 44400 49800 5 10 1 1 0 0 1
refdes=C3
T 44500 50000 5 10 0 0 270 0 1
symversion=0.1
T 44400 49600 5 10 1 1 0 0 1
value=22uF
}
C 45400 50300 1 180 0 resistor-1.sym
{
T 45100 49900 5 10 0 0 180 0 1
device=RESISTOR
T 45000 49800 5 10 1 1 0 0 1
refdes=R1
T 44900 49600 5 10 1 1 0 0 1
value=100
T 45400 50300 5 10 0 0 90 0 1
footprint=R025
}
N 43700 50200 44500 50200 4
C 41400 49200 1 90 1 capacitor-4.sym
{
T 40300 49000 5 10 0 0 270 2 1
device=POLARIZED_CAPACITOR
T 40900 48700 5 10 1 1 0 6 1
refdes=C1
T 40700 49000 5 10 0 0 270 2 1
symversion=0.1
T 40600 48600 5 10 1 1 180 6 1
value=22uF
}
C 41900 48000 1 0 1 gnd-1.sym
C 41300 50200 1 90 1 coil-2.sym
{
T 40800 50000 5 10 0 0 90 6 1
device=COIL
T 40800 49900 5 10 1 1 180 6 1
refdes=L2
T 40600 50000 5 10 0 0 90 6 1
symversion=0.1
}
N 42900 43800 42900 47000 4
N 41200 50200 42100 50200 4
C 41600 48300 1 270 1 capacitor-1.sym
{
T 42300 47900 5 10 0 0 90 2 1
device=CAPACITOR
T 42500 47900 5 10 0 0 90 2 1
symversion=0.1
T 41600 47700 5 10 0 0 0 6 1
footprint=CK05_type_Capacitor
T 41700 49400 5 10 1 1 180 6 1
refdes=C4
T 41700 49600 5 10 1 1 180 6 1
value=0.1uF
}
C 48300 47600 1 180 1 capacitor-1.sym
{
T 47900 46900 5 10 0 0 180 6 1
device=CAPACITOR
T 47900 46700 5 10 0 0 180 6 1
symversion=0.1
T 47700 47600 5 10 0 0 90 2 1
footprint=CK05_type_Capacitor
T 48600 48000 5 10 1 1 180 6 1
refdes=C2
T 48500 47800 5 10 1 1 180 6 1
value=0.1uF
}
C 42100 49600 1 0 0 lm7805-1.sym
{
T 43700 50900 5 10 0 0 0 0 1
device=7805
T 43500 50600 5 10 1 1 0 6 1
refdes=U2
}
N 46100 46200 55200 46200 4
N 49000 46800 49000 45400 4
N 46100 47400 48300 47400 4
N 42900 43800 51800 43800 4
N 42700 42100 42700 47200 4
N 54000 44000 54900 44000 4
N 48600 47000 48600 45400 4
N 46100 46000 55200 46000 4
C 48500 45400 1 270 0 resistor-1.sym
{
T 48900 45100 5 10 0 0 270 0 1
device=RESISTOR
T 48200 45100 5 10 1 1 0 0 1
refdes=R5
T 48200 45300 5 10 1 1 0 0 1
value=10K
T 48500 45400 5 10 0 0 180 0 1
footprint=R025
}
C 48900 45400 1 270 0 resistor-1.sym
{
T 49300 45100 5 10 0 0 270 0 1
device=RESISTOR
T 49200 45100 5 10 1 1 0 0 1
refdes=R3
T 49100 45300 5 10 1 1 0 0 1
value=10K
T 48900 45400 5 10 0 0 180 0 1
footprint=R025
}
N 49000 44500 47100 44500 4
N 43700 46800 42500 46800 4
N 42500 46800 42500 41900 4
C 54000 44100 1 180 0 resistor-1.sym
{
T 53700 43700 5 10 0 0 180 0 1
device=RESISTOR
T 53500 44400 5 10 1 1 0 0 1
refdes=R16
T 53500 44200 5 10 1 1 0 0 1
value=1M
T 54000 44100 5 10 0 0 90 0 1
footprint=R025
}
C 54000 43700 1 180 0 resistor-1.sym
{
T 53700 43300 5 10 0 0 180 0 1
device=RESISTOR
T 53500 43200 5 10 1 1 0 0 1
refdes=R17
T 53500 43000 5 10 1 1 0 0 1
value=1M
T 54000 43700 5 10 0 0 90 0 1
footprint=R025
}
C 52900 42700 1 0 1 resistor-1.sym
{
T 52600 43100 5 10 0 0 0 6 1
device=RESISTOR
T 52100 43100 5 10 1 1 180 6 1
refdes=R15
T 52500 43100 5 10 1 1 180 6 1
value=1M
T 52900 42700 5 10 0 0 270 2 1
footprint=R025
}
C 52800 44200 1 180 0 lm324.sym
{
T 52600 43100 5 10 0 0 180 0 1
device=LM324
T 52000 44100 5 10 1 1 180 0 1
refdes=U3
T 52600 41700 5 10 0 0 180 0 1
footprint=DIP14
}
N 52800 44000 53100 44000 4
N 52800 43600 53100 43600 4
N 52900 42800 53100 42800 4
N 53100 42800 53100 43600 4
N 51700 42800 51700 43800 4
C 52900 44600 1 180 0 resistor-1.sym
{
T 52600 44200 5 10 0 0 180 0 1
device=RESISTOR
T 52100 44900 5 10 1 1 180 6 1
refdes=R14
T 52500 44900 5 10 1 1 180 6 1
value=1M
T 52900 44600 5 10 0 0 90 0 1
footprint=R025
}
N 53100 44000 53100 44500 4
N 53100 44500 52900 44500 4
C 51700 44400 1 270 1 gnd-1.sym
C 41200 50000 1 90 0 5V-plus-1.sym
C 46200 48300 1 0 0 5V-plus-1.sym
C 44700 47700 1 0 0 5V-plus-1.sym
C 42000 45300 1 90 0 capacitor-1.sym
{
T 41300 44900 5 10 0 0 90 0 1
device=CAPACITOR
T 41100 44900 5 10 0 0 90 0 1
symversion=0.1
T 42000 44700 5 10 0 0 0 0 1
footprint=CK05_type_Capacitor
T 41300 45800 5 10 1 1 0 0 1
refdes=C4
T 41200 45600 5 10 1 1 0 0 1
value=0.1uF
}
N 42900 49600 42900 49300 4
N 42900 49300 44000 49300 4
C 43900 49000 1 0 0 gnd-2.sym
N 54000 43600 54400 43600 4
N 41800 46200 43700 46200 4
N 41200 46400 43700 46400 4
N 42300 46600 43700 46600 4
N 47500 46000 47500 45400 4
N 47100 46200 47100 45400 4
C 47000 45400 1 270 0 resistor-1.sym
{
T 47400 45100 5 10 0 0 270 0 1
device=RESISTOR
T 46700 45100 5 10 1 1 0 0 1
refdes=R6
T 46700 45300 5 10 1 1 0 0 1
value=10K
T 47000 45400 5 10 0 0 180 0 1
footprint=R025
}
C 47400 45400 1 270 0 resistor-1.sym
{
T 47800 45100 5 10 0 0 270 0 1
device=RESISTOR
T 47700 45100 5 10 1 1 0 0 1
refdes=R4
T 47600 45300 5 10 1 1 0 0 1
value=10K
T 47400 45400 5 10 0 0 180 0 1
footprint=R025
}
N 46100 46800 55200 46800 4
N 46100 47000 55200 47000 4
C 46500 47400 1 90 0 resistor-1.sym
{
T 46100 47700 5 10 0 0 90 0 1
device=RESISTOR
T 46200 48100 5 10 1 1 180 0 1
refdes=R2
T 46300 47900 5 10 1 1 180 0 1
value=10K
T 46500 47400 5 10 0 0 0 0 1
footprint=R025
}
C 41700 41000 1 90 0 resistor-1.sym
{
T 41300 41300 5 10 0 0 90 0 1
device=RESISTOR
T 42100 41600 5 10 1 1 180 0 1
refdes=R9
T 42100 41400 5 10 1 1 180 0 1
value=5.6K
T 41700 41000 5 10 0 0 0 0 1
footprint=R025
}
C 41500 40700 1 0 0 gnd-1.sym
N 41100 41900 42500 41900 4
C 41300 41900 1 90 0 diode-1.sym
{
T 40700 42300 5 10 0 0 90 0 1
device=DIODE
T 40800 42600 5 10 1 1 180 0 1
refdes=D4
T 41000 42800 5 10 1 1 180 0 1
value=1N914
T 41300 41900 5 10 0 0 180 0 1
file=/home/ksarkies/Development-Electronics/Spice-models/models.lib
}
C 40900 42800 1 0 0 5V-plus-1.sym
C 41400 42800 1 0 0 vcc-1.sym
N 52300 47400 52300 48800 4
C 53300 49200 1 180 0 lm324.sym
{
T 53100 48100 5 10 0 0 180 0 1
device=LM324
T 52700 48400 5 10 1 1 180 0 1
refdes=U3
T 53100 46700 5 10 0 0 180 0 1
footprint=DIP14
}
N 53300 48600 53700 48600 4
N 53500 48000 53700 48000 4
N 53700 47400 53700 48600 4
N 52600 48000 52300 48000 4
C 51300 49700 1 0 0 5V-plus-1.sym
C 51700 48800 1 90 0 diode-1.sym
{
T 51100 49200 5 10 0 0 90 0 1
device=DIODE
T 52100 49600 5 10 1 1 180 0 1
refdes=D6
T 52300 49400 5 10 1 1 180 0 1
value=1N914
T 51700 48800 5 10 0 0 180 0 1
file=/home/ksarkies/Development-Electronics/Spice-models/models.lib
}
N 53300 49000 53700 49000 4
C 52300 49600 1 270 1 gnd-1.sym
N 53500 49700 53700 49700 4
N 53700 49700 53700 49000 4
N 43500 48800 52300 48800 4
N 54600 49000 55500 49000 4
N 54600 48600 55500 48600 4
N 42300 49200 42300 46600 4
C 41900 45000 1 0 1 gnd-1.sym
C 45400 50400 1 270 0 vcc-1.sym
C 42900 41100 1 270 0 connector3-3.sym
{
T 44400 40600 5 10 1 1 0 6 1
refdes=I2C
T 44550 40800 5 10 0 0 270 0 1
device=CONNECTOR_3
T 44750 40800 5 10 0 0 270 0 1
footprint=SIP3N
}
N 43100 41100 43100 46000 4
N 43100 46000 43700 46000 4
N 43400 41100 43400 45800 4
N 43400 45800 43700 45800 4
N 43700 41100 43700 45600 4
C 50300 44700 1 0 0 5V-plus-1.sym
C 50700 43800 1 90 0 diode-1.sym
{
T 51100 44300 5 10 1 1 180 0 1
refdes=D5
T 51300 44500 5 10 1 1 180 0 1
value=1N914
T 50100 44200 5 10 0 0 90 0 1
device=DIODE
T 50700 43800 5 10 0 0 180 0 1
file=/home/ksarkies/Development-Electronics/Spice-models/models.lib
}
C 41700 41900 1 90 0 resistor-1.sym
{
T 41300 42200 5 10 0 0 90 0 1
device=RESISTOR
T 42100 42600 5 10 1 1 180 0 1
refdes=R8
T 42200 42400 5 10 1 1 180 0 1
value=33K
T 41700 41900 5 10 0 0 0 0 1
footprint=R025
}
C 49500 47500 1 90 1 gnd-1.sym
C 48000 44200 1 0 1 gnd-1.sym
C 55000 43100 1 90 0 resistor-1.sym
{
T 54600 43400 5 10 0 0 90 0 1
device=RESISTOR
T 55400 43700 5 10 1 1 180 0 1
refdes=R9
T 55400 43500 5 10 1 1 180 0 1
value=5.6K
T 55000 43100 5 10 0 0 0 0 1
footprint=R025
}
C 55000 44000 1 90 0 resistor-1.sym
{
T 54600 44300 5 10 0 0 90 0 1
device=RESISTOR
T 55400 44700 5 10 1 1 180 0 1
refdes=R8
T 55500 44500 5 10 1 1 180 0 1
value=33K
T 55000 44000 5 10 0 0 0 0 1
footprint=R025
}
N 54400 43600 54400 43100 4
N 54400 43100 55500 43100 4
N 54900 44900 55500 44900 4
C 54600 48900 1 0 1 resistor-1.sym
{
T 54300 49300 5 10 0 0 0 6 1
device=RESISTOR
T 53800 49400 5 10 1 1 180 6 1
refdes=R22
T 54300 49400 5 10 1 1 180 6 1
value=20K
T 54600 48900 5 10 0 0 270 2 1
footprint=R025
}
C 53500 49600 1 0 1 resistor-1.sym
{
T 53200 50000 5 10 0 0 0 6 1
device=RESISTOR
T 53000 50000 5 10 1 1 180 6 1
refdes=R21
T 53000 50200 5 10 1 1 180 6 1
value=180K
T 53500 49600 5 10 0 0 270 2 1
footprint=R025
}
C 53500 48100 1 180 0 resistor-1.sym
{
T 53200 47700 5 10 0 0 180 0 1
device=RESISTOR
T 51800 47800 5 10 1 1 0 0 1
refdes=R20
T 51700 48000 5 10 1 1 0 0 1
value=180K
T 53500 48100 5 10 0 0 90 0 1
footprint=R025
}
C 54600 48500 1 0 1 resistor-1.sym
{
T 54300 48900 5 10 0 0 0 6 1
device=RESISTOR
T 53900 48400 5 10 1 1 180 6 1
refdes=R19
T 54400 48400 5 10 1 1 180 6 1
value=20K
T 54600 48500 5 10 0 0 270 2 1
footprint=R025
}
N 52000 42800 51700 42800 4
N 42700 47200 43700 47200 4
T 55700 48900 9 10 1 0 0 0 1
+ Iload
T 55700 44800 9 10 1 0 0 0 1
+ Vbatt
T 55700 43100 9 10 1 0 0 0 1
- Vbatt
T 55700 48600 9 10 1 0 0 0 1
- Iload
T 55500 45900 9 10 1 0 0 0 1
Buck Lower
T 55500 46700 9 10 1 0 0 0 1
Boost Upper
T 55500 46900 9 10 1 0 0 0 1
Boost Lower
T 55500 46100 9 10 1 0 0 0 1
Buck Upper
N 46100 40400 46100 42100 4
C 47100 42500 1 180 0 lm324.sym
{
T 46900 41400 5 10 0 0 180 0 1
device=LM324
T 46500 41700 5 10 1 1 180 0 1
refdes=U3
T 46900 40000 5 10 0 0 180 0 1
footprint=DIP14
}
N 47100 41900 47500 41900 4
N 47300 41300 47500 41300 4
N 47500 40400 47500 41900 4
N 46400 41300 46100 41300 4
C 45100 43000 1 0 0 5V-plus-1.sym
C 45500 42100 1 90 0 diode-1.sym
{
T 44900 42500 5 10 0 0 90 0 1
device=DIODE
T 45900 42900 5 10 1 1 180 0 1
refdes=D6
T 46100 42700 5 10 1 1 180 0 1
value=1N914
T 45500 42100 5 10 0 0 180 0 1
file=/home/ksarkies/Development-Electronics/Spice-models/models.lib
}
N 47100 42300 47500 42300 4
C 46100 42900 1 270 1 gnd-1.sym
N 47300 43000 47500 43000 4
N 47500 43000 47500 42300 4
N 48400 42300 55700 42300 4
N 48400 41900 55700 41900 4
C 48400 42200 1 0 1 resistor-1.sym
{
T 48100 42600 5 10 0 0 0 6 1
device=RESISTOR
T 47600 42700 5 10 1 1 180 6 1
refdes=R22
T 48100 42700 5 10 1 1 180 6 1
value=20K
T 48400 42200 5 10 0 0 270 2 1
footprint=R025
}
C 47300 42900 1 0 1 resistor-1.sym
{
T 47000 43300 5 10 0 0 0 6 1
device=RESISTOR
T 46800 43300 5 10 1 1 180 6 1
refdes=R21
T 46800 43500 5 10 1 1 180 6 1
value=180K
T 47300 42900 5 10 0 0 270 2 1
footprint=R025
}
C 47300 41400 1 180 0 resistor-1.sym
{
T 47000 41000 5 10 0 0 180 0 1
device=RESISTOR
T 46800 40700 5 10 1 1 0 0 1
refdes=R20
T 46700 40900 5 10 1 1 0 0 1
value=180K
T 47300 41400 5 10 0 0 90 0 1
footprint=R025
}
C 48400 41800 1 0 1 resistor-1.sym
{
T 48100 42200 5 10 0 0 0 6 1
device=RESISTOR
T 47700 41700 5 10 1 1 180 6 1
refdes=R19
T 48200 41700 5 10 1 1 180 6 1
value=20K
T 48400 41800 5 10 0 0 270 2 1
footprint=R025
}
T 55900 42200 9 10 1 0 0 0 1
+ Ibatt
T 55900 41900 9 10 1 0 0 0 1
- Ibatt
N 42700 42100 46100 42100 4
N 43500 48800 43500 47400 4
N 43500 47400 43700 47400 4
C 46500 40600 1 180 1 capacitor-1.sym
{
T 46100 39900 5 10 0 0 180 6 1
device=CAPACITOR
T 46100 39700 5 10 0 0 180 6 1
symversion=0.1
T 45900 40600 5 10 0 0 90 2 1
footprint=CK05_type_Capacitor
T 47900 40800 5 10 1 1 180 6 1
refdes=C2
T 47800 40600 5 10 1 1 180 6 1
value=4.7nF
}
N 47400 40400 47500 40400 4
N 46500 40400 46100 40400 4
C 52700 47600 1 180 1 capacitor-1.sym
{
T 52300 46900 5 10 0 0 180 6 1
device=CAPACITOR
T 52300 46700 5 10 0 0 180 6 1
symversion=0.1
T 52100 47600 5 10 0 0 90 2 1
footprint=CK05_type_Capacitor
T 54100 47800 5 10 1 1 180 6 1
refdes=C2
T 54000 47600 5 10 1 1 180 6 1
value=4.7nF
}
N 53600 47400 53700 47400 4
N 52700 47400 52300 47400 4
C 46200 48100 1 180 0 ATtiny261.sym
{
T 44200 45200 5 10 1 1 180 6 1
refdes=U1
T 45800 42050 5 10 0 0 180 0 1
device=ATtiny261
T 45800 41850 5 10 0 0 180 0 1
footprint=DIP20
}
