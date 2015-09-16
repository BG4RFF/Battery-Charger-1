v 20080127 1
C 40000 40000 0 0 0 title-B.sym
C 45800 46300 1 0 0 bjt-x.sym
{
T 46400 46800 5 10 1 1 0 0 1
refdes=Qx1
T 45800 46300 5 8 0 0 0 0 1
file=/home/ksarkies/Electronics-Development/spice-models/models.lib
T 45800 46300 5 8 1 1 0 0 1
value=Q2SC1815
}
C 45100 46800 1 270 0 diode-1.sym
{
T 45700 46400 5 10 0 0 270 0 1
device=DIODE
T 45500 46400 5 10 1 1 0 0 1
refdes=Dx1
T 45100 46800 5 8 1 1 0 0 1
value=D1N5231
T 45100 46800 5 8 0 0 0 0 1
file=/home/ksarkies/Electronics-Development/spice-models/models.lib
}
N 45800 46800 44800 46800 4
N 46300 46300 46300 45700 4
N 45300 45900 46300 45900 4
N 46300 47200 46300 47800 4
C 47700 48100 1 0 0 spice-subcircuit-LL-1.sym
{
T 47800 48400 5 10 0 1 0 0 1
device=spice-subcircuit-LL
T 47800 48500 5 10 1 1 0 0 1
refdes=A1
T 47800 48200 5 10 1 1 0 0 1
model-name=Q2SC1815-x
}
C 46600 47600 1 90 0 spice-subcircuit-IO-1.sym
{
T 46200 48500 5 10 0 1 90 0 1
device=spice-IO
T 46350 48450 5 10 1 1 90 0 1
refdes=P1
}
C 46000 45900 1 270 0 spice-subcircuit-IO-1.sym
{
T 46400 45000 5 10 0 1 270 0 1
device=spice-IO
T 46250 45050 5 10 1 1 270 0 1
refdes=P3
}
C 45000 47100 1 180 0 spice-subcircuit-IO-1.sym
{
T 44100 46700 5 10 0 1 180 0 1
device=spice-IO
T 44150 46850 5 10 1 1 180 0 1
refdes=P2
}
