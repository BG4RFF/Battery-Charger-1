SMPS Lead Acid Battery Charger
------------------------------

This project aims to provide a PCB for a small capacity battery charger for
lead acid batteries. The context is to build a unit that will provide off-line
12V power to an external device while it is powered on, and charge the battery
when the device is turned off or when the battery capacity becomes critically
low. The battery charger itself is to be used in other projects and as such is
brought out as an independent circuit.

The input voltage may be derived from a 12V battery and as such may range from
as low as 11V up to 13V. The SMPS needed for this is a buck-boost architecture
that can work as boost for charging 12V batteries, and buck for charging 6V
batteries. The buck-boost circuit can be a little tricky to control so various
configurations are tested, such as setting the boost to a fixed value and using
the buck to bring it back to the desired voltage.

Charging algorithms are borrowed from the BMS project.

(c) K. Sarkies 16/09/2015
