
Circuit: * simulation of test

Error on line 56 : .alias bc547 bc546b npn
	 unimplemented control card - error 
Error on line 69 : .alias irf530 irf130 subckt
	 unimplemented control card - error 
Doing analysis at TEMP = 27.000000 and TNOM = 27.000000


No. of Data Rows : 1

result_show:
No matching instances or models
No matching instances or models
 Vsource: Independent voltage source
     device                    v1
         dc                    12
      acmag                     0
      pulse         -
       sine         -
        sin         -
        exp         -
        pwl         -
       sffm         -
         am         -
          i            -0.0121606
          p              0.145928

 BJT: Bipolar Junction Transistor
     device                    q1
      model                 bc337
         ic             0.0119208
         ib           0.000206736
         ie            -0.0121275
        vbe              0.725619
        vbc              0.649451
         gm              0.457762
        gpi            0.00265269
        gmu             0.0043484
         gx                     0
         go             0.0289991
        cpi           2.50477e-10
        cmu           4.49217e-11
        cbx                     0
        ccs                     0

 Resistor: Simple linear resistor
     device                    r2
      model                     R
 resistance                 22000
         ac                 22000
      dtemp                     0
      noisy                     1
          i          -3.30929e-05
          p           2.40931e-05

 Resistor: Simple linear resistor
     device                    r1
      model                     R
 resistance                 47000
         ac                 47000
      dtemp                     0
      noisy                     1
          i          -0.000239829
          p            0.00270334

 Resistor: Simple linear resistor
     device                    rl
      model                     R
 resistance                  1000
         ac                  1000
      dtemp                     0
      noisy                     1
          i            -0.0119208
          p              0.142106




result_op_start:
v(1) = 1.200000e+01
v(2) = 7.280441e-01
v(3) = 7.918878e-02
v1#branch = -1.21606e-02
result_op_end
