# BiRo Walking

Follow installation.md for setup instructions


## Basic command

Run script:
```
python3 main.py
```

Scaled biped:

```
python3 main.py -x 2
```


Step in place: 
```
python3 main.py -vx 0
```

It can track velocity while moving backwards (try vx = 1.0, 1.3, 1.8)
```
python3 main.py -vx 1
```

Walk slant: (Not tuned for velocity tracking)
```
python3 main.py -vx 0.5 -vy 0.5
```


## Run script with different slopes

For slope = 10:
```
python3 main.py -ts 10
```

For slope = 30:
```
python3 main.py -ts 30
```

For slope = 30 (backward walking scaled biped):
```
python3 main.py -ts 30 -x 2 -vx 2
```

Walk slant: (Not tuned)
```
python3 main.py -vx -0.5 -vy 0.5 -ts 30
```

For slope = 45, scaled biped:
```
python3 main.py -ts 45 -x 2
```

## Run script with sinusoidal slopes

For slope = 30:
```
python3 main.py -ss 30
```

For random sinusoidal:
```
python3 main.py -ss 30 -rs 1
```

For random sinusoidal, scaled biped (doesn't work):
```
python3 main.py -ss 30 -rs 1 -x 2
```


## Added weight mid-trajectory

Add 10% weight
```
python3 main.py -w 10
```

For slope = 30 scaled biped (backward walking with 70 Kg added weight):
```
python3 main.py -ts 30 -x 2 -vx 2 -w 185 -wk
```

For slope = 45, scaled biped (23 kg payload):
```
python3 main.py -ts 45 -x 2 -wk -w 55
```

For slope = 45 scaled biped (backward walking with 50 Kg added weight):
```
python3 main.py -ts 45 -x 2 -vx 2 -w 130 -wk
```

When the mass is added and taken off is specified in time_added, time_removed arrays in main.py


#### Notes:

##### Non-scaled biped experiments

Initial total mass = 5.47 Kg
The MPC weights are not changed for these experiments


###### Weight added known (-wk option)

Flat ground: 6.288 Kg added (114%)​
30 degree slope: 5 Kg added (91%)​
Negative 30 degree slope: 3.75 Kg added (68%)​
30 degree sinusoidal slope: 1.875 Kg added (34%)​


###### Weight added unknown

Flat ground: 5.2 Kg added (95% )​
30 degree slope: 4.1 Kg added (75%)​
Negative 30 degree slope: 1.64 Kg added (30%)​
30 degree sinusoidal slope: 0.54 Kg added (10%)​



##### Scaled biped experiments

###### Weight added known (-wk option)


Initial total mass = 38 Kg
The MPC weights are not changed for these experiments

###### Weight added known (-wk option)

Flat ground: 64.6 Kg added (170%)​
30 degree slope: 30 Kg added (87.5%)​
45 degree slope: 23 Kg added (57.5%)​
Negative 30 degree slope: 0 Kg added (0%)​
Negative 30 degree slope (reverse walking): 70.3 Kg added (185%)​
Negative 45 degree slope: 0 Kg added (0%)​
Negative 45 degree slope (reverse walking): 70.3 Kg added (185%)​
30 degree sinusoidal slope: 0 Kg added (0%)​


###### Weight added unknown

Flat ground: 7.6 Kg added (20%)​
30 degree slope: 1.9 Kg added (5%)​
Negative 30 degree slope: 0 Kg added (0%)​
30 degree sinusoidal slope: 0 Kg added (0%)​



### Debugging

 - Still some error in the code, when argument vx = -1, the code is not clipping this to -1
 - Also, the robot is not rotating. The yaw of the robot does not change

