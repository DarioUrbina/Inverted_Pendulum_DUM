import math as m
import numpy as np

motors=4
power_blocks=3
steps=30
activation_profile = np.zeros(shape=(steps*power_blocks,motors))
line_count=0


for i in range (steps):
    for j in range(power_blocks):
        for p in range(motors):
            if (p % 2) == 0:
                activation_profile[line_count][p]=m.sin(i)
            else:
                activation_profile[line_count][p]=m.sin(i+3.14)
            print (p)
        line_count=line_count+1
"""           
for i in range (100):
    for j in range(8):
        for k in range(100):
            a[n][k]=m.sin(i)
        n=n+1
"""

print(activation_profile)

