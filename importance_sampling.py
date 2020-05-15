import numpy as np
import matplotlib.pyplot as plt

# TODO: add linear interpolation

def scoring_function(v):
    return 1/abs(v-0.2+0.01) + 1/abs(v-0.8 + 0.01)


def find_nearest_index(array, value):
    min_index = -1
    min_val = 100000000
    for i in range(len(array)):
        if (abs(array[i] - value) < min_val):
            min_index = i
            min_val = abs(array[i] - value)
    return min_index


val = np.random.random_sample(100)
val.sort()

plt.hist(val, bins='auto')

scores = [scoring_function(v) for v in val]


cumsum = np.cumsum(scores)
max = cumsum[-1]
cumsum = [c/max for c in cumsum]


plt.figure()
plt.plot(val, scores)

plt.figure()
plt.plot(val, cumsum)

# Resample
newval = np.random.random_sample(100)
newval.sort()

resampled = np.zeros(100)
for i in range(100):
    resampled[i] = val[find_nearest_index(cumsum, newval[i])]

plt.figure()
plt.hist(resampled, bins='auto')

plt.show()




