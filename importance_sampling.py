import numpy as np
import matplotlib.pyplot as plt


def scoring_function(v):
    return 1/abs(v-0.2+0.01) + 1/abs(v-0.8 + 0.01)


# TODO: linear interpolation instead
def find_nearest_value(array, value):
    min_index = -1
    min_val = 100000000
    for i in range(len(array)):
        if abs(array[i] - value) < min_val:
            min_index = i
            min_val = abs(array[i] - value)

    return min_index

val = np.random.random_sample(1000)
val.sort()

plt.hist(val, bins='auto')
plt.xlabel('Sample value')
plt.ylabel('Number of samples')

scores = [scoring_function(v) for v in val]
cumsum = np.cumsum(scores)
max = cumsum[-1]
cumsum = [c/max for c in cumsum]

plt.figure()
plt.plot(val, scores)

plt.figure()
plt.plot(val, cumsum)
plt.xlabel('Sample value')
plt.ylabel('Weighted cululative sum')

# Resample
newval = np.random.random_sample(1000)
newval.sort()

resampled = np.zeros(1000)
for i in range(1000):
    resampled[i] = val[find_nearest_value(cumsum, newval[i])]

plt.figure()
plt.hist(resampled, bins='auto')
plt.xlabel('Sample value')
plt.ylabel('Number of samples (Resampled)')

plt.show()




