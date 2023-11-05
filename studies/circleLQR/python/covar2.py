# illustrates covariance of mixtures

# useful for visualizing behavior of VarianceWeightedLinearPooling

import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

random_seed = 1000
plt.figure(figsize=(15,15))

#
# input "a"
#
ax = plt.subplot(2, 2, 1)
mean = np.array([0, 0])
cov = np.array([[1, 0.5], [0.5, 2]])
distr = stats.multivariate_normal(cov=cov, mean=mean, seed=random_seed)
data1 = distr.rvs(size=5000)
mean1 = np.mean(data1,axis=0)
print("mean1 ", mean1)
cov1 = np.cov(data1,rowvar=False)
plt.plot(
    data1[:, 0],
    data1[:, 1],
    "o",
    c="lime",
    markeredgewidth=0.5,
    markeredgecolor="black",
)
plt.title(f"Input A\nMean {mean1}\nCovariance {cov1}")
ax.set_xlim(-10,10)
ax.set_ylim(-10,10)
plt.axhline()
plt.axvline()

#
# input "b"
#
ax = plt.subplot(2, 2, 2)
mean = np.array([1, 1])
cov = np.array([[2, 0.5], [0.5, 1]])
distr = stats.multivariate_normal(cov=cov, mean=mean, seed=random_seed)
data2 = distr.rvs(size=5000)
mean2 = np.mean(data2,axis=0)
print("mean2 ", mean2)
cov2 = np.cov(data2,rowvar=False)
plt.plot(
    data2[:, 0],
    data2[:, 1],
    "o",
    c="lime",
    markeredgewidth=0.5,
    markeredgecolor="black",
)
plt.title(f"Input B\nMean {mean2}\nCovariance {cov2}")
ax.set_xlim(-10,10)
ax.set_ylim(-10,10)
plt.axhline()
plt.axvline()


api = np.linalg.inv(cov1)
bpi = np.linalg.inv(cov2)
print("api" , api)
print("bpi" , bpi)
pisum = api+bpi
pisumi = np.linalg.inv(pisum)
pa = np.matmul(api , pisumi)
pb = np.matmul(bpi , pisumi)

print("pa" , pa)
print("pb" , pb)

# check the sum
pc = pa + pb
print("pc" , pc)

#
# the mixture of "a" and "b"
#
ax = plt.subplot(2, 2, 3)
data3 = np.concatenate((data1, data2))
mean3 = np.mean(data3,axis=0)
cov3 = np.cov(data3,rowvar=False)
print("mean3", mean3)
print("cov3", cov3)

# favor the tighter distribution
cx = np.matmul(pa,mean1) + np.matmul(pb,mean2)

# democratic averaging makes the picture look better
# but it ignores the tightness in the distribution
# which is a signal about accuracy
#cx = mean1 * 0.5 + mean2 * 0.5
print("calculated mean cx ", cx)

cpa = np.matmul(pa, cov1)
cpb = np.matmul(pb, cov2)
print("cpa" , cpa)
print("cpb" , cpb)

# putting the dispersion on the diagonal does the right thing
# for the diagonal elements of the covariance
# but it does nothing for the off-diagonal elements :-)
d = np.diag(mean1 - mean2)
# this yields a larger influence but only positive covariance.
#d = np.ones((2,1)) * (mean1-mean2)
print("d" , d)

# elementwise yields negative off-diagonal
#d2 = np.multiply(d,d.transpose())
# hm, just try reducing it, there's no justification for this
#d2 = np.multiply(d2, 0.25)
# matmul yields only positive
d2 = np.matmul(d,d.transpose())
print("d2" , d2)

papb = np.matmul(pa, pb)
print("papb" , papb)
# the weights here make this term too small when the variances are
# really unequal
dp = np.matmul(papb, d2)
#dp = d2
print("dp" , dp)

cp = cpa + cpb + dp
print("calculated covariance cp" , cp)


plt.plot(
    data3[:, 0],
    data3[:, 1],
    "o",
    c="lime",
    markeredgewidth=0.5,
    markeredgecolor="black",
)
plt.title(f"Mixture\nMean {mean3}\nCovariance {cov3}")
ax.set_xlim(-10,10)
ax.set_ylim(-10,10)
plt.axhline()
plt.axvline()


#
# a new dist with the calculated mean/cov for comparison
#
ax = plt.subplot(2, 2, 4)
# use calculated instead
#distr = stats.multivariate_normal(cov=cov3, mean=mean3, seed=random_seed)
distr = stats.multivariate_normal(cov=cp, mean=cx, seed=random_seed)
data4 = distr.rvs(size=5000)
mean4 = np.mean(data4,axis=0)
cov4 = np.cov(data4,rowvar=False)
plt.plot(
    data4[:, 0],
    data4[:, 1],
    "o",
    c="lime",
    markeredgewidth=0.5,
    markeredgecolor="black",
)
plt.title(f"Fit\nMean {mean4}\nCovariance {cov4}")
ax.set_xlim(-10,10)
ax.set_ylim(-10,10)
plt.axhline()
plt.axvline()

plt.show()
