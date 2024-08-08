# Java RBF

This is an implementation of Radial Basis Function interpolation, using the WPILib matrix primitives.

The general idea is this: given an unknown function, $F:\R^n \rightarrow \R^m$:

$$ Y = F(x) $$

approximate $F$ as $S$, a linear
combination of scalar basis functions, $\varphi$:

$$F(x) \approx S(x) = \sum_{i} w_k \varphi(\|x-x_i\|)$$

Given a set of $k$ examples,

$$ Y_k = F(x_k) $$

this becomes a linear system with constant $\Phi$ and $F$:

$$ \Phi W = F $$


that can be solved for $W$, e.g. with LU or QR decomposition.

In our case, we're working on $\R^4\rightarrow\R^2$.
The domain is the image frame $(u, v)$, the apparent tag size in pixels, $d$,
and the known tag altitude, $Z$.
The range is the cartesian relative position, $(X, Y)$.

It might be better to cast the dependent variables in polar coordinates given the
vastly different variances in between $r$ and $\theta$.

Note this project requires WPILib 2025 since it uses EigenJNI.

### Background

Background on this idea and other examples:

http://www.jessebett.com/Radial-Basis-Function-USRA/

(an accessible explanation of the concept)

https://en.wikipedia.org/wiki/Radial_basis_function_interpolation

https://github.com/scipy/scipy/blob/main/scipy/interpolate/_rbfinterp.py

(the python version combines an RBF term with a polynomial term)

https://github.com/haifengl/smile/blob/master/base/src/main/java/smile/interpolation/RBFInterpolation.java

