# Java RBF

This is an implementation of Radial Basis Function interpolation, using the WPILib matrix primitives.

The general idea is to represent interpolation as a linear system:

$A\boldsymbol{\lambda}=\boldsymbol{f}$


Background on this idea and other examples:

http://www.jessebett.com/Radial-Basis-Function-USRA/

(an accessible explanation of the concept)

https://en.wikipedia.org/wiki/Radial_basis_function_interpolation

https://github.com/scipy/scipy/blob/main/scipy/interpolate/_rbfinterp.py

(the python version combines an RBF term with a polynomial term)

https://github.com/haifengl/smile/blob/master/base/src/main/java/smile/interpolation/RBFInterpolation.java

