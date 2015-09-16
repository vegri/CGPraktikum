
import sympy as sp

lst=sp.symbols("x,y,z,a1,a2,a3,b1,b2,b3,c1,c2,c3,d1,d2,d3")

x=lst[0:3]
A=lst[3:12]
d=lst[12:15]

A=sp.Matrix([A[0:3],A[3:6],A[6:9]])
x=sp.Matrix([[i] for i in x])
d=sp.Matrix([[i] for i in d])
