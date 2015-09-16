
import sympy as sp

lst=sp.symbols("x,y,z,a,b,c,d,e,f,g,h,i,j,k,l")

x=lst[0:3]
A=lst[3:12]
d=lst[12:15]

A=sp.Matrix([A[0:3],A[3:6],A[6:9]])
x=sp.Matrix([[i] for i in x])
d=sp.Matrix([[i] for i in d])
