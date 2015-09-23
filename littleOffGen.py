import numpy as np

n=15

out=open('num_%d_tris.off'%n,'w')

rds=np.random.rand(n,3)

points=np.zeros((n*3,3))
idx=np.zeros((n*3,3))

points[0][0]=-750; points[0][1]=0; points[0][2]=-750
points[1][0]=-750; points[1][1]=0; points[1][2]= 750
points[2][0]= 750; points[2][1]=0; points[2][2]=-750
points[3][0]= 750; points[3][1]=0; points[3][2]= 750

idx[0][0]=0; idx[0][1]=1; idx[0][2]=2
idx[1][0]=3; idx[1][1]=1; idx[1][2]=2

i=4
j=2
l=0

while j<n:
    dx=points[idx[l][1]]-points[idx[l][0]]
    dy=points[idx[l][2]]-points[idx[l][0]]
    points[i]=points[idx[l][0]]+dx*rds[j][0]*rds[j][1]+rds[j][0]*(1-rds[j][1])*dy
    points[i][1]=(rds[j][0]-rds[j][1])*500
    idx[j][0]=idx[l][0]; idx[j][1]=idx[l][1]; idx[j][2]=i
    idx[j+1][0]=idx[l][1]; idx[j+1][1]=idx[l][2]; idx[j+1][2]=i
    idx[l][0]=idx[l][0]; idx[l][1]=idx[l][2]; idx[l][2]=i
    l+=1
    j+=2
    i+=1

out.write('OFF\n')
out.write('%d %d %d\n'%(i,j,l))
    
l=0
while l<i:
    out.write("%.5f %.5f %.5f\n"%(points[l][0],points[l][1],points[l][2]))
    l+=1
    
l=0
while l<j:
    out.write("3 %d %d %d\n"%(idx[l][0],idx[l][1],idx[l][2]))
    l+=1
    
    

out.close()