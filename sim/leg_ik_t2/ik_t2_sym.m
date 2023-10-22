clear;
clc;

syms l1 l2 l3 l4 l5 l6 pi x y

b1_p = x^2+y^2;
b1 = sqrt(b1_p);
a1 = atan2(y,x)
a0 = pi-a1;
b2_p = (l1/2)^2+b1_p-l1*b1*cos(a0);
b2 = sqrt(b2_p)
a2 = get_tri_rad(b1,b2,l1/2)
a3 = get_tri_rad(b2,l3+l6,l2)
a4 = get_tri_rad(l2,l3+l6,b2)
a5 = 2*pi-a0-a2-a3-a4;
j1 = 1.5*pi-a5

a7 = a4+a5-pi;
p1x = x-l6*cos(a7);
p1y = y-l6*sin(a7);
e1_p = p1x^2+p1y^2;
e1 = sqrt(e1_p);
c1 = atan2(p1y,p1x);
c2 = get_tri_rad(l5,e1,l4);
j2 = 0.5*pi+c1-c2