% syms l1 l2 l3 l4 l5 l6;
% syms a1 a2 a3 a4 a5 a6;
% 
% syms b1 b2 b3;
% syms x y;
% syms pi;

l1=29.1;
l2=48;
l3=38;
l6=34.5;
l5=33;
l4=60;

x=25;
y=50;

b1_p = (x+(l1/2))^2+y^2;
b1 = sqrt(b1_p);
a1 = acos( (l2^2+(l3+l6)^2-b1_p)/(2*l2*(l3+l6)) );
ll1 = l3+l6;
a2 = acos( (l2^2+b1_p-ll1^2)/(2*l2*b1) );
a3 = atan2( y,(x+l1/2) );

j1 = 1.5*pi-a2-a3

b2_p = l1^2+l2^2-2*l1*l2*cos(a2+a3);
b2 = sqrt(b2_p);
a4 = asin(l1*sin(a2+a3)/b2);
a5 = a1-a4;
b3_p = b2_p+l3^2-2*b2*l3*cos(a5);
b3 = sqrt(b3_p);

a6 = asin( b2*sin(a5)/b3 );
cc1 = 2*pi-a2-a3-a1-a6;
c3 = acos( (b3_p+l5^2-l4^2)/(2*b3*l5) );

j2 = 1.5*pi-cc1-c3