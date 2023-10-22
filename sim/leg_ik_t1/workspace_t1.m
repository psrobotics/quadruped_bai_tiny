syms l1 l2 l3 l4 l5 l6;
syms x y;
syms pi;

syms j1 j2;

l1=29.1;
l2=48;
l3=38;
l6=34.5;
l5=33;
l4=60;

pi=3.1415926535

% j1 = 2.6515;
% j2 = 2.4754;

for i=1:10000
    j1 = pi/3+rand*(pi+pi/2-pi/3)
    j2 = pi/3+rand*(pi+pi/2-pi/3)
    
    e1 = 1.5*pi-j1;
    e2 = 1.5*pi-j2;

d1_p = l1^2+l5^2-2*l1*l5*cos(e2);
d1 = sqrt(d1_p);
o1 = asin( l5*sin(e2)/d1 );
o2 = e1-o1;

d2_p = l2^2+d1_p-2*l2*d1*cos(o2);
d2 = sqrt(d2_p);
e3 = asin(d1*sin(o2)/d2);

e4 = acos( (d2_p+l3^2-l4^2)/(2*d2*l3) );
e5 = e3+e4-(pi-e1);

x = -1*l1/2+l2*cos(e1)+(l3+l6)*cos(e5)
y = l2*sin(e1)+(l3+l6)*sin(e5)

if isreal(x) && isreal(y)
plot(x,-1*y,'b.')
hold on
end

end

grid on
line([0,0], ylim, 'Color', 'k', 'LineWidth', 1); % Draw line for Y axis.
line(xlim, [0,0], 'Color', 'k', 'LineWidth', 1); % Draw line for X axis.