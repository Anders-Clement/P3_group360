clear; clc;

syms theta1 theta2 theta3 theta4

d1 = 0.055;
a3 = 0.22;
a4 = 0.15;

%   theta       a(1-i)     d         alpha(1-i)
 A=[theta1        0        d1         0;
    theta2+pi/2   0        0          pi/2;
    theta3        a3       0          0;
    theta4        a4       0          pi/2];
 
for i=1:4

    T(:,:,i)=[cos(A(i,1)) (-sin(A(i,1))) 0 A(i,2); 
    sin(A(i,1))*cos(A(i,4)) cos(A(i,1))*cos(A(i,4)) -sin(A(i,4)) -sin(A(i,4))*A(i,3);
    sin(A(i,1))*sin(A(i,4)) cos(A(i,1))*sin(A(i,4)) cos(A(i,4)) cos(A(i,4))*A(i,3);
    0 0 0 1];
        
    
end

F=T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)

%rotation in euler angels

Rotation=rad2deg(tr2eul(F))

%rotation in rpy angels with rotation xyz as model uses as default.
%tr2rpy(F, 'deg', 'xyz')





