function [q] =  FT(dh)
for i = 1:size(dh,1)
    alpha = dh(i,1);
    ai = dh(i,2);
    di = dh(i,3);
    theta = dh(i,4);
    A(1,1,i)= cos(theta);
    A(1,2,i)= -sin(theta)*cos(alpha);
    A(1,3,i)= sin(theta)*sin(alpha);
    A(1,4,i)= ai*cos(theta);
    A(2,1,i)= sin(theta);
    A(2,2,i)= cos(theta)*cos(alpha)
    A(2,3,i)= -cos(theta)*sin(alpha);
    A(2,4,i)= ai*sin(theta);
    A(3,1,i)= 0;
    A(3,2,i)= sin(alpha);
    A(3,3,i)= cos(alpha);
    A(3,4,i)= di;
    A(4,1,i)= 0;
    A(4,2,i)= 0;
    A(4,3,i)= 0;
    A(4,4,i)= 1;
        
end
T = [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];

T_1 = T * A(:,:,1);
T_2 = T_1 * A(:,:,2);
T_3 = T_2 * A(:,:,3);
q = T_3(1:3,1:3);
end
