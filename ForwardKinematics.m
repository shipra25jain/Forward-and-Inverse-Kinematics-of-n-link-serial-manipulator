dh=input('enter the matrix of DH-parameters ')
for i = 1:size(dh,1)
    alpha = dh(i,1);
    ai = dh(i,2);
    di = dh(i,3);
    theta = dh(i,4);
    A(1,1,i)= cosd(theta);
    A(1,2,i)= -sind(theta)*cosd(alpha);
    A(1,3,i)= sind(theta)*sind(alpha);
    A(1,4,i)= ai*cosd(theta);
    A(2,1,i)= sind(theta);
    A(2,2,i)= cosd(theta)*cosd(alpha)
    A(2,3,i)= -cosd(theta)*sind(alpha);
    A(2,4,i)= ai*sind(theta);
    A(3,1,i)= 0;
    A(3,2,i)= sind(alpha);
    A(3,3,i)= cosd(alpha);
    A(3,4,i)= di;
    A(4,1,i)= 0;
    A(4,2,i)= 0;
    A(4,3,i)= 0;
    A(4,4,i)= 1;
        
end
T = [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];
for i=1:(size(dh,1))
    T = T * A(:,:,i);
end
pitch = asind(-T(3,1));
roll= asind(T(2,1)/cosd(pitch));
yaw = acosd(T(3,3)/cosd(pitch));
disp(T);
