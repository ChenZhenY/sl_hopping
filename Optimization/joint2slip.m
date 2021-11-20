function [angle, length]=joint2slip(th)%,dth)
%_____________________________________________________________________
%Function description
    %Input: joint angle, joint angular velocity
    %Output: COM(with respect to E_foot) position and velocity vector
%_____________________________________________________________________

%To be fixed: both position and velocity are not accurate
p = parameters();
z = [th(1); th(2); 0; 0; 0; 0; 0; 0; 0; 0];  

%For intuition, referrence picture from lab
foot_pos = position_foot(z,p); % E position   with respect to O                           
com = COM_jumping_leg(z,p)     % COM position with respect to O

%calculate COM vector from point E
x = -foot_pos(1) + com(1)
y = -foot_pos(2) + com(2)

%calculate slip angle&length
angle = -atan(y / x)
length = sqrt(x * x + y * y)
end
