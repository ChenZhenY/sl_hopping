function [angle, length]=joint2slip(th)%,dth)
%_____________________________________________________________________
%Function description
    %Input: joint angles th = [q1 q2]
    %Output: SLIP model angle and foot length from B to E
%_____________________________________________________________________

p = parameters();
z = [th(1); th(2); 0; 0; 0; 0; 0; 0; 0; 0];  

%For intuition, referrence picture from lab
foot_pos = position_foot(z,p);  % E position   with respect to O                           
pos_hip = position_hip(z,p);     % B position with respect to O

%calculate COM vector from point E
x = -foot_pos(1) + pos_hip(1);
y = -foot_pos(2) + pos_hip(2);

%calculate slip angle&length
angle = -atan2(y , x);
length = sqrt(x * x + y * y);
end