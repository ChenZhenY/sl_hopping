    
for i = 1:numel(tout)
    sw_pos = position_swinging_foot(zout(:, i),p);
    sw_Cy(i) = sw_pos(2) - ground_height;      % foot height from ground
    k_pos = position_knee(zout(:, i),p);
    k_Cy(i) = k_pos(2) - ground_height;        % knee height from ground
    h_pos = position_hip(zout(:, i),p);
    h_Cy(i) = h_pos(2) - ground_height;        % hip height from ground
end