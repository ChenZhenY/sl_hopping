function u = BezierCurve(ctrl_pt, t)

% n = length(ctrl_pt);
% u = 0;
%     for i = 1:n
%         u = u + nchoosek(n-1, i-1)*((1-t)^(n-i))*(t^(i-1))*ctrl_pt(i); % compute return value. Write your code instead of 1.
%     end

n = length(ctrl_pt)-1; % 5
u = 0;
    for i = 0:n
        u = u + nchoosek(n, i)*((1-t)^(n-i))*(t^(i))*ctrl_pt(i+1); % compute return value. Write your code instead of 1.
    end
end

