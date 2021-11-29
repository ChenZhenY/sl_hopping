function u = BezierCurve(ctrl_pt, t)

% n = length(ctrl_pt);
% u = 0;
%     for i = 1:n
%         u = u + factorial(n-1)/(factorial(i-1) * factorial(n-i)) * t^(i-1)*(1-t)^(n-i)*ctrl_pt(i);
%     end
% end

n = length(ctrl_pt);
u = 0;
    for i = 1:n
        % temp = u + nchoosek(n-1,i-1)*(1-t)^(n-i)*t^(i-1)*ctrl_pt(i);
        u = u + nchoosek(n-1,i-1)*(1-t)^(n-i)*t^(i-1)*ctrl_pt(i); % evaluate bezier curve at t
    end
end