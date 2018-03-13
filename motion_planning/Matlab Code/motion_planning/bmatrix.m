function [ B ] = bmatrix(x, u, Cv, Cr)
    % Constant
    W = sqrt((85/2)^2 + 115^2); % robot diagonal 14cm
    offset = atan((85/2)/115);
    theta = x(3);
    tau_L = u(1);
    tau_R = u(2);
    p_str = (tau_L * tau_R) > 0;
    c = cos(theta);
    s = sin(theta);
    a1 = theta - offset;
    a2 = theta + Cr*tau_R - offset;
    a = p_str*Cv*c+(1-p_str)*W*(cos(a1)-cos(a2));
    b = p_str*Cv*s+(1-p_str)*W*(sin(a2)-sin(a1));
    %a = p_str*Cv*c-(1-p_str)*(W*(1-cos(Cr*tau_R)));
    %b = p_str*Cv*s+(1-p_str)*W*(sin(Cr*tau_R));
    c = (1-p_str)*Cr;
    B = [0 a; 0 b; 0 c];
end

