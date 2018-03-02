function y = hfunction(x, Lx, Ly)
    %% function h, used in linearization
    % Unpack value from x
    rx = x(1);
    ry = x(2);
    th = x(3);
    % Compute Threshold Angle
    th_abs = abs(x(3));
    tht = atan((Ly-ry) / (Lx - rx));
    thb = atan((ry) / (Lx - rx));
    thr = atan((Lx-rx)/ry);
    thl = atan(rx/ry);
    c = cos(th_abs);
    s = sin(th_abs);
    if (th >= 0 && th_abs <= tht && th_abs <= thr)
        disp("Case1: th >= 0 && th_abs <= tht && th_abs <= thr");      
        y = [(Lx-rx)/c; ry/c; th];
    elseif (th >= 0 && th_abs <= tht && th_abs >= thr)
        disp("Case2: th >= 0 && th_abs <= tht && th_abs >= thr");    
        y = [(Lx-rx)/c; (Lx-rx)/s; th];
    elseif (th >= 0 && th_abs >= tht && th_abs <= thr)
        disp("Case3: th >= 0 && th_abs >= tht && th_abs <= thr");    
        y = [(Lx-ry)/s; ry/c; th];
    elseif (th >= 0 && th_abs >= tht && th_abs >= thr)
        disp("Case4: th >= 0 && th_abs >= tht && th_abs >= thr");    
        y = [(Lx-ry)/s; (Lx-rx)/s; th];
    elseif (th <= 0 && th_abs <= thb && th_abs <= thl)
        disp("Case5: th <= 0 && th_abs <= thb && th_abs <= thl");    
        y = [(Lx-rx)/c; ry/c; th];
    elseif (th <= 0 && th_abs <= thb && th_abs >= thl)
        disp("Case6: th <= 0 && th_abs <= thb && th_abs >= thl");    
        y = [(Lx-rx)/c; rx/s; th];
    elseif (th <= 0 && th_abs >= thb && th_abs <= thl)
        disp("Case7: th <= 0 && th_abs >= thb && th_abs <= thl");    
        y = [ry/s; ry/c; th];
    elseif (th <= 0 && th_abs >= thb && th_abs >= thl)
        disp("Case8: th <= 0 && th_abs >= thb && th_abs >= thl");    
        y = [ry/s; rx/s; th];
    end
end

