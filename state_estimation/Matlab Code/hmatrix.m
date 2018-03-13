function [H, C] = hmatrix(x, Lx, Ly)
    % get y0 from x0 (linearization centered at x0)
    x0 = x;
    y0 = hfunction(x0, Lx, Ly);
    % unpack
    rx = x(1);
    ry = x(2);
    th = x(3);
    th_abs = abs(th);
    rx0 = x0(1);
    ry0 = x0(2);
    th0 = x0(3);
    lx0 = y0(1);
    ly0 = y0(2);
    al0 = y0(3);
    c = cos(th0);
    s = sin(th0);
    th0_abs = abs(th0);
    s_abs = sin(th0_abs);
    % Get threshold angle
    tht = atan((Ly-ry) / (Lx - rx));
    thb = atan((ry) / (Lx - rx));
    thr = atan((Lx-rx)/ry);
    thl = atan(rx/ry);
    if (th >= 0 && th_abs <= tht && th_abs <= thr)
        disp("Case1: th >= 0 && th_abs <= tht && th_abs <= thr");
        a = (Lx-rx0)*s/c^2;
        b = ry0*s/c^2;
        H = [-1/c, 0, a;
           0, 1/c, b;
           0, 0, 1
           ];
        C = [rx0/c - a*th0 + lx0;
           -ry0/c - b*th0 + ly0;
           0];
    elseif (th >= 0 && th_abs <= tht && th_abs >= thr)
        disp("Case2: th >= 0 && th_abs <= tht && th_abs >= thr");
        a = (Lx-rx0)*s/c^2;
        b = (Lx-rx0)*c/s_abs^2;
        H = [-1/c, 0, a;
            -1/s_abs, 0, -(th0/th0_abs)*b;
            0, 0, 1];
        C = [rx0/c - a*th0 + lx0;
            rx0/s_abs + b*th0_abs + ly0;
            0];
    elseif (th >= 0 && th_abs >= tht && th_abs <= thr)  
        disp("Case3: th >= 0 && th_abs >= tht && th_abs <= thr");
        a = (Ly-ry0)*c/s_abs^2;
        b = ry0*s/c^2;
        H = [0, -1/s_abs, -(th0/th0_abs)*a;
            0, 1/c, b;
            0, 0, 1];
        C = [ry0/s_abs + a*th0_abs + lx0;
            -ry0/c - b*th0 + ly0;
            0];
    elseif (th >= 0 && th_abs >= tht && th_abs >= thr)
        disp("Case4: th >= 0 && th_abs >= tht && th_abs >= thr");
        a = (Ly-ry0)*c/s_abs^2;
        b = (Lx-rx0)*c/s_abs^2;
        H = [0, -1/s_abs, (-th0/th0_abs)*a;
            -1/s_abs, 0, (-th0/th0_abs)*b
            0, 0, 1
            ];
        C = [ry0/s_abs + a*th0_abs + lx0;
            rx0/s_abs + b*th0_abs + ly0;
            0];
    elseif (th <= 0 && th_abs <= thb && th_abs <= thl)
        disp("Case5: th <= 0 && th_abs <= thb && th_abs <= thl");
        a = (Lx-rx0)*s/c^2;
        b = ry0*s/c^2;
        H = [-1/c, 0, a;
           0, 1/c, b;
           0, 0, 1
           ];
        C = [rx0/c - a*th0 + lx0;
           -ry0/c - b*th0 + ly0;
           0];
    elseif (th <= 0 && th_abs <= thb && th_abs >= thl)
        disp("Case6: th <= 0 && th_abs <= thb && th_abs >= thl");
        a = (Lx-rx0)*s/c^2;
        b = rx0*c/s_abs^2;
        H = [-1/c, 0, a;
            1/s_abs, 0, (-th/th_abs)*b;
            0, 0, 1];
        C = [rx0/c - a*th0 + lx0;
            -rx0/s_abs + b*th0_abs + ly0;
            0];
    elseif (th <= 0 && th_abs >= thb && th_abs <= thl)
        disp("Case7: th <= 0 && th_abs >= thb && th_abs <= thl");
        a = ry0*c/s_abs^2;
        b = ry0*s/c^2;
        H = [0, 1/s_abs, (-th0/th0_abs)*a;
            0, 1/c, b;
            0, 0, 1];
        C = [-ry0/s_abs + a*th0_abs + lx0;
            -ry0/c - b*th0 + ly0;
            0];
    elseif (th <= 0 && th_abs >= thb && th_abs >= thl)
        disp("Case8: th <= 0 && th_abs >= thb && th_abs >= thl");
        a = ry0*c/s_abs^2;
        b = rx0*c/s_abs^2;
        H = [0, 1/s_abs, (-th0/th0_abs)*a;
            1/s_abs, 0 (-th0/th0_abs)*b;
            0, 0, 1];
        C = [-ry0/s_abs + a*th0_abs + lx0;
            -rx0/s_abs + b*th0_abs + ly0;
            0];
    end
end

