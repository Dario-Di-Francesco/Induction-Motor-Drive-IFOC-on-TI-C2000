function [w_ref, a_ref] = w_a_profile_quintic_soft_2(t, wn)
% Profilo velocità + accelerazione "soft" con quintica (minimum-jerk)
% Timeline richiesta:
%  0-3   s:  w = 0
%  3-4   s:  0     ->  wn     (quintica)
%  4-5   s:  w = wn
%  5-6   s:  wn    ->  2*wn   (quintica)
%  6-7   s:  w = 2*wn
%  7-9   s:  2*wn  -> -wn     (quintica)
%  9-10  s:  w = -wn
%
% INPUT:
%   t  [s]  (scalare)
%   wn [rad/s]
% OUTPUT (single):
%   w_ref [rad/s]
%   a_ref [rad/s^2]

% --- forza single ---
t  = single(t);
wn = single(wn);

% ===== quintica minimum-jerk =====
% w(tau) = w0 + (w1-w0) * (10 tau^3 - 15 tau^4 + 6 tau^5)
% a(tau) = (w1-w0)/T * (30 tau^2 - 60 tau^3 + 30 tau^4)
p  = @(tau) (single(10).*tau.^3 - single(15).*tau.^4 + single(6).*tau.^5);
dp = @(tau) (single(30).*tau.^2 - single(60).*tau.^3 + single(30).*tau.^4);
clip01 = @(x) min(max(x, single(0)), single(1));

    function [w,a] = seg_quintic(tloc, t0, T, w0, w1)
        % Segmento quintico da t0 a t0+T
        if tloc <= t0
            w = w0; a = single(0);
        elseif tloc >= (t0 + T)
            w = w1; a = single(0);
        else
            tau = clip01((tloc - t0)./T);
            dw  = (w1 - w0);
            w   = w0 + dw .* p(tau);
            a   = (dw./T) .* dp(tau);
        end
        w = single(w); a = single(a);
    end

% ===== tempi =====
t0 = single(0);
t1 = single(1);
t2 = single(3);
t3 = single(4);
t4 = single(6);
t5 = single(7);
t6 = single(9.5);
t7 = single(10);

% ===== livelli =====
w0 = single(0);
w1n = wn;
w2n = single(2)*wn;
wneg = -wn;

% ===== profilo a tratti =====
if t < t1
    % 0-3: fermo
    w_ref = w0;
    a_ref = single(0);

elseif t < t2
    % 3-4: 0 -> wn
    [w_ref, a_ref] = seg_quintic(t, t1, (t2-t1), w0, w1n);

elseif t < t3
    % 4-5: hold wn
    w_ref = w1n;
    a_ref = single(0);

elseif t < t4
    % 5-6: wn -> 2wn
    [w_ref, a_ref] = seg_quintic(t, t3, (t4-t3), w1n, w2n);

elseif t < t5
    % 6-7: hold 2wn
    w_ref = w2n;
    a_ref = single(0);

elseif t < t6
    % 7-9: 2wn -> -wn
    [w_ref, a_ref] = seg_quintic(t, t5, (t6-t5), w2n, wneg);

elseif t <= t7
    % 9-10: hold -wn
    w_ref = wneg;
    a_ref = single(0);

else
    % oltre 10s: resta a -wn
    w_ref = wneg;
    a_ref = single(0);
end

% forza output single
w_ref = single(w_ref);
a_ref = single(a_ref);
end
