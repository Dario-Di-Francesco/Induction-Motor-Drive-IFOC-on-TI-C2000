function make_ref_profile(Tstop, Ts_ref, wn)
% Genera un timeseries con [w_ref a_ref] (single) da usare con From Workspace

Tstop  = double(Tstop);          % time vector deve essere double
Ts_ref = double(Ts_ref);
wn_s   = single(wn);

t = (0:Ts_ref:Tstop).';          % tempo (double)
N = numel(t);

w = zeros(N,1,'single');
a = zeros(N,1,'single');

for k = 1:N
    %[wk, ak] = w_a_profile_quintic_soft(single(t(k)), wn_s);
    [wk, ak] = w_a_profile_quintic_soft_2(single(t(k)), wn_s);
    
    w(k) = wk;
    a(k) = ak;
end

% Timeseries: tempo double, dati single (ok)
ref_ts = timeseries([w a], t);
ref_ts.Name = 'ref_ts';
ref_ts.Data = single(ref_ts.Data);

assignin('base','ref_ts',ref_ts);
end
