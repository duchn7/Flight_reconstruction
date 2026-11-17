function Rwp = integrate_gyro(Rwp, tg, tAng, ang, tStart, biasWindow)
%INTEGRATE_GYRO Replace the attitude from time tStart onward by integrating the gyro.
%   The fused orientation is corrupted by sustained linear acceleration (it reads acceleration as a tilt), but the
%   gyro is not. Start from the fused attitude at tStart and integrate the angular velocity (phone axes, rad/s) with
%   the gyro bias removed. The bias is the mean rate over biasWindow = [t1 t2] (a steady, non-rotating stretch).
W = interp1(tAng, ang{:,:}, tg, 'linear', 'extrap');
W = W - mean(W(tg >= biasWindow(1) & tg <= biasWindow(2), :), 1);
k0 = find(tg >= tStart, 1);
for k = k0:numel(tg)-1
    Rwp(:,:,k+1) = Rwp(:,:,k) * expR(0.5*(W(k,:)+W(k+1,:))*0.1);
end
end

function R = expR(v)
% rotation matrix for the rotation vector v (rad), Rodrigues formula
th = norm(v); if th < 1e-12, R = eye(3); return, end
K = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0]/th; R = eye(3) + sin(th)*K + (1-cos(th))*K*K;
end
