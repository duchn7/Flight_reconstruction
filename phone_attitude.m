function [heading, pitch, roll] = phone_attitude(Rwp, tg, tFA, FA, mountYaw, headingOffset, headingSource)
%PHONE_ATTITUDE Aircraft heading, pitch and roll (deg) from the phone's rotation matrices.
%   Phone lying flat, screen DOWN. mountYaw = angle (deg) the phone was turned about the vertical relative to the
%   aircraft (0 = top edge towards the nose; 'auto' = search for the angle that best matches the FlightAware course).
%   headingOffset = deg added to the compass heading ('auto' = align with the FlightAware course while airborne).
noseWorld = @(beta) squeeze(pagemtimes(Rwp, [-sind(beta); cosd(beta); 0]));   % 3 x N, phone top edge turned by beta
headOf    = @(n) mod(atan2d(n(1,:),n(2,:)),360)';

cE = interp1(tFA, sind(FA.Course), tg); cN = interp1(tFA, cosd(FA.Course), tg);
fast = interp1(tFA, FA.Kts, tg) > 80 & ~isnan(cE);
cC = atan2d(cE(fast),cN(fast));                    % FlightAware course where airborne
if ischar(mountYaw) || isstring(mountYaw)          % 'auto': search for the mounting angle
    if any(fast)
        bs = 0:359; err = zeros(size(bs));
        for i = 1:numel(bs), n = noseWorld(bs(i)); h = headOf(n); err(i) = mean(1-cosd(h(fast)-cC)); end
        [~,ib] = min(err); beta = bs(ib);
        fprintf('Phone mounting yaw (auto): %d deg.\n', beta);
    else
        beta = 0; warning('No airborne FlightAware fixes in range - mountYaw = 0.');
    end
else
    beta = mountYaw;                               % default 0: face down, top edge towards the nose
end
n_p = [-sind(beta); cosd(beta); 0];                % nose in phone axes
r_p = [-n_p(2); n_p(1); 0];                        % right wing in phone axes (phone lies screen DOWN)
nW = squeeze(pagemtimes(Rwp, n_p)); rW = squeeze(pagemtimes(Rwp, r_p));
heading = headOf(nW);                              % compass heading of the nose (deg)
% The phone's compass is unreliable inside an aircraft (and may be offset), so align it to the FlightAware
% course. Pitch and roll do NOT depend on this - they come from gravity / the sensor fusion alone.
if ischar(headingOffset) || isstring(headingOffset)
    if any(fast)
        dh = cC - heading(fast);
        headingOffset = atan2d(mean(sind(dh)), mean(cosd(dh)));
        fprintf('Phone compass vs FlightAware course: offset %.0f deg applied to the heading (RMS error afterwards %.1f deg).\n', ...
            headingOffset, sqrt(mean((mod(dh - headingOffset + 180, 360) - 180).^2)));
    else
        headingOffset = 0;
    end
end
heading = mod(heading + headingOffset, 360);
pitch   = asind(nW(3,:))';                         % nose-up positive
roll    = asind(max(min(-rW(3,:)'./cosd(pitch),1),-1));   % right-wing-down positive
pitch = smoothdata(pitch,'gaussian',10); roll = smoothdata(roll,'gaussian',10);
if strcmpi(headingSource,'track')
    hT = mod(rad2deg(interp1(tFA, unwrap(deg2rad(FA.Course)), tg, 'linear', NaN)), 360);
    heading(~isnan(hT)) = hT(~isnan(hT));
end
end
