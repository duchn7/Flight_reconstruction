function Rwp = phone_rotations(tOri, ori, tg)
%PHONE_ROTATIONS Rotation matrices (phone axes -> East/North/Up) from the phone's fused Orientation sensor.
%   Rwp is 3 x 3 x numel(tg). Orientation columns are azimuth, pitch, roll in degrees.
%   Convention verified on the data: with this order and signs, gravity cancels in the world frame.
Rz = @(a)[cosd(a) -sind(a) 0; sind(a) cosd(a) 0; 0 0 1];
Ry = @(a)[cosd(a) 0 sind(a); 0 1 0; -sind(a) 0 cosd(a)];
Rx = @(a)[1 0 0; 0 cosd(a) -sind(a); 0 sind(a) cosd(a)];
Ang = interp1(tOri, ori{:,:}, tg, 'nearest', 'extrap');
Rwp = zeros(3,3,numel(tg));
for k = 1:numel(tg), Rwp(:,:,k) = Rz(-Ang(k,1)) * Rx(-Ang(k,2)) * Ry(Ang(k,3)); end
end
