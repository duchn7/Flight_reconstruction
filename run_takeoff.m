%% Takeoff: phone orientation + FlightAware track -> time series and 3D flight visualisation
% Position : FlightAware track log (ADS-B, .mhtml) matched to the phone clock. Before the first FlightAware fix the
%            phone's own GPS gives the taxi phase, the aircraft then waits at the threshold until the take-off roll,
%            and the roll itself (position and speed) comes from the integrated accelerometer up to the first fix.
% Attitude : the phone's orientation sensor (phone lying flat, screen DOWN, top edge to the nose); from the start of
%            the roll the gyro is integrated instead, because the fused pitch reads acceleration as a tilt.
% Liftoff  : detected from the acceleration peak; the jet stays on the (flat) runway until then.
% Helpers  : read_flightaware, get_runways, phone_rotations, integrate_gyro, phone_attitude, flight_timeseries, flight_viewer
clear; close all; clc

%% ---- Settings -------------------------------------------------------------
dataFile    = 'data_takeoff.mat';
faFile      = 'Flightaware track log.mhtml';
modelFile   = fullfile('3D models','md90.mat');
timeWindow  = [150 230];   % [start end] in s: playback starts at 'start' and stops at 'end' (the slider still covers the whole recording)
syncOffset  = 0;          % s, added to the phone clock to line it up with FlightAware (phone UTC + offset)
headingSource = 'phone';  % 'phone' = compass azimuth of the phone, 'track' = FlightAware course
mountYaw    = 0;          % deg the phone was turned about the vertical relative to the aircraft (0 = face down, top edge to the nose)
headingOffset = 'auto';   % deg added to the compass heading; 'auto' aligns it with the FlightAware course
alignGroundRoll = true;   % put the line-up point (start of the roll) on the runway line; the phone's taxi GPS fixes are kept as recorded
accelSpeed  = true;       % ground-roll position and speed from the integrated accelerometer (instead of a smooth curve)
gyroAttitude = true;      % gyro-integrated attitude from the start of the roll (fused pitch is corrupted by acceleration)
tLiftoff    = NaN;        % s, time the wheels leave the ground; NaN = detect from the acceleration peak (~189 s here)
tRollStart  = NaN;        % s, start of the take-off roll; NaN = detect automatically
groundAlt   = 190;        % m, airport elevation used as 0 height (EGGD Bristol 190 m)
runwayAirport = 'EGGD';   % runway to draw ('' for none)
modelSizeFrac = 0.05;     % main view: jet length as a fraction of the size of the path (exaggerated so you can see it)
realLength  = 36.2;       % m, true length of the aircraft (Embraer E190); the following view draws it at 1:1 scale
followHalfWidth = 0.75;   % following view: half-width of the window, in aircraft lengths (smaller = more zoomed in)
noseAxis    = '-x';       % md90 model: nose points along -x (up is +z)
playSpeed   = 1;          % initial playback speed (1, 2 or 3); can be changed in the window while playing
autoPlay    = true;      % true -> start playing immediately
makeVideo   = false;      % true -> writes flight.mp4
frameStep   = 1;          % video only: every Nth 10 Hz sample (1 = 10 frames/s; the video always plays in real time x playSpeed)

%% ---- Load phone data -------------------------------------------------------
d = load(dataFile);
acc = d.Acceleration; ori = d.Orientation; ang = d.AngularVelocity; pos = d.Position;
t0 = min([acc.Timestamp(1) ori.Timestamp(1) ang.Timestamp(1)]); t0.TimeZone = '';
rel = @(T) seconds(T - t0);
tAcc = rel(acc.Timestamp); tOri = rel(ori.Timestamp); tAng = rel(ang.Timestamp); tPos = rel(pos.Timestamp);
tClock0 = t0 + seconds(syncOffset);               % UTC time of phone-recording t = 0

%% ---- FlightAware track (times converted to phone-recording seconds) ----------
FA = read_flightaware(faFile);
tFA = seconds(FA.Time - tClock0);
% Blank altitudes: slow (< 100 kt) or no speed = on the ground; otherwise interpolate in time.
FA.Alt(isnan(FA.Alt) & (FA.Kts < 100 | isnan(FA.Kts))) = groundAlt;
if any(isnan(FA.Alt)), FA.Alt = fillmissing(FA.Alt,'linear','SamplePoints',seconds(FA.Time - FA.Time(1)),'EndValues','nearest'); end
fprintf('FlightAware: %d fixes, %s to %s UTC. Phone recording: %s to %s.\n', height(FA), ...
    char(FA.Time(1),'HH:mm:ss'), char(FA.Time(end),'HH:mm:ss'), ...
    char(tClock0,'HH:mm:ss'), char(tClock0+seconds(max(tAcc)),'HH:mm:ss'));

% Use fixes around the recording; stop at the first long gap (aircraft leaves the track)
iFA = find(tFA > min(tAcc)-90 & tFA < max(tAcc)+90);
assert(numel(iFA) >= 2, 'No FlightAware fixes overlap the phone recording - check syncOffset / files.');
cut = find(diff(tFA(iFA)) > 60, 1);
if ~isempty(cut), iFA = iFA(1:cut); end
tA = tFA(iFA); latA = FA.Lat(iFA); lonA = FA.Lon(iFA); UA = max(FA.Alt(iFA) - groundAlt, 0);

%% ---- Ground phase from the phone's own GPS, take-off roll, liftoff ----------------
inRec = tPos >= min(tAcc) & tPos <= max(tAcc);
pg = pos(inRec,:); tg_ = tPos(inRec);
assert(numel(tg_) >= 2, 'No phone GPS fixes in the recording: this does not look like a take-off from standstill.');
iEnd = find(diff(tg_) > 5, 1); if isempty(iEnd), iEnd = numel(tg_); end
assert(tFA(iFA(1)) > tg_(iEnd) + 10, 'The phone GPS ground phase does not end before the first FlightAware fix.');
tGPSlost = tg_(iEnd);
if isnan(tRollStart)     % take-off roll = sustained forward (phone Y) acceleration step vs the parked level
    tRoll = NaN;
    tq = (tGPSlost:0.1:max(tAcc))'; ay = interp1(tAcc, acc.Y, tq);
    parked = median(ay(tq > tGPSlost+10 & tq < tGPSlost+60));
    k = find(abs(movmean(ay,20) - parked) > 1 & tq > tGPSlost + 30, 1);
    if ~isempty(k), tRoll = tq(k); end
else
    tRoll = tRollStart;
end
if isnan(tRoll) || tRoll >= tA(1)-2, tRoll = tA(1) - 30; end
fprintf('Phone GPS: %.0f s of taxi, then no GPS; take-off roll starts at t = %.0f s (%s UTC).\n', ...
    tGPSlost, tRoll, char(tClock0+seconds(tRoll),'HH:mm:ss'));
gLat = pg.latitude(1:iEnd); gLon = pg.longitude(1:iEnd);     % taxi fixes: used exactly as recorded (the phone GPS is correct there)
holdLat = gLat(end); holdLon = gLon(end);                    % where the take-off roll starts (line-up point)
if alignGroundRoll
    % The phone GPS stops at tGPSlost, but the aircraft did not: it carried on to the runway and turned onto it (the compass
    % swings ~180 deg between 50 and 125 s), so the last GPS fix is NOT where the roll starts. Put the line-up point on the
    % runway line instead: on the backward extension of the departure course through the first FlightAware fix, at the
    % distance L before it (L = distance from the last GPS fix to the first FlightAware fix, i.e. the length of the roll).
    % The taxi fixes themselves are not moved; the path between the last fix and the line-up point is interpolated.
    Rm = 6371000; cF = FA.Course(iFA(1));
    dE = deg2rad(gLon(end)-lonA(1))*Rm*cosd(latA(1)); dN = deg2rad(gLat(end)-latA(1))*Rm;   % last fix rel. to first FlightAware fix
    L  = hypot(dE,dN);
    holdLat = latA(1) + rad2deg(-L*cosd(cF)/Rm); holdLon = lonA(1) + rad2deg(-L*sind(cF)/(Rm*cosd(latA(1))));
    fprintf('Line-up point put on the departure course (%.0f deg), %.0f m from the last phone GPS fix (taxi fixes unchanged).\n', ...
        cF, hypot(deg2rad(holdLon-gLon(end))*Rm*cosd(latA(1)), deg2rad(holdLat-gLat(end))*Rm));
end
tA   = [tg_(1:iEnd); tRoll; tA];
latA = [gLat;  holdLat;  latA];
lonA = [gLon;  holdLon;  lonA];
UA   = [zeros(iEnd+1,1); UA];
% Liftoff: keep the jet on the (flat) runway until the wheels leave the ground, then climb to the first FlightAware
% fix. Liftoff is taken as the peak of the (1 s smoothed) acceleration magnitude between 20 s after the start of the
% roll and the first FlightAware fix: the load factor peaks (~1.2 g) as the aircraft rotates and lifts off.
tFirstFA = tA(iEnd+2);
if isnan(tLiftoff)
    tq = (tRoll+20 : 0.1 : tFirstFA)';
    if numel(tq) > 10
        an = movmean(interp1(tAcc, vecnorm(acc{:,:},2,2), tq), 10);
        [~,jm] = max(an); tLiftoff = tq(jm);
    end
end
if ~isnan(tLiftoff) && tLiftoff > tRoll && tLiftoff < tFirstFA - 0.5
    latL = interp1(tA,latA,tLiftoff,'pchip'); lonL = interp1(tA,lonA,tLiftoff,'pchip');
    tA = [tA; tLiftoff]; latA = [latA; latL]; lonA = [lonA; lonL]; UA = [UA; 0];      % on the runway at liftoff
    fprintf('Liftoff taken at t = %.1f s (%s UTC): jet stays on the runway until then.\n', tLiftoff, char(tClock0+seconds(tLiftoff),'HH:mm:ss'));
end
[tA,iu] = unique(tA); latA = latA(iu); lonA = lonA(iu); UA = UA(iu);

%% ---- Common 10 Hz time base --------------------------------------------------
% All processing runs on the FULL recording (the gyro bias, the parked acceleration level and the integration all need
% the stretch before the roll). The slider in the viewer covers all of it; timeWindow only sets where playback starts.
tw = [max(timeWindow(1),0) min(timeWindow(2),max(tAcc))];
tg = (max([tA(1) min(tAcc)]) : 0.1 : min([tA(end) max(tAcc)]))';
assert(numel(tg) > 10 && any(tg >= tw(1) & tg <= tw(2)), 'timeWindow does not overlap the FlightAware track.');

%% ---- Position in East/North/Up (m) -------------------------------------------
R0 = 6371000; lat0 = latA(1); lon0 = lonA(1);
toE = @(lon) deg2rad(lon - lon0) * R0 * cosd(lat0);
toN = @(lat) deg2rad(lat - lat0) * R0;
P = [toE(interp1(tA,lonA,tg,'pchip')), toN(interp1(tA,latA,tg,'pchip')), interp1(tA,UA,tg,'pchip')];
fixE = [toE(lonA) toN(latA) UA];                  % anchor points (for plotting)
vel = [gradient(P(:,1),0.1) gradient(P(:,2),0.1) gradient(P(:,3),0.1)];

%% ---- Attitude from the phone --------------------------------------------------
Rwp = phone_rotations(tOri, ori, tg);              % fused orientation -> rotation matrices (phone -> East/North/Up)
% From the start of the roll the fused orientation reads the forward acceleration (~2.3 m/s^2) as a nose-up tilt
% (11 deg while the aircraft is still on the runway). Integrate the gyro instead, starting from the fused attitude
% while parked and with the gyro bias removed (bias = mean rate while parked).
if gyroAttitude && tRoll > tg(1) + 30 && tRoll < tg(end)
    Rwp = integrate_gyro(Rwp, tg, tAng, ang, tRoll, [tRoll-25 tRoll-3]);
    fprintf('Attitude from gyro integration after take-off roll start (t = %.0f s).\n', tRoll);
end
% Take-off roll: position along the runway from the accelerometer. Rotate the acceleration into East/North/Up with the
% (gyro-based) attitude, remove gravity + bias (mean while parked), take the component along the runway line and
% integrate twice from the start of the roll (speed 0). The distance is scaled slightly (a few %) so that it ends
% exactly at the first FlightAware fix; the path before that fix then follows the measured acceleration instead of
% an arbitrary smooth curve, and the speed shown is the integrated one.
if accelSpeed && tRoll > tg(1) + 30 && tFirstFA < tg(end)
    k0 = find(tg >= tRoll, 1); kF = find(tg >= tFirstFA, 1);
    Araw = interp1(tAcc, acc{:,:}, tg);
    Awr = zeros(numel(tg),3);
    for k = 1:numel(tg), Awr(k,:) = (Rwp(:,:,k)*Araw(k,:)')'; end
    Awr = Awr - mean(Awr(tg >= tRoll-25 & tg <= tRoll-3, :), 1);       % remove gravity + bias measured while parked
    uRun = P(kF,1:2) - P(k0,1:2); LRun = norm(uRun); uRun = uRun/LRun;   % runway line: parked point -> first FlightAware fix
    vRun = zeros(numel(tg),1);
    for k = k0+1:kF, vRun(k) = vRun(k-1) + 0.5*(Awr(k,1:2)*uRun' + Awr(k-1,1:2)*uRun')*0.1; end
    sRun = cumtrapz(tg, vRun); kappa = LRun / sRun(kF);
    P(k0:kF,1:2) = P(k0,1:2) + kappa*sRun(k0:kF)*uRun;
    vel = [gradient(P(:,1),0.1) gradient(P(:,2),0.1) gradient(P(:,3),0.1)];
    kLo = find(tg >= tLiftoff, 1);
    fprintf('Ground roll from the accelerometer: distance scaled by %.2f to end at the first FlightAware fix; speed at liftoff %.0f kt.\n', ...
        kappa, kappa*vRun(kLo)*1.944);
end
[heading, pitch, roll] = phone_attitude(Rwp, tg, tFA, FA, mountYaw, headingOffset, headingSource);

tUTC = tClock0 + seconds(tg);                      % the slider covers the whole recording; timeWindow only sets where playback starts / stops

%% ---- Plots --------------------------------------------------------------------
flight_timeseries(sprintf('Phone sensors and FlightAware track - %s', strrep(dataFile,'_',' ')), ...
    tAcc, acc, tAng, ang, tOri, ori, tFA, FA, groundAlt, tg, heading);
D = struct('P',P,'vel',vel,'heading',heading,'pitch',pitch,'roll',roll,'tg',tg,'tUTC',tUTC,'fixE',fixE, ...
    'tStart',tw(1),'tPlayEnd',tw(2), ...
    'RW',get_runways(runwayAirport),'toE',toE,'toN',toN,'modelFile',modelFile,'noseAxis',noseAxis, ...
    'modelSizeFrac',modelSizeFrac,'realLength',realLength,'followHalfWidth',followHalfWidth, ...
    'playSpeed',playSpeed,'autoPlay',autoPlay,'makeVideo',makeVideo,'frameStep',frameStep);
view3d = flight_viewer(D);
