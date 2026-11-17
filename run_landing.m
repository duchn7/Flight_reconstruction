%% Landing: phone orientation + FlightAware track -> time series and 3D flight visualisation
% Position : FlightAware track log (ADS-B, .mhtml) matched to the phone clock (the phone's own GPS is not used).
% Touchdown: detected from the accelerometer (first burst of runway-contact vibration); the jet is at runway height
%            from then on. FlightAware leaves the altitude blank for the last rows at approach speed, so the height
%            is interpolated down to touchdown.
% Attitude : the phone's orientation sensor (phone lying flat, screen DOWN, top edge to the nose); from 20 s before
%            touchdown the gyro is integrated instead, because the fused pitch reads braking as a nose-down tilt.
% Helpers  : read_flightaware, get_runways, phone_rotations, integrate_gyro, phone_attitude, flight_timeseries, flight_viewer
clear; close all; clc

%% ---- Settings -------------------------------------------------------------
dataFile    = 'data_landing.mat';
faFile      = 'Flightaware track log.mhtml';
modelFile   = fullfile('3D models','md90.mat');
timeWindow  = [30 228];   % [start end] in s: playback starts at 'start' and stops at 'end' (the slider still covers the whole recording)
syncOffset  = 0;          % s, added to the phone clock to line it up with FlightAware (phone UTC + offset)
headingSource = 'phone';  % 'phone' = compass azimuth of the phone, 'track' = FlightAware course
mountYaw    = 0;          % deg the phone was turned about the vertical relative to the aircraft (0 = face down, top edge to the nose)
headingOffset = 'auto';   % deg added to the compass heading; 'auto' aligns it with the FlightAware course
gyroAttitude = true;      % gyro-integrated attitude from 20 s before touchdown (fused pitch is corrupted by braking)
groundAlt   = -3;         % m, airport elevation used as 0 height (EHAM Schiphol -3 m)
runwayAirport = 'EHAM';   % runway to draw ('' for none)
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
acc = d.Acceleration; ori = d.Orientation; ang = d.AngularVelocity;
t0 = min([acc.Timestamp(1) ori.Timestamp(1) ang.Timestamp(1)]); t0.TimeZone = '';
rel = @(T) seconds(T - t0);
tAcc = rel(acc.Timestamp); tOri = rel(ori.Timestamp); tAng = rel(ang.Timestamp);
tClock0 = t0 + seconds(syncOffset);               % UTC time of phone-recording t = 0

%% ---- FlightAware track (times converted to phone-recording seconds) ----------
FA = read_flightaware(faFile);
tFA = seconds(FA.Time - tClock0);
% Blank altitudes: slow (< 100 kt) or no speed = on the ground. Blank while still at approach speed just means FlightAware
% did not report the altitude (Schiphol: 12:00:41 and 12:00:57 at ~120 kt), so interpolate in time between the last
% reported altitude and the first on-ground row (touchdown assumed between the last airborne row and that one).
blankAlt = isnan(FA.Alt);
% Touchdown from the phone: first burst of vibration (std of |acceleration| > 5x its normal level) after mid-recording.
tTD = NaN;
an = vecnorm(acc{:,:},2,2); tq = (0:0.1:max(tAcc))'; anq = interp1(tAcc,an,tq); sq = movstd(anq,10);
base = median(sq(tq > 30 & tq < 0.8*max(tAcc)));
k = find(sq > 5*base & tq > 0.5*max(tAcc), 1);
if ~isempty(k), tTD = tq(k); fprintf('Touchdown detected from the accelerometer at t = %.1f s (%s UTC).\n', tTD, char(tClock0+seconds(tTD),'HH:mm:ss')); end
FA.Alt(blankAlt & (FA.Kts < 100 | isnan(FA.Kts) | tFA >= tTD - 2)) = groundAlt;   % (tTD = NaN: comparison is false)
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
[tA,iu] = unique(tA); latA = latA(iu); lonA = lonA(iu); UA = UA(iu);

%% ---- Common 10 Hz time base --------------------------------------------------
% The whole recording is processed and the viewer's slider covers all of it; timeWindow only sets where playback
% starts and stops.
tw = [max(timeWindow(1),0) min(timeWindow(2),max(tAcc))];
tg = (max([tA(1) min(tAcc)]) : 0.1 : min([tA(end) max(tAcc)]))';
assert(numel(tg) > 10 && any(tg >= tw(1) & tg <= tw(2)), 'timeWindow does not overlap the FlightAware track.');
tUTC = tClock0 + seconds(tg);

%% ---- Position in East/North/Up (m) -------------------------------------------
R0 = 6371000; lat0 = latA(1); lon0 = lonA(1);
toE = @(lon) deg2rad(lon - lon0) * R0 * cosd(lat0);
toN = @(lat) deg2rad(lat - lat0) * R0;
P = [toE(interp1(tA,lonA,tg,'pchip')), toN(interp1(tA,latA,tg,'pchip')), interp1(tA,UA,tg,'pchip')];
fixE = [toE(lonA) toN(latA) UA];                  % anchor points (for plotting)
vel = [gradient(P(:,1),0.1) gradient(P(:,2),0.1) gradient(P(:,3),0.1)];

%% ---- Attitude from the phone --------------------------------------------------
Rwp = phone_rotations(tOri, ori, tg);              % fused orientation -> rotation matrices (phone -> East/North/Up)
% Braking after touchdown (about -2.5 m/s^2) is read by the fused orientation as a nose-DOWN tilt (-12 deg on the
% runway). Integrate the gyro from 20 s before touchdown (steady approach, fused attitude fine), with the gyro bias
% taken from the steady approach (mean rate between 50 s and 10 s before touchdown).
if gyroAttitude && ~isnan(tTD) && tTD - 50 > tg(1) && tTD < tg(end)
    Rwp = integrate_gyro(Rwp, tg, tAng, ang, tTD-20, [tTD-50 tTD-10]);
    fprintf('Attitude from gyro integration from 20 s before touchdown (t = %.0f s).\n', tTD);
end
[heading, pitch, roll] = phone_attitude(Rwp, tg, tFA, FA, mountYaw, headingOffset, headingSource);

%% ---- Plots --------------------------------------------------------------------
flight_timeseries(sprintf('Phone sensors and FlightAware track - %s', strrep(dataFile,'_',' ')), ...
    tAcc, acc, tAng, ang, tOri, ori, tFA, FA, groundAlt, tg, heading);
D = struct('P',P,'vel',vel,'heading',heading,'pitch',pitch,'roll',roll,'tg',tg,'tUTC',tUTC,'fixE',fixE, ...
    'tStart',tw(1),'tPlayEnd',tw(2), ...
    'RW',get_runways(runwayAirport),'toE',toE,'toN',toN,'modelFile',modelFile,'noseAxis',noseAxis, ...
    'modelSizeFrac',modelSizeFrac,'realLength',realLength,'followHalfWidth',followHalfWidth, ...
    'playSpeed',playSpeed,'autoPlay',autoPlay,'makeVideo',makeVideo,'frameStep',frameStep);
view3d = flight_viewer(D);
