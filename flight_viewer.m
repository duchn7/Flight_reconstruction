function h = flight_viewer(D)
%FLIGHT_VIEWER Interactive 3D flight visualisation: main view, aircraft-following view, attitude indicator,
%   time slider, play/pause and x1/x2/x3 playback speed.
%   D is a struct with fields: P (E,N,Up in m), vel, heading, pitch, roll (deg), tg (s), tUTC, fixE (anchor points),
%   RW (runways), toE, toN (lat/lon -> metres), modelFile, noseAxis, modelSizeFrac, realLength, followHalfWidth,
%   playSpeed, autoPlay, makeVideo, frameStep.   h returns the main handles (fig, ax, ax2, sld, btn, spdMenu).
P = D.P; vel = D.vel; heading = D.heading; pitch = D.pitch; roll = D.roll; tg = D.tg; tUTC = D.tUTC;
fixE = D.fixE; RW = D.RW; toE = D.toE; toN = D.toN;
Rz = @(a)[cosd(a) -sind(a) 0; sind(a) cosd(a) 0; 0 0 1];

m = load(D.modelFile);
span = max(max(P) - min(P));
V = m.V - (max(m.V)+min(m.V))/2;                   % centre the model
V = V * (max(35, D.modelSizeFrac*span) / max(max(V)-min(V)));
switch D.noseAxis                                  % re-orient so the nose lies along +Y
    case '+y'
    case '-y', V = V*Rz(180)';
    case '+x', V = V*Rz(90)';
    case '-x', V = V*Rz(-90)';
end
V(:,3) = V(:,3) - min(V(:,3));                     % lowest point of the model at height 0: at "zero height" the aircraft sits ON the ground
                                                   % (the model origin is now its underside, so pitch/roll pivot there)
modelLen = max(max(V)-min(V));

spdKmh = smoothdata(hypot(vel(:,1),vel(:,2))*3.6, 'gaussian', 20);   % ground speed along the track (km/h)
% The slider covers the whole recording; tStart / tPlayEnd (the user's time window) only set where playback begins
% and where it stops when started from before that point.
tStart = tg(1); tPlayEnd = tg(end);
if isfield(D,'tStart'),   tStart   = min(max(D.tStart,  tg(1)), tg(end)); end
if isfield(D,'tPlayEnd'), tPlayEnd = min(max(D.tPlayEnd,tg(1)), tg(end)); end

fig = figure('Name','Flight visualisation','Color','w','Position',[60 50 1400 760]);
ax = axes(fig,'Position',[0.04 0.12 0.62 0.84]); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,35,22)
xlabel('East (m)'); ylabel('North (m)'); zlabel('Height above airport (m)')
plot3(ax, P(:,1), P(:,2), P(:,3), '-', 'Color',[.8 .8 .8]);
plot3(ax, fixE(:,1), fixE(:,2), fixE(:,3), 'k.', 'MarkerSize',10);
trail = plot3(ax, NaN, NaN, NaN, 'b-', 'LineWidth',2.5);
legend(ax,{'Full path','FlightAware fixes / phone GPS (taxi)','Path so far'},'Location','northwest','AutoUpdate','off');
if isempty(RW)
    fprintf('No runway data stored for this airport - runway not drawn.\n');
end
tf = hgtransform(ax);
patch('Parent',tf,'Faces',m.F,'Vertices',V,'FaceColor',[.75 .78 .85], ...
      'EdgeColor','none','FaceLighting','gouraud','AmbientStrength',.5);
camlight(ax,'headlight'); material(ax,'dull')
pad = modelLen;
fullLim = [min(P(:,1))-pad max(P(:,1))+pad; min(P(:,2))-pad max(P(:,2))+pad; min(P(:,3))-pad max(P(:,3))+pad];
set(ax,'XLim',fullLim(1,:),'YLim',fullLim(2,:),'ZLim',fullLim(3,:));
ttl = title(ax,'');

% --- primary-flight-display style instrument (attitude, speed km/h, height m), bottom right ---
axI = axes(fig,'Position',[0.67 0.07 0.32 0.33]);
H = buildPFD(axI);

% --- aircraft-following 3D view, above the instrument, top right --------------------
ax2 = axes(fig,'Position',[0.69 0.50 0.28 0.45]); hold(ax2,'on'); grid(ax2,'on'); axis(ax2,'equal'); view(ax2,35,22)
xlabel(ax2,'East (m)'); ylabel(ax2,'North (m)'); zlabel(ax2,'Height (m)')
plot3(ax2, P(:,1), P(:,2), P(:,3), '-', 'Color',[.8 .8 .8]);
trail2 = plot3(ax2, NaN, NaN, NaN, 'b-', 'LineWidth',2.5);
drawRunways(ax, RW, toE, toN); drawRunways(ax2, RW, toE, toN);
tf2 = hgtransform(ax2);
patch('Parent',tf2,'Faces',m.F,'Vertices',V*(D.realLength/modelLen),'FaceColor',[.75 .78 .85], ...
      'EdgeColor','none','FaceLighting','gouraud','AmbientStrength',.5);
camlight(ax2,'headlight'); material(ax2,'dull')
% Mouse: dragging in either 3D view rotates BOTH views together, live; the wheel zooms the view under the pointer.
% (MATLAB's built-in rotate only updates a linked view when the mouse is released, so rotation is handled here.)
for a = [ax ax2], disableDefaultInteractivity(a); a.Toolbar.Visible = 'off'; end
fig.WindowButtonDownFcn = @(f,~) viewDown(f);
fig.WindowScrollWheelFcn = @(f,e) viewScroll(f,e);

% --- time slider and play/pause ------------------------------------------------------
sld = uicontrol(fig,'Style','slider','Units','normalized','Position',[0.19 0.01 0.73 0.035], ...
    'Min',tg(1),'Max',tg(end),'Value',tStart,'SliderStep',[1/numel(tg) 0.05]);   % slider covers ALL the data; playback starts at tStart
btn = uicontrol(fig,'Style','togglebutton','String','Play','Units','normalized','Position',[0.02 0.01 0.08 0.035]);
spdMenu = uicontrol(fig,'Style','popupmenu','String',{'x1','x2','x3'},'Units','normalized', ...
    'Position',[0.11 0.01 0.07 0.035],'Value',min(max(round(D.playSpeed),1),3));   % playback speed: x1 / x2 / x3
fig.UserData = struct('ax',ax,'tf',tf,'trail',trail,'ttl',ttl,'ax2',ax2,'tf2',tf2,'trail2',trail2,'H',H,'P',P, ...
    'spd',spdKmh,'tg',tg,'tUTC',tUTC,'heading',heading,'pitch',pitch,'roll',roll,'R',D.followHalfWidth*D.realLength,'k',1,'tStart',tStart,'tPlayEnd',tPlayEnd);
addlistener(sld,'ContinuousValueChange',@(s,~) showFrame(fig,s.Value));
sld.Callback = @(s,~) showFrame(fig,s.Value);

if D.makeVideo
    % Real-time video: one frame every frameStep samples (data spacing dt), so the frame rate must be 1/(frameStep*dt),
    % times the chosen playback speed. (The default of 30 fps made the video play ~12x too fast with frameStep = 4.)
    dt = median(diff(tg));
    vw = VideoWriter('flight.mp4','MPEG-4'); vw.FrameRate = D.playSpeed / (D.frameStep*dt); open(vw);
    closeVideo = onCleanup(@() close(vw));        % always finalise the file, even if the export is interrupted (Ctrl+C) or fails
    fig.Resize = 'off';                           % every frame must have the same size: do not resize the window while exporting
    kv = find(tg >= tStart & tg <= tPlayEnd)';
    frames = [kv(1):D.frameStep:kv(end), kv(end)];
    fprintf('Writing flight.mp4: %d frames at %.1f frames/s = real time x%g. Capturing takes about 0.3 s per frame\n', ...
        numel(frames), vw.FrameRate, D.playSpeed);
    fprintf('(the window steps through the flight slower than real time while this runs - leave it alone until it finishes).\n');
    wallStart = tic; nextReport = 0.1; nDropped = 0;
    for i = 1:numel(frames)
        showFrame(fig, tg(frames(i)));
        [fr, ok] = captureFrame(fig);
        if ok
            lastFr = fr;
        elseif exist('lastFr','var')
            fr = lastFr; nDropped = nDropped + 1;     % capture kept failing: repeat the previous picture so the video keeps its timing
        else
            error('flight_viewer:capture','Could not capture the first video frame - keep the figure window visible and not minimised.');
        end
        writeVideo(vw, fr);
        if i/numel(frames) >= nextReport
            fprintf('  %3.0f%% (%d/%d frames, about %.0f s left)\n', 100*i/numel(frames), i, numel(frames), toc(wallStart)/i*(numel(frames)-i));
            nextReport = nextReport + 0.1;
        end
    end
    clear closeVideo; fig.Resize = 'on';
    fprintf('Finished flight.mp4 (%.1f s of video).\n', numel(frames)/vw.FrameRate);
    if nDropped > 0, fprintf('Warning: %d frame(s) could not be captured and repeat the previous picture.\n', nDropped); end
end
btn.Callback = @(b,~) togglePlay(fig,sld,b,spdMenu);
fig.DeleteFcn = @(f,~) stopPlayback(f);
showFrame(fig, tStart);
% autoPlay starts live playback on a timer (MATLAB stays free for the mouse). Not after a video export: the flight has
% just been shown in the window while recording, so playing it again straight afterwards looks like the script re-running.
if D.autoPlay && ~D.makeVideo, btn.Value = 1; togglePlay(fig,sld,btn,spdMenu); end
h = struct('fig',fig,'ax',ax,'ax2',ax2,'sld',sld,'btn',btn,'spdMenu',spdMenu);
end

function [fr, ok] = captureFrame(fig)
    % getframe on a web figure occasionally fails ("Export unsuccessful", e.g. when the window is covered, minimised or
    % busy). Retry a few times, with a short pause and a redraw, before giving up on this frame.
    fr = []; ok = false;
    for attempt = 1:8
        try
            fr = getframe(fig); ok = true; return
        catch
            drawnow; pause(0.3*attempt);
            if attempt == 3, figure(fig); end          % bring the window to the front once
        end
    end
end

function drawRunways(ax, RW, toE, toN)
    % Draw each runway as a grey strip of its real width on the ground (z = 0), with a dashed centreline.
    for r = 1:numel(RW)
        x = toE(RW(r).lon); y = toN(RW(r).lat); w = RW(r).width/2;
        tx = gradient(x); ty = gradient(y); tl = hypot(tx,ty); nx = -ty./tl; ny = tx./tl;   % unit normals
        px = [x + w*nx; flipud(x - w*nx)]; py = [y + w*ny; flipud(y - w*ny)];
        patch(ax,'XData',px,'YData',py,'ZData',zeros(size(px)),'FaceColor',[.45 .45 .48],'EdgeColor','none','HandleVisibility','off');
        line(ax,x,y,zeros(size(x))+0.05,'Color',[.95 .95 .95],'LineStyle','--','LineWidth',0.8,'HandleVisibility','off');
    end
end

function showFrame(fig, tNow)
    S = fig.UserData;
    [~,k] = min(abs(S.tg - tNow)); S.k = k; fig.UserData = S;
    % compass heading h -> rotation about Up by -h (nose starts on +Y = North)
    M = makehgtform('translate',S.P(k,:)) * makehgtform('zrotate',-deg2rad(S.heading(k))) * ...
        makehgtform('xrotate',deg2rad(S.pitch(k))) * makehgtform('yrotate',deg2rad(S.roll(k)));
    S.tf.Matrix = M; S.tf2.Matrix = M;
    set(S.trail, 'XData',S.P(1:k,1),'YData',S.P(1:k,2),'ZData',S.P(1:k,3));
    set(S.trail2,'XData',S.P(1:k,1),'YData',S.P(1:k,2),'ZData',S.P(1:k,3));
    c = S.P(k,:); r = S.R;                          % following view: window centred on the aircraft
    set(S.ax2,'XLim',c(1)+[-r r],'YLim',c(2)+[-r r],'ZLim',c(3)+[-r r]);
    S.ttl.String = sprintf('%s UTC   (t = %.1f s)', char(S.tUTC(k),'HH:mm:ss'), S.tg(k));
    updatePFD(S.H, S.pitch(k), S.roll(k), S.spd(k), S.P(k,3), S.heading(k));
    drawnow limitrate
end

function togglePlay(fig, sld, btn, spdMenu)
    % Real-time playback on a timer (not a blocking loop): the simulated time follows the wall clock,
    % x1 / x2 / x3 as chosen in the speed menu. MATLAB stays free, so the mouse works during playback.
    stopPlayback(fig);
    if btn.Value
        S = fig.UserData;
        if sld.Value >= S.tg(end) - 0.2, sld.Value = S.tStart; end              % at the very end: restart from the start point
        tEnd = S.tPlayEnd; if sld.Value >= tEnd - 0.2, tEnd = S.tg(end); end    % past the window end: play on to the end of the data
        st = struct('tSim',sld.Value,'lastSet',sld.Value,'clk',tic,'speed',spdMenu.Value,'tEnd',tEnd);
        tm = timer('ExecutionMode','fixedSpacing','Period',0.03,'BusyMode','drop', ...
            'TimerFcn',@(~,~) playTick(fig,sld,btn,spdMenu));
        setappdata(fig,'play',struct('tm',tm,'st',st));
        btn.String = 'Pause'; start(tm);
    else
        btn.String = 'Play';
    end
end

function playTick(fig, sld, btn, spdMenu)
    if ~isvalid(fig), return, end
    p = getappdata(fig,'play'); if isempty(p), return, end
    st = p.st; tEnd = st.tEnd;
    if abs(sld.Value - st.lastSet) > 1e-9, st.tSim = sld.Value; st.clk = tic; end   % slider dragged: re-anchor
    if spdMenu.Value ~= st.speed                                                       % speed changed: re-anchor
        st.tSim = st.lastSet; st.clk = tic; st.speed = spdMenu.Value;
    end
    tNow = min(st.tSim + toc(st.clk)*st.speed, tEnd);
    sld.Value = tNow; st.lastSet = sld.Value;
    p.st = st; setappdata(fig,'play',p);
    showFrame(fig, tNow);
    if tNow >= tEnd, stopPlayback(fig); btn.Value = 0; btn.String = 'Play'; end
end

function stopPlayback(fig)
    p = getappdata(fig,'play');
    if ~isempty(p) && isvalid(p.tm), stop(p.tm); delete(p.tm); end
    setappdata(fig,'play',[]);
end

function viewDown(f)
    S = f.UserData; if isempty(S), return, end
    if isempty(axesUnderPointer(f,[S.ax S.ax2])), return, end
    setappdata(f,'rot',struct('p0',f.CurrentPoint,'v0',S.ax.View));
    f.WindowButtonMotionFcn = @(ff,~) viewMove(ff);
    f.WindowButtonUpFcn = @(ff,~) viewUp(ff);
end

function viewMove(f)
    r = getappdata(f,'rot'); if isempty(r), return, end
    S = f.UserData; d = f.CurrentPoint - r.p0;
    v = [r.v0(1) - 0.4*d(1), min(max(r.v0(2) - 0.4*d(2), -90), 90)];   % azimuth, elevation (deg)
    S.ax.View = v; S.ax2.View = v;                                      % both views, every mouse move
    drawnow limitrate
end

function viewUp(f)
    setappdata(f,'rot',[]); f.WindowButtonMotionFcn = ''; f.WindowButtonUpFcn = '';
end

function viewScroll(f, e)
    S = f.UserData; a = axesUnderPointer(f,[S.ax S.ax2]);
    if ~isempty(a), camzoom(a, 1.15^(-e.VerticalScrollCount)); end
end

function a = axesUnderPointer(f, axs)
    a = []; p = f.CurrentPoint;
    for i = 1:numel(axs)
        pos = getpixelposition(axs(i), true);
        if p(1) >= pos(1) && p(1) <= pos(1)+pos(3) && p(2) >= pos(2) && p(2) <= pos(2)+pos(4), a = axs(i); return, end
    end
end

function H = buildPFD(a)
    % Dark panel: attitude indicator (centre), speed tape (left, km/h), height tape (right, m).
    bg = [0.08 0.09 0.11];
    set(a,'XLim',[-2.35 2.35],'YLim',[-1.3 1.3],'Visible','off'); hold(a,'on'); daspect(a,[1 1 1]);
    patch('Parent',a,'XData',[-2.35 2.35 2.35 -2.35],'YData',[-1.3 -1.3 1.3 1.3],'FaceColor',bg,'EdgeColor','none');
    H.sc = 1/22;                                          % panel units per degree of pitch
    H.hz = hgtransform('Parent',a);                       % horizon + pitch ladder (moves with pitch and roll)
    big = 6;
    patch('Parent',H.hz,'XData',[-big big big -big],'YData',[0 0 big big],'FaceColor',[.20 .50 .85],'EdgeColor','none');
    patch('Parent',H.hz,'XData',[-big big big -big],'YData',[-big -big 0 0],'FaceColor',[.55 .36 .16],'EdgeColor','none');
    line('Parent',H.hz,'XData',[-big big],'YData',[0 0],'Color','w','LineWidth',1.5);
    for p = -20:2.5:20
        if p == 0, continue, end
        if mod(p,10)==0, len = 0.30; elseif mod(p,5)==0, len = 0.17; else, len = 0.08; end
        line('Parent',H.hz,'XData',[-len len],'YData',[p p]*H.sc,'Color','w','LineWidth',1);
        if mod(p,10)==0
            text('Parent',H.hz,'Position',[-len-0.04 p*H.sc],'String',sprintf('%d',abs(p)),'Color','w','FontSize',8,'HorizontalAlignment','right');
            text('Parent',H.hz,'Position',[ len+0.04 p*H.sc],'String',sprintf('%d',abs(p)),'Color','w','FontSize',8,'HorizontalAlignment','left');
        end
    end
    % mask everything outside the circle
    th = linspace(0,2*pi,181); Ri = 1; Ro = 3;
    X = [Ri*cos(th(1:end-1)); Ri*cos(th(2:end)); Ro*cos(th(2:end)); Ro*cos(th(1:end-1))];
    Y = [Ri*sin(th(1:end-1)); Ri*sin(th(2:end)); Ro*sin(th(2:end)); Ro*sin(th(1:end-1))];
    patch('Parent',a,'XData',X,'YData',Y,'FaceColor',bg,'EdgeColor','none');
    line('Parent',a,'XData',cos(th),'YData',sin(th),'Color',[.8 .8 .8],'LineWidth',1);
    % roll scale (rotates with the horizon) and fixed pointer
    H.hr = hgtransform('Parent',a);
    for b = [-60 -45 -30 -20 -10 0 10 20 30 45 60]
        ph = deg2rad(90 - b); len = 0.10 + 0.05*(mod(b,30)==0);
        line('Parent',H.hr,'XData',[1 1+len]*cos(ph),'YData',[1 1+len]*sin(ph),'Color','w','LineWidth',1.2);
    end
    patch('Parent',a,'XData',[-0.05 0.05 0],'YData',[1.17 1.17 1.03],'FaceColor',[1 .8 0],'EdgeColor','none');
    % fixed aircraft symbol
    for s = [-1 1]
        line('Parent',a,'XData',s*[0.55 0.20],'YData',[0 0],'Color',[1 .8 0],'LineWidth',3.5);
        line('Parent',a,'XData',s*[0.20 0.20],'YData',[0 -0.07],'Color',[1 .8 0],'LineWidth',3.5);
    end
    patch('Parent',a,'XData',0.03*[-1 1 1 -1],'YData',0.03*[-1 -1 1 1],'FaceColor',[1 .8 0],'EdgeColor','none');
    % tapes
    tapeBg = [0.15 0.16 0.19];
    patch('Parent',a,'XData',[-2.25 -1.25 -1.25 -2.25],'YData',[-0.95 -0.95 0.95 0.95],'FaceColor',tapeBg,'EdgeColor',[.5 .5 .5]);
    patch('Parent',a,'XData',[ 1.25  2.25  2.25  1.25],'YData',[-0.95 -0.95 0.95 0.95],'FaceColor',tapeBg,'EdgeColor',[.5 .5 .5]);
    H.spdLine = line('Parent',a,'XData',NaN,'YData',NaN,'Color','w');
    H.altLine = line('Parent',a,'XData',NaN,'YData',NaN,'Color','w');
    H.spdTxt = gobjects(1,9); H.altTxt = gobjects(1,9);
    for i = 1:9
        H.spdTxt(i) = text('Parent',a,'Position',[-1.40 0],'String','','Color','w','FontSize',9,'HorizontalAlignment','right','Visible','off');
        H.altTxt(i) = text('Parent',a,'Position',[ 1.42 0],'String','','Color','w','FontSize',9,'HorizontalAlignment','left','Visible','off');
    end
    patch('Parent',a,'XData',[-2.25 -1.25 -1.25 -2.25],'YData',[-0.13 -0.13 0.13 0.13],'FaceColor','k','EdgeColor',[1 .8 0]);
    patch('Parent',a,'XData',[ 1.25  2.25  2.25  1.25],'YData',[-0.13 -0.13 0.13 0.13],'FaceColor','k','EdgeColor',[1 .8 0]);
    H.spdBox = text('Parent',a,'Position',[-1.32 0],'String','0','Color',[1 1 1],'FontSize',14,'FontWeight','bold','HorizontalAlignment','right');
    H.altBox = text('Parent',a,'Position',[ 2.18 0],'String','0','Color',[1 1 1],'FontSize',14,'FontWeight','bold','HorizontalAlignment','right');
    text('Parent',a,'Position',[-1.75 1.10],'String','SPEED  km/h','Color',[.8 .8 .8],'FontSize',9,'HorizontalAlignment','center');
    text('Parent',a,'Position',[ 1.75 1.10],'String','HEIGHT  m','Color',[.8 .8 .8],'FontSize',9,'HorizontalAlignment','center');
    H.hdg = text('Parent',a,'Position',[0 -1.17],'String','','Color','w','FontSize',11,'HorizontalAlignment','center');
end

function updatePFD(H, pitch, roll, spd, alt, hdg)
    set(H.hz,'Matrix', makehgtform('zrotate',deg2rad(roll)) * makehgtform('translate',[0 -pitch*H.sc 0]));
    set(H.hr,'Matrix', makehgtform('zrotate',deg2rad(roll)));
    updateTape(H.spdLine, H.spdTxt, spd, 70, 10, 20, -1.25, -1, 0.95/70);
    updateTape(H.altLine, H.altTxt, alt, 60, 10, 20,  1.25, +1, 0.95/60);
    set(H.spdBox,'String',sprintf('%.0f',spd));
    set(H.altBox,'String',sprintf('%.0f',alt));
    set(H.hdg,'String',sprintf('HDG %03.0f\\circ    PITCH %+.1f\\circ    ROLL %+.1f\\circ',mod(hdg,360),pitch,roll));
end

function updateTape(ln, txt, val, range, tickStep, lblStep, xEdge, dir, sc)
    v = (ceil((val-range)/tickStep)*tickStep) : tickStep : (val+range);
    v = v(v >= 0);                                     % no negative speeds / heights on the tape
    y = (v - val)*sc;
    xs = [repmat(xEdge,1,numel(v)); repmat(xEdge+dir*0.12,1,numel(v)); nan(1,numel(v))];
    ys = [y; y; nan(1,numel(v))];
    set(ln,'XData',xs(:),'YData',ys(:));
    lbl = v(mod(v,lblStep)==0 & abs(y) < 0.85);
    for i = 1:numel(txt)
        if i <= numel(lbl)
            p = txt(i).Position; p(2) = (lbl(i)-val)*sc;
            set(txt(i),'Position',p,'String',sprintf('%.0f',lbl(i)),'Visible','on');
        else
            set(txt(i),'Visible','off');
        end
    end
end
