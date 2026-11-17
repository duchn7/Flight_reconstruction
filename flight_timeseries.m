function flight_timeseries(titleStr, tAcc, acc, tAng, ang, tOri, ori, tFA, FA, groundAlt, tg, heading)
%FLIGHT_TIMESERIES Phone sensor time series next to the FlightAware track; greys out the part not animated.
figure('Name','Time series','Color','w','Position',[50 50 1100 900]);
tl = tiledlayout(6,1,'TileSpacing','compact');
title(tl, titleStr);
nexttile; plot(tAcc, acc{:,:}); ylabel('Accel (m/s^2)'); legend('X','Y','Z','Orientation','horizontal'); grid on
nexttile; plot(tAng, ang{:,:}); ylabel('Ang. vel (rad/s)'); grid on
nexttile; plot(tOri, ori{:,:}, '.', 'MarkerSize',4); ylabel('Phone orient. (deg)'); grid on
inFA = tFA >= min(tAcc)-30 & tFA <= max(tAcc)+30;
nexttile; plot(tFA(inFA), FA.Alt(inFA)-groundAlt, '-o','MarkerSize',3); ylabel('FA height (m)'); grid on
nexttile; plot(tFA(inFA), FA.Kts(inFA), '-o','MarkerSize',3); ylabel('FA speed (kt)'); grid on
nexttile; hold on; plot(tFA(inFA), FA.Course(inFA), 'o-','MarkerSize',3);
plot(tg, heading, '.', 'MarkerSize',3); ylabel('Heading (deg)'); xlabel('Time since phone recording start (s)');
legend('FlightAware course','Aircraft heading from phone','Location','best'); grid on
axs = findall(gcf,'Type','axes'); linkaxes(axs,'x'); xlim(axs(1),[0 max(tAcc)])
for a = axs'                                       % grey out data outside the animated range
    yl = ylim(a);
    patch(a,[0 tg(1) tg(1) 0],yl([1 1 2 2]),[.5 .5 .5],'FaceAlpha',.25,'EdgeColor','none','HandleVisibility','off');
    patch(a,[tg(end) max(tAcc) max(tAcc) tg(end)],yl([1 1 2 2]),[.5 .5 .5],'FaceAlpha',.25,'EdgeColor','none','HandleVisibility','off');
end
end
