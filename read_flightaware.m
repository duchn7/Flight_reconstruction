function T = read_flightaware(mhtmlFile)
%READ_FLIGHTAWARE Parse a FlightAware "track log" page saved as .mhtml.
%   T = read_flightaware(file) returns a table with columns
%     Time (UTC datetime), Lat, Lon (deg), Course (deg), Kts, Alt (m), Rate (m/min)
%   FlightAware shows times in EST (UTC-5); the page also gives the UTC time of
%   some rows (e.g. "@ Monday 11:10:08 GMT"), which is used to work out the offset.

raw = fileread(mhtmlFile);                       % MHTML is 7-bit ASCII (quoted-printable)
a = strfind(raw,'<!DOCTYPE html>'); a = a(1);
b = strfind(raw(a:end),'------MultipartBoundary'); b = a + b(1) - 2;
q = raw(a:b);
q = regexprep(q,'=\r?\n','');                    % soft line breaks
q = strrep(q,'=3D','=');                         % the only escape that matters for parsing
q = regexprep(q,'=[0-9A-F]{2}','');              % drop other =XX escapes (arrows, nbsp ...)
q = regexprep(q,'<!--.*?-->','');                % drop HTML comments (commented-out cells)

% Date of the flight from the page's snapshot URL (.../history/YYYYMMDD/HHMMZ/...)
d = regexp(raw,'history/(\d{4})(\d{2})(\d{2})/','tokens','once');
day0 = datetime(str2double(d{1}),str2double(d{2}),str2double(d{3}));

rows = regexp(q,'<tr[^>]*>(.*?)</tr>','tokens');
tUTC = NaT(0,1); lat = []; lon = []; crs = []; kts = []; alt = []; rate = [];
utcRef = [];                                     % (EST-shown time, UTC time) pair to get offset
for i = 1:numel(rows)
    r = rows{i}{1};
    tds = regexp(r,'<td[^>]*>(.*?)</td>','tokens');
    % cell text: first <span> if there is one (full precision), else the cell text
    c = cellfun(@(x) cellText(x{1}), tds, 'UniformOutput',false);
    if isempty(utcRef) && numel(c) >= 4 && ~isempty(regexp(c{1},'^\w{3} \d\d:\d\d:\d\d$','once')) ...
            && ~isempty(regexp(r,'GMT','once')) && ~isempty(regexp(c{3},'^@ ','once'))
        g = regexp(c{3},'(\d\d):(\d\d):(\d\d)','tokens','once');
        l = regexp(c{1},'(\d\d):(\d\d):(\d\d)','tokens','once');
        if ~isempty(g) && ~isempty(l)
            utcRef = (str2double(g{1})*3600+str2double(g{2})*60+str2double(g{3})) - ...
                     (str2double(l{1})*3600+str2double(l{2})*60+str2double(l{3}));
        end
    end
    if numel(c) >= 8 && ~isempty(regexp(c{1},'^\w{3} \d\d:\d\d:\d\d$','once')) ...
            && ~isnan(str2double(c{2})) && ~isnan(str2double(c{3}))
        l = regexp(c{1},'(\d\d):(\d\d):(\d\d)','tokens','once');
        tUTC(end+1,1) = day0 + seconds(str2double(l{1})*3600+str2double(l{2})*60+str2double(l{3})); %#ok<AGROW>
        lat(end+1,1) = str2double(c{2}); lon(end+1,1) = str2double(c{3}); %#ok<AGROW>
        cc = regexp(c{4},'(\d+)','tokens','once'); crs(end+1,1) = str2double(cc{1}); %#ok<AGROW>
        kts(end+1,1) = str2double(c{5}); %#ok<AGROW>
        alt(end+1,1) = str2double(strrep(c{7},',','')); %#ok<AGROW>
        rt = regexprep(c{8},'[^\d,\.\-]',''); rate(end+1,1) = str2double(strrep(rt,',','')); %#ok<AGROW>
    end
end
if isempty(utcRef), utcRef = 5*3600; end         % FlightAware page times are EST = UTC-5
tUTC = tUTC + seconds(utcRef);
T = table(tUTC,lat,lon,crs,kts,alt,rate,'VariableNames',{'Time','Lat','Lon','Course','Kts','Alt','Rate'});
T = sortrows(T,'Time');
end

function s = cellText(h)
sp = regexp(h,'<span[^>]*>(.*?)</span>','tokens','once');
if ~isempty(sp), h = sp{1}; end
s = strtrim(regexprep(regexprep(h,'<[^>]+>',''),'&nbsp;',' '));
end
