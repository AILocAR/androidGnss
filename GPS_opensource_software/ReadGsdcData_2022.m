function gnssGsdc = ReadGsdcData_2022(rawCsvFile,dataFilter)
% Read Gsdc 2022 (Google Smartphone Decimeter Challenge 2022) data to
% workspace
% Author: Xu Weng @ NTUsg



%factored into a few main sub-functions:
% ReadRawCsv()
% FilterData()
% PackGnssRaw()
% CheckGnssClock()
% ReportMissingFields()


[header,C] = ReadRawCsv(rawCsvFile);
[bOk,C] = FilterData(C,dataFilter,header);
if ~bOk, return, end

%% pack data into gnssRaw structure
[gnssGsdc,~] = PackGnssRaw(C,header);


end





function [header,C] = ReadRawCsv(GsdcCsvFile)
%% read data from csv file into a numerical matrix 'S' and cell array 'header'
%Note csvread fills ,, with zero, so we will need a lower level read function to
%tell the difference between empty fields and valid zeros
%T = readtable(csvFileName,'FileType','text'); %use this to debug


%read header row:
fid = fopen(GsdcCsvFile);
if fid<0
    error('file ''%s'' not found',GsdcCsvFile);
end
headerString = fgetl(fid);
if isempty(strfind(headerString,'TimeNanos'))
    error('\n"TimeNanos" string not found in file %s\n',fileName)
end

header=textscan(headerString,'%s','Delimiter',',');
header = header{1}; %this makes header a numFieldsx1 cell array
numFields = size(header,1);

%read lines using formatSpec so we get TimeNanos and FullBiasNanos as
%int64, everything else as doubles, and empty values as NaN
formatSpec='';
for i=1:numFields
    %lots of || here, because we are comparing a vector, 'header'
    %to a specific string each time. Not sure how to do this another way
    %and still be able to easily read and debug. Better safe than too clever.
    
    %longs
    if i == find(strcmp(header,'CodeType')) || ...
            i == find(strcmp(header,'SignalType'))||...
            i == find(strcmp(header,'MessageType'))
        formatSpec = formatSpec+" %s"; %append(formatSpec, ' %s');
    elseif i == find(strcmp(header,'TimeNanos')) || ...
            i == find(strcmp(header,'utcTimeMillis')) || ...
            i == find(strcmp(header,'FullBiasNanos')) || ...
            i == find(strcmp(header,'ReceivedSvTimeNanos')) || ...
            i == find(strcmp(header,'CarrierFrequencyHz')) || ...
            i == find(strcmp(header,'ChipsetElapsedRealtimeNanos')) || ...
            i == find(strcmp(header,'ArrivalTimeNanosSinceGpsEpoch'))|| ...
            i == find(strcmp(header,'ReceivedSvTimeNanosSinceGpsEpoch'))
        formatSpec = sprintf('%s %%d64',formatSpec); % formatSpec+ %d64
    elseif i == find(strcmp(header,'ConstellationType')) || ...
           i == find(strcmp(header,'Svid')) || ...
           i == find(strcmp(header,'MultipathIndicator')) || ...  
           i == find(strcmp(header,'AccumulatedDeltaRangeState')) || ...
           i == find(strcmp(header,'LeapSecond'))|| ...
           i == find(strcmp(header,'HardwareClockDiscontinuityCount'))|| ...
           i == find(strcmp(header,'TimeOffsetNanos'))||...
           i == find(strcmp(header,'State'))||...
           i == find(strcmp(header,'ReceivedSvTimeUncertaintyNanos'))||...
           i == find(strcmp(header,'AccumulatedDeltaRangeState'))
        formatSpec = sprintf('%s %%d32',formatSpec);
    else
        %everything else doubles
        formatSpec = sprintf('%s %%f',formatSpec);%在formatSpec后面不断追加
    end
end
%Replace empty string fields with 'NaN'
C = textscan(fid,formatSpec,'Delimiter',',','EmptyValue',NaN);

fclose(fid);
end

function [bOk,C] = FilterData(C,dataFilter,header)
%% filter C based on contents of dataFilter

bOk = true;
iS = ones(size(C{1})); %initialize index into rows of C
for i=1:size(dataFilter,1)
    j=find(strcmp(header,dataFilter{i,1}));%j = index into header
    %we should always be a value of j, because checkDataFilter checks for this:
    if ~any(j)
        continue;
    end
%     assert(any(j),'dataFilter{i} = %s not found in header\n',dataFilter{i,1})
    
    %now we must evaluate the expression in dataFilter{i,2}, for example:
    % 'BiasUncertaintyNanos < 1e7'
    %assign the relevant cell of C to a variable with same name as the header    
    ts = sprintf('%s = C{%d};',header{j},j);% header{j}=C{j}
    eval(ts);
    %create an index vector from the expression in dataFilter{i,2}
    ts = sprintf('iSi = %s;',dataFilter{i,2});% iSI = dataFilter{i,2}
    eval(ts);
    
    %AND the iS index values on each iteration of i
    iS = iS & iSi;
end
% Check if filter removes all values
if ~any(iS) %if all zeros
    fprintf('\nAll measurements removed. Specify dataFilter less strictly than this:, ')
    dataFilter(:,2)
    bOk=false;
    C=[];
    return
end

% Keep only those values of C indexed by iS
for i=1:length(C)
    C{i} = C{i}(iS);
end

%Replace CodeType string fields with 'NaN'
codeTypeHeaderIndex = find(strcmp(header,'CodeType'));
numberOfLogs = size(C{1},1);
C(codeTypeHeaderIndex) = {nan(numberOfLogs, 1)};

%Replace SignalType string fields with 'NaN'
signalTypeHeaderIndex = find(strcmp(header,'SignalType'));
numberOfLogs = size(C{1},1);
C(signalTypeHeaderIndex) = {nan(numberOfLogs, 1)};

%Replace MessageType string fields with 'NaN'
messageTypeHeaderIndex = find(strcmp(header,'MessageType'));
numberOfLogs = size(C{1},1);
C(messageTypeHeaderIndex) = {nan(numberOfLogs, 1)};

end %end of function FilterDataS



function [gsdcData,missing] = PackGnssRaw(C,header)
%% pack data into gnssRaw, and report missing fields
assert(length(C)==length(header),...
    'length(C) ~= length(header). This should have been checked before here')

gsdcData = [];
%report clock fields present/missing, based on:
gnssClockFields = {...
    'MeasTimeSinceStartNs'
    'TimeNanos'
    };
missing.ClockFields = {};

%report measurements fields present/missing, based on:
gnssMeasurementFields = {...
    'Cn0DbHz'
    'ConstellationType'  
    'CarrierFrequencyHz'
    'AzDeg'
    'ElDeg'
    'RawPrM'
    'RawPrUncM'
    'RawPrM'
    'RawPrErrorM'
    'SmPrM'
    'SmPrUncM'
    'SmPrErrorM'
    'PrrMps'
    'PrrUncMps'
    'PrrErrorMps'
    'AdrM'
    'AdrErrorM'
    'Svid'
    };
%leave these out for now, 'cause we dont care (for now), or they're deprecated,
% or they could legitimately be left out (because they are not computed in
% a particular GNSS implementation)
% SnrInDb, TimeOffsetNanos, CarrierFrequencyHz, CarrierCycles, CarrierPhase,
% CarrierPhaseUncertainty
missing.MeasurementFields = {};

%pack data into vector variables of the class 'gnssRaw', if the fields are not NaNs
% The first column is raw
for j = 2:length(header)
    if any(isfinite(C{j})) %not all NaNs
        %TBD what if there are some NaNs, but not all. i.e. some missing
        %data in the log file - TBD deal with this
        eval(['gsdcData.',header{j}, '=C{j};']);
    elseif any(strcmp(header{j},gnssClockFields))
        missing.ClockFields{end+1} = header{j};
    elseif any(strcmp(header{j},gnssMeasurementFields))
        missing.MeasurementFields{end+1} = header{j};
    end
end
%So, if a field is not reported, it will be all NaNs from makeCsv, and the above
%code will not load it into gnssRaw. So when we call 'CheckGnssClock' it can
%check for missing fields in gnssRaw.

%TBD look for all zeros that can not legitimately be all zero,
%e.g. AccumulatedDeltaRangeMeters, and report these as missing data

end %end of function PackGnssRaw



