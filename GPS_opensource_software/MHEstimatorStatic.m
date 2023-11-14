function [xHat_k,Sum_numSvs] = MHEstimatorStatic(CurrentEpoch,gnssMeas,allGpsEph,WindowSize,xo,weekNum,count1,iono)

xHat_k = []; % state updates at the kth epoch

% MHE based WLS Positioning for the i+L epoch

% Determine the current position of the MHE window
if count1 == 1
    % Stop MHE
    L = 0;
elseif 1< count1 && count1 <= WindowSize
    % Warm up MHE
    L = count1-1;
elseif count1 > WindowSize
    L = WindowSize;    
end

% Allocate space to the measurements in a window
prs_stack = cell([L+1,1]);

% Allocate space to the Ephemeris in a window
gpsEph_stack = cell([L+1,1]);

iValid0 = (isfinite(gnssMeas.PrM(CurrentEpoch-L:CurrentEpoch,:)));

Sum_numSvs = 0;

for j=1:L+1
    
    iValid = find(iValid0(j,:));
    
    svid = gnssMeas.Svid(iValid)';
    
    [gpsEph,iSv] = ClosestGpsEph(allGpsEph,svid,gnssMeas.FctSeconds(CurrentEpoch-L+j-1));
    
    gpsEph_stack(j) = {gpsEph};
    
    svid = svid(iSv); %svid for which we have ephemeris
    
    numSvs = length(svid); %number of satellites this epoch
    
    Sum_numSvs = Sum_numSvs+numSvs;% Total number of visible sv in the window
    
%     gpsPvt.numSvs(CurrentEpoch-L+j-1) = numSvs;
    
    %for those svIds with valid ephemeris, pack prs matrix for WlsNav
    prM = gnssMeas.PrM(CurrentEpoch-L+j-1,iValid(iSv))';
  
    prSigmaM= gnssMeas.PrSigmaM(CurrentEpoch-L+j-1,iValid(iSv))';
    prrMps  = gnssMeas.PrrMps(CurrentEpoch-L+j-1,iValid(iSv))';
    prrSigmaMps = gnssMeas.PrrSigmaMps(CurrentEpoch-L+j-1,iValid(iSv))';
    tRx = [ones(numSvs,1)*weekNum(CurrentEpoch-L+j-1),gnssMeas.tRxSeconds(CurrentEpoch-L+j-1,iValid(iSv))'];
    
    prs = [tRx, svid, prM, prSigmaM, prrMps, prrSigmaMps];
    prs_stack(j) = {prs};
end

if Sum_numSvs < 4
    return;%skip to next epoch(next window)
end

% MHE-WLS
[xHat_k,~,~,H,~] = WlsPvtMHE2ndOrderStatic(prs_stack,gpsEph_stack,xo,iono);
end