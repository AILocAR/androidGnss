function gpsPvt = GpsWlsPvt(gnssMeas,allGpsEph,iono,bRaw)
%gpsPvt = GpsWlsPvt(gnssMeas,allGpsEph,bRaw)
%compute PVT from gnssMeas
% Input: gnssMeas, structure of pseudoranges, etc. from ProcessGnssMeas
%        allGpsEph, structure with all ephemeris
%        [bRaw], default true, true => use raw pr, false => use smoothed
%
% Output: 
% gpsPvt.FctSeconds    Nx1 time vector, same as gnssMeas.FctSeconds
%       .allLlaDegDegM Nx3 matrix, (i,:) = [lat (deg), lon (deg), alt (m)]
%       .sigmaLlaM     Nx3 standard deviation of [lat,lon,alt] (m)
%       .allBcMeters   Nx1 common bias computed with llaDegDegM
%       .allVelMps     Nx3 (i,:) = velocity in NED coords
%       .sigmaVelMps   Nx3 standard deviation of velocity (m/s)
%       .allBcDotMps   Nx1 common freq bias computed with velocity
%       .numSvs        Nx1 number of satellites used in corresponding llaDegDegM
%       .hdop          Nx1 hdop of corresponding fix
%
%Algorithm: Weighted Least Squares

%Author: Frank van Diggelen
%Modified by Xu Weng
%Open Source code for processing Android GNSS Measurements

if nargin < 4 % Number of function input arguments
    bRaw = true;
else
    %check that smoothed pr fields exists in input
    if any(~isfield(gnssMeas,{'PrSmM','PrSmSigmaM'}))
       error('If bRaw is false, gnssMeas must have fields gnssMeas.PrSmM and gnssMeas.PrSmSigmaM')
    end
end

xo =zeros(8,1);%initial state: [center of the Earth, bc=0, velocities = 0]'
xo(1:3) = [-1509706.868, 6195107.314,149731.63]'; 
xo_MHE =zeros(8,1);%initial state for MHE: [center of the Earth, bc=0, velocities = 0]'
xo_MHE_k_N =zeros(8,1);%initial state at the k-Nth epoch for MHE: [center of the Earth, bc=0, velocities = 0]'

weekNum     = floor(gnssMeas.FctSeconds/GpsConstants.WEEKSEC);
%TBD check for week rollover here (it is checked in ProcessGnssMeas, but
%this function should stand alone, so we should check again, and adjust 
%tRxSeconds by +- a week if necessary)
%btw, Q. why not just carry around fct and not worry about the hassle of
%weeknumber, and the associated week rollover problems?
% A. because you cannot get better than 1000ns (1 microsecond) precsision
% when you put fct into a double. And that would cause errors of ~800m/s * 1us
% (satellite range rate * time error) ~ 1mm in the range residual computation
% So what? well, if you start processing with carrier phase, these errors
% could accumulate.

N = length(gnssMeas.FctSeconds);
gpsPvt.FctSeconds      = gnssMeas.FctSeconds;
gpsPvt.allXyzMMM       = zeros(N,3)+NaN;
gpsPvt.allLlaDegDegM   = zeros(N,3)+NaN; 
gpsPvt.sigmaLLaM       = zeros(N,3)+NaN;
gpsPvt.allBcMeters     = zeros(N,1)+NaN;
gpsPvt.allVelMps       = zeros(N,3)+NaN;
gpsPvt.allVelMpsXyz    = zeros(N,3)+NaN;
gpsPvt.sigmaVelMps     = zeros(N,3)+NaN;
gpsPvt.allBcDotMps     = zeros(N,1)+NaN;
gpsPvt.numSvs          = zeros(N,1);
gpsPvt.hdop            = zeros(N,1)+inf;
gpsPvt.SvPosR          = [];
gpsPvt.Wpr             = cell(N,1);
gpsPvt.dtsv            = zeros(size(gnssMeas.PrM))+NaN;
gpsPvt.RrMps           = zeros(size(gnssMeas.PrM))+NaN;

% Outputs of EKF
gpsPvt.allXhat         = zeros(N,8)+NaN;
gpsPvt.allPhat         = zeros(8,8,N)+NaN;
gpsPvt.allXyzMMMEKF    = zeros(N,3)+NaN;
gpsPvt.allLlaDegDegMEKF= zeros(N,3)+NaN;
gpsPvt.allBcMetersEKF  = zeros(N,1)+NaN;
gpsPvt.allfcMpsEKF     = zeros(N,1)+NaN;
gpsPvt.allVelMpsEKF    = zeros(N,3)+NaN;

% Outputs of MHE
gpsPvt.allXyzMMMMHE    = zeros(N,3)+NaN;
gpsPvt.allLlaDegDegMMHE= zeros(N,3)+NaN;
gpsPvt.allBcMetersMHE  = zeros(N,1)+NaN;
gpsPvt.allBcDotMpsMHE  = zeros(N,1)+NaN;
gpsPvt.allVelMpsMHE    = zeros(N,3)+NaN;

GT_data = load('DenoisedPrM.csv');
% GT_data = load('SvPVT3D_Error_label_static_data.csv');


f_3DPVT = fopen('PVT3D.txt','w');

% offset = 17;
offset = 1;
% N = 612;

for i=offset:N    
    iValid = find(isfinite(gnssMeas.PrM(i,:)) & gnssMeas.Cn0DbHz(i,:) > 0); %index into valid svid
    svid    = gnssMeas.Svid(iValid)';
    
    [gpsEph,iSv] = ClosestGpsEph(allGpsEph,svid,gnssMeas.FctSeconds(i));% full cycle time
%     [gpsEph,iSv] = ClosestBdsEph(allGpsEph,svid,gnssMeas.FctSeconds(i));
    svid = svid(iSv); %svid for which we have ephemeris
    numSvs = length(svid); %number of satellites this epoch
    gpsPvt.numSvs(i) = numSvs;
    
    if numSvs < 4     
        A = [i,gnssMeas.FctSeconds(i),0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
        gpsPvt.SvPosR = [gpsPvt.SvPosR; A];
        gpsPvt.Wpr(i) = {0};
        
        % Update states of EKF
        gpsPvt.allXhat(i,:)         = gpsPvt.allXhat(i-1,:);
        gpsPvt.allPhat(:,:,i)       = gpsPvt.allPhat(:,:,i-1);
        gpsPvt.allXyzMMMEKF(i,:)    = gpsPvt.allXyzMMMEKF(i-1,:);
        gpsPvt.allLlaDegDegMEKF(i,:)= gpsPvt.allLlaDegDegMEKF(i-1,:);
        gpsPvt.allBcMetersEKF(i,:)  = gpsPvt.allBcMetersEKF(i-1,:);
        gpsPvt.allfcMpsEKF(i,:)     = gpsPvt.allfcMpsEKF(i-1,:);
        gpsPvt.allVelMpsEKF(i,:)    = gpsPvt.allVelMpsEKF(i-1,:);
        
        continue;%skip to next epoch
    end
    
    %% WLS PVT -----------------------------------------------------------------
    %for those svIds with valid ephemeris, pack prs matrix for WlsNav
    prM     = gnssMeas.PrM(i,iValid(iSv))';
    % Feed back the groundtruth pseudoranges without PrM error and user
    % clock bias
    index_GT = GT_data(:,2) == i;
%     prM = GT_data(index_GT,end);
    prSigmaM= gnssMeas.PrSigmaM(i,iValid(iSv))';
    
    prrMps  = gnssMeas.PrrMps(i,iValid(iSv))';
    prrSigmaMps = gnssMeas.PrrSigmaMps(i,iValid(iSv))';
     
    tRx = [ones(numSvs,1)*weekNum(i),gnssMeas.tRxSeconds(i,iValid(iSv))'];
    
    delta_t =1;
    if i>1
        delta_t_vector = gnssMeas.tRxSeconds(i,iValid(iSv))-gnssMeas.tRxSeconds(i-1,iValid(iSv));
        if ~isempty(delta_t_vector) && delta_t_vector(1) > 0
            delta_t = delta_t_vector(1);
        end          
    end
    
    prs = [tRx, svid, prM, prSigmaM, prrMps, prrSigmaMps];
       
    xo(5:8) = zeros(4,1); %initialize speed to zero
    
    
    %% WLS
    [xHat,~,svPos,H,Wpr,Wrr,R_Ac,I_iono_logger,I_trop_logger] = WlsPvt(prs,gpsEph,xo,iono,delta_t);%compute WLS solution
    xo = xo + xHat;    
                   
    %% Calculate the elevation and azimuth angle
    %Calculate line of sight vectors from user to the satellite
    v = zeros(length(gnssMeas.Svid),3)+NaN;
    v(iValid(iSv),:) = svPos(:,2:4) - xo(1:3)';
    
    %Calculate the geodetic latitude and longitude of the user
    llaDegDegM = Xyz2Lla(xo(1:3)');
    
    %Calculate the rotation matrix to convert an ECEF vector to
    % North, East, Down coordinates, and vice-versa
    RE2N = RotEcef2Ned(llaDegDegM(1),llaDegDegM(2));
    
    %Calculate line of sight vectors from user to the satellite in Ned
    %coordinate system
    vNed = RE2N*v';
   
    % Calculate Elevation angle and azimuth angle
    Ele = asin(-vNed(3,:)./sqrt( sum(vNed.^2) )); % rad
    Azi = atan2(vNed(2,:), vNed(1,:)); % rad
    
    %% Extract CN0 from gnssMeas
    CN0Sv = gnssMeas.Cn0DbHz(i,iValid(iSv));
    
    %% State update
    % the pseudorange corrected by satellite clock biases
    % R = prs(:,4)+GpsConstants.LIGHTSPEED*svPos(:,5);  
    R = R_Ac + GpsConstants.LIGHTSPEED*svPos(:,5); % R is the pseduorange excluding satellite clock bias, atmospherical delays
%     R = prM + GpsConstants.LIGHTSPEED*svPos(:,5); % Pseudorange measurements without AC correction
    
    gpsPvt.dtsv(i,iValid(iSv)) = svPos(:,5)'; 
    gpsPvt.dtsvDot(i,iValid(iSv)) = svPos(:,6)'; 
    gpsPvt.RrMps(i,iValid(iSv)) = svPos(:,7)'; 
    
     %% Extended Kalman Filtering
    if i - (offset-1) < 3 % Stop Kalman Filter 
       Xhat = [xo(1),xo(5),xo(2),xo(6),xo(3),xo(7),xo(4),xo(8)];
       Phat = zeros(8,8); 
    elseif i - (offset-1) == 3 % Warm Up Kalman Filter
       Xhat = [gpsPvt.allXyzMMM(i-1, 1),gpsPvt.allVelMpsXyz(i-1, 1),gpsPvt.allXyzMMM(i-1, 2),gpsPvt.allVelMpsXyz(i-1, 2),gpsPvt.allXyzMMM(i-1, 3),gpsPvt.allVelMpsXyz(i-1, 3),gpsPvt.allBcMeters(i-1),gpsPvt.allBcDotMps(i-1)];
       Phat = zeros(8,8);
       [Xp, Pp] = PredictionEKF0(Xhat, Phat, gnssMeas, gpsPvt, i);
       [Xhat,Phat] = AdjustmentEKF(Xp, Pp, prs,gpsEph,iono);
    elseif i - (offset-1)>3 % Start Kalman Filter
       Xhat = gpsPvt.allXhat(i-1,:);
       Phat = gpsPvt.allPhat(:,:,i-1);
       [Xp, Pp] = PredictionEKF0(Xhat, Phat, gnssMeas, gpsPvt, i); 
       [Xhat,Phat] = AdjustmentEKF(Xp, Pp, prs,gpsEph,iono); 
    end
    
    
    %% Moving horizon estimator
    % The window size of MHE, the number of historical data excluding the current epoch
    WindowSize = 8;
    % xo_MHE(5:7) = zeros(3,1); %initialize speed to zero
    [xHat_MHE,Sum_numSvs,xHat_k_N] = MHEstimator(i,gnssMeas,allGpsEph,WindowSize,xo_MHE_k_N,weekNum,GT_data);
    
    if Sum_numSvs<4
        continue;%skip to next epoch
    end
    
    xo_MHE = xo_MHE + xHat_MHE;
    xo_MHE_k_N = xo_MHE_k_N + xHat_k_N;
       
    %% Pack gpsPvt
    gpsPvt.allXhat(i,:) = Xhat;
    gpsPvt.allPhat(:,:,i) = Phat;
    
    gpsPvt.elevation(i,:) = Ele;
    gpsPvt.azimuth(i,:) = Azi;
        
    %extract position states
%     llaDegDegM = Xyz2Lla(xo(1:3)');
    
    % Convert EKF positioning results from ECEF to lla
    llaDegDegMEKF = Xyz2Lla([Xhat(1),Xhat(3),Xhat(5)]);
    
    % Convert MHE positioning results from ECEF to lla
    llaDegDegMMHE = Xyz2Lla(xo_MHE(1:3)'); 
    
    % Pack ECEF positioning results of WLS, EKF and MHE
    gpsPvt.allXyzMMM(i,:) = xo(1:3);
    gpsPvt.allXyzMMMEKF(i,:) = [Xhat(1),Xhat(3),Xhat(5)];
    gpsPvt.allXyzMMMMHE(i,:) = xo_MHE(1:3);
    
    % Compute Pseudorange based on EKF positioning results
    PrMEkf = sqrt(sum((gpsPvt.allXyzMMMEKF(i,:) - svPos(:,2:4))'.^2))';
    A = [i*ones(length(R),1),gnssMeas.FctSeconds(i)*ones(length(R),1),svPos(:,1:5),Ele(iValid(iSv))',Azi(iValid(iSv))',CN0Sv',R,I_iono_logger,I_trop_logger,prM,prSigmaM,prrMps,PrMEkf];
    % Index of time, time, PRN, SvX, SvY, SvZ, dtsv, Elevation, Azimuth, CN0, Pseudorange after corrections, Iono_delay, Trop_delay, Raw Pseudoranges 
    fprintf(f_3DPVT,'%d \t %f \t %d \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f \t %f\n',A');
    gpsPvt.SvPosR = [gpsPvt.SvPosR; A];
        
    % Pack lla positioning results of WLS, EKF and MHE
    gpsPvt.allLlaDegDegM(i,:) = llaDegDegM;
    gpsPvt.allLlaDegDegMEKF(i,:) = llaDegDegMEKF;
    gpsPvt.allLlaDegDegMMHE(i,:) = llaDegDegMMHE;
       
    % Pack timing results of WLS, EKF and MHE
    gpsPvt.allBcMeters(i) = xo(4);
    gpsPvt.allBcMetersEKF(i) = Xhat(7);
    gpsPvt.allfcMpsEKF(i) = Xhat(8);
    gpsPvt.allBcMetersMHE(i) = xo_MHE(4);
    gpsPvt.allBcDotMpsMHE(i) = xo_MHE(8);
    
    %extract velocity states
    RE2N = RotEcef2Ned(llaDegDegM(1),llaDegDegM(2));
    %NOTE: in real-time code compute RE2N once until position changes
    vNed = RE2N*xo(5:7); %velocity in NED
    gpsPvt.allVelMps(i,:) = vNed;
    gpsPvt.allVelMpsXyz(i,:) = xo(5:7);%velocity in xyz
    gpsPvt.allBcDotMps(i) = xo(8);
    
    gpsPvt.allVelMpsEKF(i,:) = [Xhat(2),Xhat(4),Xhat(6)];%velocity in xyz
    
    %extract velocity states of MHE
    RE2NMHE = RotEcef2Ned(llaDegDegMMHE(1),llaDegDegMMHE(2));
    %NOTE: in real-time code compute RE2N once until position changes
    vNedMHE = RE2NMHE*xo_MHE(5:7); %velocity in NED
    gpsPvt.allVelMpsMHE(i,:) = vNedMHE;
    
    % extract weights in WLS
    gpsPvt.Wpr(i) = {Wpr};
    
    %compute HDOP
    H = [H(:,1:3)*RE2N', ones(numSvs,1)]; %observation matrix in NED
    P = inv(H'*H);%unweighted covariance
    gpsPvt.hdop(i) = sqrt(P(1,1)+P(2,2));
    
    %compute variance of llaDegDegM
    %inside LsPvt the weights are used like this:
    %  z = Hx, premultiply by W: Wz = WHx, and solve for x:
    %  x = pinv(Wpr*H)*Wpr*zPr;
    %  the point of the weights is to make sigma(Wz) = 1
    %  therefore, the variances of x come from  diag(inv(H'Wpr'WprH))
    P = inv(H'*(Wpr'*Wpr)*H); %weighted covariance
    gpsPvt.sigmaLLaM(i,:) = sqrt(diag(P(1:3,1:3)));
    
    %similarly, compute variance of velocity
    P = inv(H'*(Wrr'*Wrr)*H); %weighted covariance
    gpsPvt.sigmaVelMps(i,:) = sqrt(diag(P(1:3,1:3)));
    %%end WLS PVT --------------------------------------------------------------
end
fclose(f_3DPVT);
end %end of function GpsWlsPvt
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Copyright 2016 Google Inc.
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

