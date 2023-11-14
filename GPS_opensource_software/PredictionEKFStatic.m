function [Xp, Pp] = PredictionEKFStatic(Xhat, Phat, gnssMeas, gpsPvt, iepoch, flag2)
% State X = [x, y, z, deltat, deltaf]
% Input:
%      Xhat --- Posterior estimation of state of last epoch
%      Pe --- Posterior covariance matrix
%      gnssMeas --- GNSS raw measurement
%      iepoch --- Index of current epoch
%      flag2 --- flag2 indicates if the current epoch is the second epoch
%      in warm-up process
% Output:
%      Xp --- Prior estimation of current state
%      Pp --- Prior covariance matrix

% For low speed movement, Full Time in Seconds is precise enough
% Author: Xu Weng
% if iepoch < 3
%     error('Kalman Filtering should start at the third epoch');
% end
Ts = gnssMeas.FctSeconds(iepoch) - gnssMeas.FctSeconds(iepoch - 1);

if flag2 == true
    Sf = ((gpsPvt.allBcDotMps(iepoch)-gpsPvt.allfcMpsEKF(iepoch-1))/Ts).^2;
    St = ((gpsPvt.allBcMeters(iepoch)-gpsPvt.allBcMetersEKF(iepoch-1))/Ts - gpsPvt.allBcDotMps(iepoch)).^2;
else
    Sf = ((gpsPvt.allfcMpsEKF(iepoch-1)-gpsPvt.allfcMpsEKF(iepoch-2))/Ts).^2;
    St = ((gpsPvt.allBcMetersEKF(iepoch-1)-gpsPvt.allBcMetersEKF(iepoch-2))/Ts - gpsPvt.allfcMpsEKF(iepoch-1)).^2;
end

% Covariance matrix of process noise
Q0 = zeros(3,2);
Qxyz = 0.005*eye(3);
Qt = [St*Ts+Sf*Ts^3/3, Sf*Ts^2/2; Sf*Ts^2/2, Sf*Ts];

Qe = [Qxyz, Q0;
      Q0',    Qt];

% State transition matrix 
a0 = zeros(3,2);
a = [1, Ts; 0, 1];
a1 = eye(3);
A = [a1, a0;
     a0', a];
  
Xp = A*Xhat';
Pp = A*Phat*A'+Qe;

end

% Copyright 2022 Nanyang Technological University
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