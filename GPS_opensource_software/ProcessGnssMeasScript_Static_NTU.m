clc;
clear;
%ProcessGnssMeasScript_Static_NTU.m, script to read GnssLogger output, compute and plot:
% pseudoranges, C/No, and weighted least squares PVT solution
%
% you can run the data in the NTU data files provided for you: 

prFileName = 'HuaweiMate10Pro_GnssLog.txt';

%Groundtruth file name
gtcsvFileName = 'NTU-ublox-GT.csv';

% as follows
% 1) copy everything from GitHub androidGNSS to 
%    a local directory on your machine
% 2) change 'dirName = ...' to match the local directory you are using:
dirName = '../data/HuaweiMate10Pro/Training_data/2022-10-09-NTU-ADM-4';
% 3) run ProcessGnssMeasScript.m script file 
param.llaTrueDegDegM = [];

indexlist = strfind(dirName, '/');
index = indexlist(end);
filename_date = dirName(index+1:end);

%Author: Aaron Xu Weng, Frank van Diggelen
%These codes are based on the original open source code for processing Android GNSS Measurements
%The original codes are available at https://github.com/google/gps-measurement-tools

%% parameters
%param.llaTrueDegDegM = [];
%enter true WGS84 lla, if you know it:
param.llaTrueDegDegM = [37.422578, -122.081678, -28];%Charleston Park Test Site

%% Set the data filter and Read log file
dataFilter = SetDataFilter; 
% FullBiasNanos                 FullBiasNanos ~= 0; 
% ConstellationType             ConstellationType==1 (GPS);
% State                         bitand(State,2^0) & bitand(State,2^3)

%% Read Groundtruth data
GroundTruth = load([dirName,'/', gtcsvFileName]);

%% Read Android Raw GNSS Measurements
[gnssRaw,gnssAnalysis] = ReadGnssLogger(dirName,prFileName,dataFilter);
if isempty(gnssRaw), return, end

%% Get online ephemeris from Nasa ftp, first compute UTC Time from gnssRaw:
fctSeconds = 1e-3*double(gnssRaw.allRxMillis(end)); %Full cycle time in seconds
utcTime = Gps2Utc([],fctSeconds);
[allGpsEph, iono] = GetNasaHourlyEphemeris(utcTime,dirName);
if isempty(allGpsEph), return, end

%% process raw measurements, compute pseudoranges:
[gnssMeas] = ProcessGnssMeasStatic(gnssRaw);

%% plot pseudoranges and pseudorange rates
h1 = figure;
[colors] = PlotPseudoranges(gnssMeas,prFileName);
h2 = figure;
PlotPseudorangeRates(gnssMeas,prFileName,colors);
h3 = figure;
PlotCno(gnssMeas,prFileName,colors);

%% Detect Smartphone discontinuity
% Detect Smartphone clock discontinuity
Threshold_discontinuity = 10;
Flag_discon1 = [0;diff(gnssMeas.FctSeconds)] > Threshold_discontinuity;

% Detect Smartphone PrM discontinuity
Flag_discon2 = sum(abs([zeros(1,size(gnssMeas.PrM, 2));diff(gnssMeas.PrM)]) > 5e4, 2) > 0;

% Reported Smartphone clock discontinuouty
Flag_discon3 = [0;diff(gnssMeas.ClkDCount)~=0];%binary, 1 <=> clock discontinuity

% Combine 2 discontinuity
Flag_discon = Flag_discon1 | Flag_discon2 | Flag_discon3;

%% compute WLS position and velocity
gpsPvt = GpsWlsPvtEKFStatic(gnssMeas,allGpsEph,iono,Flag_discon);
f_3D_WLS_PVT = fopen('WLS_PVT_3D.csv','w');
fprintf(f_3D_WLS_PVT,'%f , %f , %f \n',gpsPvt.allLlaDegDegM');
fclose(f_3D_WLS_PVT);

%% NTU
figure;
plot(gpsPvt.allLlaDegDegM(:,2),gpsPvt.allLlaDegDegM(:,1),'m','linewidth',2);hold on;
plot(gpsPvt.allLlaDegDegMEKF(:,2),gpsPvt.allLlaDegDegMEKF(:,1),'c','linewidth',2);hold on;
legend("WLS Results","EKF");
axis tight;

% ADM
axis([103.68308, 103.685,1.3487, 1.35012]);
hold on;
I = imread([dirName '/ADM.png']); 
h = image('XData',[103.68308, 103.685],'YData',[1.35012,1.3487],'CData',I);%note the latitude (y-axis) is flipped in vertical direction
uistack(h,'bottom'); %move the image to the bottom of current stack

% % EEE-S2
% axis([103.67965, 103.68066,1.34134,1.34231]);
% hold on;
% I = imread([dirName '/S2_roof.png']); 
% h = image('XData',[103.67965, 103.68066],'YData',[1.34231,1.34134],'CData',I);%note the latitude (y-axis) is flipped in vertical direction
% uistack(h,'bottom'); %move the image to the bottom of current stack

%% Comparison with ground truth receiver
[GT_row, GT_clo] = size(GroundTruth);
[tRx_row, tRx_clo] = size(gnssMeas.tRxSeconds);
tRx_i = 1;
tRx_j = 1;

% The groundtruth is later than Android raw measurements
if GroundTruth(1,3) - gnssMeas.tRxSeconds(tRx_i, find(~isnan(gnssMeas.tRxSeconds(tRx_i, :)), 1)) > 0
    for i = tRx_i:tRx_row
        if abs(GroundTruth(1,3)-gnssMeas.tRxSeconds(i, find(~isnan(gnssMeas.tRxSeconds(i, :)), 1))) <= 0.5
            % Align the first epoch
            break;
        end
    end
    tRx_i = i;
    % The ground truth file is 1 Hz
    GroundTruth_cache = GroundTruth(1:GT_row,:);  
    
% The groundtruth is earlier than Android raw measurements
elseif GroundTruth(1,3) - gnssMeas.tRxSeconds(tRx_i, find(~isnan(gnssMeas.tRxSeconds(tRx_i, :)), 1)) <= 0
    for i = 1:GT_row
        if abs(GroundTruth(i,3)-gnssMeas.tRxSeconds(tRx_i, find(~isnan(gnssMeas.tRxSeconds(tRx_i, :)), 1))) <= 0.5
            % Align the first epoch
            break;
        end
    end
    % The ground truth file is 1 Hz
    GroundTruth_cache = GroundTruth(i:GT_row,:);  
end

% Ground truth should not be longer than raw measurements  
[GT_row_cache, GT_clo_cache] = size(GroundTruth_cache);
GT_index = zeros(GT_row_cache,1);
j = 0;
% Find the corresponding ground truth
% The loop of measurements
for i = tRx_i:tRx_row    
    % The loop of ground truth
    for j = 1: GT_row_cache
        if abs(GroundTruth_cache(j,3)-gnssMeas.tRxSeconds(i, find(~isnan(gnssMeas.tRxSeconds(i, :)), 1))) <= 0.5
            GT_index(j) = 1;
            break
        end
    end
end
GroundTruth0 = GroundTruth_cache(logical(GT_index),:);
[GT_row0, GT_clo0] = size(GroundTruth0);


%% Compare WLS, EKF, MHE, RTS and Groundtruth
figure1 = figure;
% WLS
plot(gpsPvt.allLlaDegDegM(tRx_i:tRx_row-2,2),gpsPvt.allLlaDegDegM(tRx_i:tRx_row-2,1),'m','linewidth',2);hold on;
% MHE
plot(gpsPvt.allLlaDegDegMMHE(tRx_i:tRx_row-2,2),gpsPvt.allLlaDegDegMMHE(tRx_i:tRx_row-2,1),'r','linewidth',2);hold on;
% EKF
plot(gpsPvt.allLlaDegDegMEKF(tRx_i:tRx_row-2,2),gpsPvt.allLlaDegDegMEKF(tRx_i:tRx_row-2,1),'b','linewidth',2);hold on;
% RTS
plot(gpsPvt.allLlaDegDegMEKFS(tRx_i:tRx_row-2,2),gpsPvt.allLlaDegDegMEKFS(tRx_i:tRx_row-2,1),'c','linewidth',2);hold on;

% Groundtruth
plot(GroundTruth0(1:GT_row0,4),GroundTruth0(1:GT_row0,5),'color','#008744','linewidth',2);hold on;
% legend("WLS","EKF",'MHE',"Google's GNSS Analysis Software Result","U-blox Receiver Ground Truth");
legend("WLS",'MHE',"EKF","RTS smoother","Ground Truth",'FontSize',40,'FontName','Times New Roman');
% legend("WLS","EKF","Google's GNSS Analysis Software Result","U-blox Receiver Ground Truth");
xlabel("Longitude",'FontSize',40,'FontName','Times New Roman');
ylabel("Latitude",'FontSize',40,'FontName','Times New Roman');
axis tight;

% ADM
axis([103.68308, 103.685,1.3487, 1.35012]);
figureh.XAxis.Visible = 'off';
figureh.YAxis.Visible = 'off';
hold on;
I = imread([dirName '/ADM.png']); 
h = image('XData',[103.68308, 103.685],'YData',[1.35012,1.3487],'CData',I);%note the latitude (y-axis) is flipped in vertical direction
uistack(h,'bottom'); %move the image to the bottom of current stack

% % EEE
% axis([103.67965, 103.68066,1.34134,1.34231]);
% figureh = gca;
% figureh.XAxis.Visible = 'off';
% figureh.YAxis.Visible = 'off';
% hold on;
% I = imread([dirName '/S2_roof.png']);
% h = image('XData',[103.67965, 103.68066],'YData',[1.34231,1.34134],'CData',I);%note the latitude (y-axis) is flipped in vertical direction
% uistack(h,'bottom'); %move the image to the bottom of current stack

saveas(figure1,[dirName,'/Results.fig']);


%% Calculate the positioning errors

for n = tRx_i:tRx_row
   % WLS
    GroundTruthXyz = Lla2Xyz([GroundTruth0(n-tRx_i+1,5), GroundTruth0(n-tRx_i+1,4), GroundTruth0(n-tRx_i+1,6)]);
    delta_x(n,1) = gpsPvt.allXyzMMM(n,1)-GroundTruthXyz(1); % Delta X
    delta_y(n,1) = gpsPvt.allXyzMMM(n,2)-GroundTruthXyz(2); % Delta Y
    delta_z(n,1) = gpsPvt.allXyzMMM(n,3)-GroundTruthXyz(3); % Delta Z
    
    delta_xEKF(n,1) = gpsPvt.allXyzMMMEKF(n,1)-GroundTruthXyz(1); % Delta X
    delta_yEKF(n,1) = gpsPvt.allXyzMMMEKF(n,2)-GroundTruthXyz(2); % Delta Y
    delta_zEKF(n,1) = gpsPvt.allXyzMMMEKF(n,3)-GroundTruthXyz(3); % Delta Z
    
    delta_xEKFS(n,1) = gpsPvt.allXyzMMMEKFS(n,1)-GroundTruthXyz(1); % Delta X
    delta_yEKFS(n,1) = gpsPvt.allXyzMMMEKFS(n,2)-GroundTruthXyz(2); % Delta Y
    delta_zEKFS(n,1) = gpsPvt.allXyzMMMEKFS(n,3)-GroundTruthXyz(3); % Delta Z
    
    delta_xMHE(n,1) = gpsPvt.allXyzMMMMHE(n,1)-GroundTruthXyz(1); % Delta X
    delta_yMHE(n,1) = gpsPvt.allXyzMMMMHE(n,2)-GroundTruthXyz(2); % Delta Y
    delta_zMHE(n,1) = gpsPvt.allXyzMMMMHE(n,3)-GroundTruthXyz(3); % Delta Z
    
    % NED System
    %Calculate the rotation matrix to convert an ECEF vector to
    % North, East, Down coordinates, and vice-versa   
    RE2N = RotEcef2Ned(GroundTruth0(n-tRx_i+1,5), GroundTruth0(n-tRx_i+1,4));
    delta_v_ned = RE2N*[delta_x(n,1);delta_y(n,1);delta_z(n,1)];
    delta_n(n,1) = delta_v_ned(1);
    delta_e(n,1) = delta_v_ned(2);
    delta_d(n,1) = delta_v_ned(3);
    
    delta_v_ned_EKF = RE2N*[delta_xEKF(n,1);delta_yEKF(n,1);delta_zEKF(n,1)];
    delta_nEKF(n,1) = delta_v_ned_EKF(1);
    delta_eEKF(n,1) = delta_v_ned_EKF(2);
    delta_dEKF(n,1) = delta_v_ned_EKF(3);
    
    delta_v_ned_EKFS = RE2N*[delta_xEKFS(n,1);delta_yEKFS(n,1);delta_zEKFS(n,1)];
    delta_nEKFS(n,1) = delta_v_ned_EKFS(1);
    delta_eEKFS(n,1) = delta_v_ned_EKFS(2);
    delta_dEKFS(n,1) = delta_v_ned_EKFS(3);
    
    delta_v_ned_MHE = RE2N*[delta_xMHE(n,1);delta_yMHE(n,1);delta_zMHE(n,1)];
    delta_nMHE(n,1) = delta_v_ned_MHE(1);
    delta_eMHE(n,1) = delta_v_ned_MHE(2);
    delta_dMHE(n,1) = delta_v_ned_MHE(3);
    
    % Geodetic System
    GroundTruth = [GroundTruth0(n-tRx_i+1,5), GroundTruth0(n-tRx_i+1,4), GroundTruth0(n-tRx_i+1,6)];
    delta_lat(n,1) = gpsPvt.allLlaDegDegM(n,1)-GroundTruth(1); % Delta latitude
    delta_lon(n,1) = gpsPvt.allLlaDegDegM(n,2)-GroundTruth(2); % Delta longitude
    delta_alt(n,1) = gpsPvt.allLlaDegDegM(n,3)-GroundTruth(3); % Delta altitude
    
    delta_lat_EKF(n,1) = gpsPvt.allLlaDegDegMEKF(n,1)-GroundTruth(1); % Delta latitude
    delta_lon_EKF(n,1) = gpsPvt.allLlaDegDegMEKF(n,2)-GroundTruth(2); % Delta longitude
    delta_alt_EKF(n,1) = gpsPvt.allLlaDegDegMEKF(n,3)-GroundTruth(3); % Delta altitude
    
    delta_lat_EKFS(n,1) = gpsPvt.allLlaDegDegMEKFS(n,1)-GroundTruth(1); % Delta latitude
    delta_lon_EKFS(n,1) = gpsPvt.allLlaDegDegMEKFS(n,2)-GroundTruth(2); % Delta longitude
    delta_alt_EKFS(n,1) = gpsPvt.allLlaDegDegMEKFS(n,3)-GroundTruth(3); % Delta altitude
    
    delta_lat_MHE(n,1) = gpsPvt.allLlaDegDegMMHE(n,1)-GroundTruth(1); % Delta latitude
    delta_lon_MHE(n,1) = gpsPvt.allLlaDegDegMMHE(n,2)-GroundTruth(2); % Delta longitude
    delta_alt_MHE(n,1) = gpsPvt.allLlaDegDegMMHE(n,3)-GroundTruth(3); % Delta altitude
    
    
    
end

figure;
subplot(3,1,1);
plot(delta_x,'m','linewidth',2);hold on;
plot(delta_xMHE,'r','linewidth',2); hold on;
plot(delta_xEKF,'b','linewidth',2);hold on;
plot(delta_xEKFS,'c','linewidth',2);
% axis([0, 1800, -100, 50]);
save('error_xWLS.mat','delta_x');
save('error_xEKF.mat','delta_xEKF');
save('error_xMHE.mat','delta_xMHE');
% save('error_xWLS_ND.mat','delta_x');
% save('error_xEKF_ND.mat','delta_xEKF');
% save('error_xMHE_ND.mat','delta_xMHE');
xlabel("Epoch (s)",'linewidth',2,'FontSize',40);
ylabel("Error on X axis (m)",'linewidth',2,'FontSize',40);
legend("WLS","MHE","EKF","RTS Smoother",'FontSize',40);
% Mean_delta_lat_m = mean(delta_lat_m(40:130));
% Mean_delta_lat_m = mean(delta_lat_m,'omitnan');
% RMSE_lat_m = rms(delta_lat_m,'omitnan');
subplot(3,1,2);
plot(delta_y,'m','linewidth',2);hold on
plot(delta_yEKF,'b','linewidth',2);hold on;
plot(delta_yMHE,'r','linewidth',2);hold on;
plot(delta_yEKFS,'c','linewidth',2);
% axis([0, 1800, -133, 50]);
save('error_yWLS.mat','delta_y');
save('error_yEKF.mat','delta_yEKF');
save('error_yMHE.mat','delta_yMHE');
% save('error_yWLS_ND.mat','delta_y');
% save('error_yEKF_ND.mat','delta_yEKF');
% save('error_yMHE_ND.mat','delta_yMHE');
xlabel("Epoch (s)",'linewidth',2);
ylabel("Error on Y axis  (m)",'linewidth',2);

subplot(3,1,3);
plot(delta_z,'m','linewidth',2);hold on
plot(delta_zEKF,'b','linewidth',2);hold on;
plot(delta_zMHE,'r','linewidth',2);hold on;
plot(delta_zEKFS,'c','linewidth',2);
% axis([0, 1800, -40, 148]);
save('error_zWLS.mat','delta_z');
save('error_zEKF.mat','delta_zEKF');
save('error_zMHE.mat','delta_zMHE');
% save('error_zWLS_ND.mat','delta_z');
% save('error_zEKF_ND.mat','delta_zEKF');
% save('error_zMHE_ND.mat','delta_zMHE');
xlabel("Epoch (s)",'linewidth',2);
ylabel("Error on Z axis  (m)",'linewidth',2);
% Mean_delta_long_m = mean(delta_long_m(40:130));
% Mean_delta_long_m = mean(delta_long_m,'omitnan');
% RMSE_long_m = rms(delta_long_m,'omitnan');
% figure;
% subplot(2,1,1);
% plot(delta_lat,'g','linewidth',2);
% xlabel("Epoch (s)",'linewidth',2);
% ylabel("Error of Latitude (degree)",'linewidth',2);
% 
% subplot(2,1,2);
% plot(delta_long,'r','linewidth',2);
% xlabel("Epoch (s)",'linewidth',2);
% ylabel("Error of Longitude (degree)",'linewidth',2);


% Plot comparison of NED results
figure_NED = figure;
subplot(3,1,1);
plot(delta_n,'m','linewidth',2);hold on;
plot(delta_nMHE,'r','linewidth',2);hold on;
plot(delta_nEKF,'b','linewidth',2);hold on;
plot(delta_nEKFS,'c','linewidth',2);
ax = gca;
ax.FontSize = 25; 
% xlabel("Epoch (s)",'FontSize',40);
ylabel("North Error(m)",'FontSize',40,'FontName','Times New Roman');
legend("WLS","MHE","EKF","RTS Smoother",'FontSize',40,'FontName','Times New Roman');
% legend("WLS Results","MHE-WLS Results","EKF Results");
Mean_delta_n_d = mean(delta_n,'omitnan');
RMSE_n = rms(delta_n,'omitnan');
RMSE_n_MHE = rms(delta_nMHE,'omitnan');
RMSE_n_EKF = rms(delta_nEKF,'omitnan');
RMSE_n_EKFS = rms(delta_nEKFS,'omitnan');


subplot(3,1,2);
plot(delta_e,'m','linewidth',2);hold on;
plot(delta_eMHE,'r','linewidth',2);hold on;
plot(delta_eEKF,'b','linewidth',2);hold on;
plot(delta_eEKFS,'c','linewidth',2);
ax = gca;
ax.FontSize = 25; 
% xlabel("Epoch (s)",'FontSize',40);
ylabel("East Error (m)",'FontSize',40,'FontName','Times New Roman');
% legend("WLS Results","EKF Results","MHE-WLS Results");
Mean_delta_e_d = mean(delta_e,'omitnan');
% Mean_delta_lon_EKF_d = mean(delta_lon_EKF,'omitnan');
% Mean_delta_lon_MHE_d = mean(delta_lon_MHE,'omitnan');
RMSE_e = rms(delta_e,'omitnan');
RMSE_e_MHE = rms(delta_eMHE,'omitnan');
RMSE_e_EKF = rms(delta_eEKF,'omitnan');
RMSE_e_EKFS = rms(delta_eEKFS,'omitnan');

subplot(3,1,3);
plot(delta_d,'m','linewidth',2);hold on;
plot(delta_dMHE,'r','linewidth',2);hold on;
plot(delta_dEKF,'b','linewidth',2);hold on;
plot(delta_dEKFS,'c','linewidth',2);
ax = gca;
ax.FontSize = 25; 
xlabel("Epoch (s)",'FontSize',40,'FontName','Times New Roman');
ylabel("Down Error (m)",'FontSize',40,'FontName','Times New Roman');
% legend("WLS Results","EKF Results","MHE-WLS Results");
Mean_delta_d_d = mean(delta_d,'omitnan');
% Mean_delta_alt_EKF_d = mean(delta_alt_EKF,'omitnan');
% Mean_delta_alt_MHE_d = mean(delta_alt_MHE,'omitnan');
RMSE_d = rms(delta_d,'omitnan');
RMSE_d_MHE = rms(delta_dMHE,'omitnan');
RMSE_d_EKF = rms(delta_dEKF,'omitnan');
RMSE_d_EKFS = rms(delta_dEKFS,'omitnan');
saveas(figure_NED,[dirName,'/NED_Results.fig']);

% Plot comparison of LLA results
figure;
subplot(3,1,1);
plot(delta_lat,'m','linewidth',2);hold on;
plot(delta_lat_EKF,'b','linewidth',2);hold on;
plot(delta_lat_MHE,'r','linewidth',2);hold on;
plot(delta_lat_EKFS,'c','linewidth',2);
xlabel("Epoch (s)",'linewidth',2);
ylabel("Error of Latitude (Degree)",'linewidth',2);
legend("WLS Results","EKF Results","MHE-WLS Results","RTS Smoothing Results");
Mean_delta_lat_d = mean(delta_lat,'omitnan');
Mean_delta_lat_EFK_d = mean(delta_lat_EKF,'omitnan');
Mean_delta_lat_MHE_d = mean(delta_lat_MHE,'omitnan');
RMSE_lat_d = rms(delta_lat,'omitnan');
RMSE_lat_EKF_d = rms(delta_lat_EKF,'omitnan');
RMSE_lat_MHE_d = rms(delta_lat_MHE,'omitnan');

subplot(3,1,2);
plot(delta_lon,'m','linewidth',2);hold on;
plot(delta_lon_EKF,'b','linewidth',2);hold on;
plot(delta_lon_MHE,'r','linewidth',2);hold on;
plot(delta_lon_EKFS,'c','linewidth',2);
xlabel("Epoch (s)",'linewidth',2);
ylabel("Error of Longitude (Degree)",'linewidth',2);
Mean_delta_lon_d = mean(delta_lon,'omitnan');
Mean_delta_lon_EKF_d = mean(delta_lon_EKF,'omitnan');
Mean_delta_lon_MHE_d = mean(delta_lon_MHE,'omitnan');
RMSE_lon_d = rms(delta_lon,'omitnan');
RMSE_lon_EKF_d = rms(delta_lon_EKF,'omitnan');
RMSE_lon_MHE_d = rms(delta_lon_MHE,'omitnan');

subplot(3,1,3);
plot(delta_alt,'m','linewidth',2);hold on;
plot(delta_alt_EKF,'b','linewidth',2);hold on;
plot(delta_alt_MHE,'r','linewidth',2);hold on;
plot(delta_alt_EKFS,'c','linewidth',2);
xlabel("Epoch (s)",'linewidth',2);
ylabel("Error of Altitude (Meter)",'linewidth',2);
Mean_delta_alt_d = mean(delta_alt,'omitnan');
Mean_delta_alt_EKF_d = mean(delta_alt_EKF,'omitnan');
Mean_delta_alt_MHE_d = mean(delta_alt_MHE,'omitnan');
RMSE_alt_d = rms(delta_alt,'omitnan');
RMSE_alt_EKF_d = rms(delta_alt_EKF,'omitnan');
RMSE_alt_MHE_d = rms(delta_alt_MHE,'omitnan');

% %% Write WLS results into a file
% offset = 0;
% tRx_i = tRx_i + offset;
% f_3D_WLS_PVT = fopen([dirName,'/WLS_PVT_3D.csv'],'w');
% index = (tRx_i:tRx_row)';
% M = [index, gpsPvt.allLlaDegDegM(tRx_i:tRx_row,2),gpsPvt.allLlaDegDegM(tRx_i:tRx_row,1),gpsPvt.allLlaDegDegM(tRx_i:tRx_row,3)];
% fprintf(f_3D_WLS_PVT,'%f , %f, %f, %f\n',M');
% fclose(f_3D_WLS_PVT);
% 
% 
% %% Write EKF results into a file
% f_3D_EKF_PVT = fopen([dirName,'/EKF_PVT_3D.csv'],'w');
% index = (tRx_i:tRx_row)';
% M = [index, gpsPvt.allLlaDegDegMEKF(tRx_i:tRx_row,2),gpsPvt.allLlaDegDegMEKF(tRx_i:tRx_row,1),gpsPvt.allLlaDegDegMEKF(tRx_i:tRx_row,3)];
% fprintf(f_3D_EKF_PVT,'%f , %f, %f, %f\n',M');
% fclose(f_3D_EKF_PVT);
% 
% %% Write RTS Smoothing results into a file
% f_3D_RTS_PVT = fopen([dirName,'/RTS_PVT_3D.csv'],'w');
% index = (tRx_i:tRx_row)';
% M = [index, gpsPvt.allLlaDegDegMEKFS(tRx_i:tRx_row,2),gpsPvt.allLlaDegDegMEKFS(tRx_i:tRx_row,1),gpsPvt.allLlaDegDegMEKFS(tRx_i:tRx_row,3)];
% fprintf(f_3D_RTS_PVT,'%f , %f, %f, %f\n',M');
% fclose(f_3D_RTS_PVT);
% 
% %% Write MHE results into a file
% f_3D_MHE_PVT = fopen([dirName,'/MHE_PVT_3D.csv'],'w');
% index = (tRx_i:tRx_row)';
% M = [index, gpsPvt.allLlaDegDegMMHE(tRx_i:tRx_row,2),gpsPvt.allLlaDegDegMMHE(tRx_i:tRx_row,1),gpsPvt.allLlaDegDegMMHE(tRx_i:tRx_row,3)];
% fprintf(f_3D_MHE_PVT,'%f , %f, %f, %f\n',M');
% fclose(f_3D_MHE_PVT);
% 
% %% Write ground truth results into a file
% f_3D_GT_PVT = fopen([dirName,'/GT_PVT_3D.csv'],'w');
% index = (1+offset:GT_row0)';
% M = [index, GroundTruth0(1:GT_row0,4),GroundTruth0(1:GT_row0,5),GroundTruth0(1:GT_row0,6)];
% fprintf(f_3D_GT_PVT,'%f , %f, %f, %f\n',M');
% fclose(f_3D_GT_PVT);


% %% Write WLS results and U-blox results into a file
% f_3D_WLS_uBlox_PVT = fopen('WLS_uBlox_PVT_3D.csv','w');
% M = [gpsPvt.allLlaDegDegM(tRx_i:tRx_row,2),gpsPvt.allLlaDegDegM(tRx_i:tRx_row,1),GroundTruth0(1:GT_row0,4),GroundTruth0(1:GT_row0,5)];
% fprintf(f_3D_WLS_uBlox_PVT,'%f , %f , %f , %f\n',M');
% fclose(f_3D_WLS_uBlox_PVT);

% %% plot Pvt results
% h4 = figure;
% ts = 'Raw Pseudoranges, Weighted Least Squares solution';
% PlotPvt(gpsPvt,prFileName,param.llaTrueDegDegM,ts); drawnow;
% h5 = figure;
% PlotPvtStates(gpsPvt,prFileName);
% 
% %% Plot Accumulated Delta Range 
% if any(any(isfinite(gnssMeas.AdrM) & gnssMeas.AdrM~=0))
%     [gnssMeas]= ProcessAdr(gnssMeas);
%     h6 = figure;
%     PlotAdr(gnssMeas,prFileName,colors);
%     [adrResid]= GpsAdrResiduals(gnssMeas,allGpsEph,param.llaTrueDegDegM);drawnow
%     h7 = figure;
%     PlotAdrResids(adrResid,gnssMeas,prFileName,colors);
% end
%% end of ProcessGnssMeasScript
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2016 Google Inc.
% Copyright 2023 Nanyang Technological University
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
