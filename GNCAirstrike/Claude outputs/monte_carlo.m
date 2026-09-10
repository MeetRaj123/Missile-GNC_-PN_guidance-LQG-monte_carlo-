%% monte_carlo.m
% Monte Carlo validation harness for the GNCAirstrike PN-guidance missile
% model. Does NOT modify the base model logic (guidance law, LQR, Kalman
% filter, terrain override) -- it only varies initial conditions, target
% location, and process/measurement noise seeds across trials, runs the
% closed-loop simulation for each, and aggregates results.
clear; clc;

%% ---- Load model + baseline workspace ----
% NOTE: model.m itself contains "clear all; close all;", so it must run
% BEFORE any of this script's own variables (N, mdl, thresholds, etc.)
% are defined -- otherwise they get wiped out.
run('model.m');

mdl = 'simulink_model';
if ~bdIsLoaded(mdl); load_system(mdl); end

%% ---- Configuration (edit here) ----
N = 300;                    % number of Monte Carlo trials
missDistThreshold = 1000;   % m, "hit" definition (slant range at closest approach)
sigma_latlon_init = 0.01;   % deg, 1-sigma scatter on launch position
sigma_latlon_tgt  = 0.01;   % deg, 1-sigma scatter on target position
masterSeed = 12345;
stopTimeOverride = '200';   % s, generous cap (nominal flight time ~78 s)

d_obstacle_nom = d_obstacle_calc(LAT_INIT, LON_INIT, LAT_OBS, LON_OBS, R);

rng(masterSeed);

ctrlPath  = ['simulink_model/Missile LQG Controller' newline '+ Airframe Model'];
wNoiseBlk = [ctrlPath '/w/noise'];
vNoiseBlk = [ctrlPath '/v/noise'];

haveParsim = exist('Simulink.SimulationInput', 'class') == 8;
d2r = pi/180;

%% ---- Build per-trial SimulationInput objects ----
trialParams = struct('LAT_INIT', cell(1,N), 'LON_INIT', cell(1,N), ...
    'LAT_TARGET', cell(1,N), 'LON_TARGET', cell(1,N), ...
    'seed_w', cell(1,N), 'seed_v', cell(1,N), 'target_downrange', cell(1,N));

if haveParsim
    simIn(1:N) = Simulink.SimulationInput(mdl);
end

for i = 1:N
    LAT_INIT_i   = LAT_INIT   + sigma_latlon_init*randn();
    LON_INIT_i   = LON_INIT   + sigma_latlon_init*randn();
    LAT_TARGET_i = LAT_TARGET + sigma_latlon_tgt*randn();
    LON_TARGET_i = LON_TARGET + sigma_latlon_tgt*randn();

    l1 = LAT_INIT_i*d2r;   u1 = LON_INIT_i*d2r;
    l2 = LAT_TARGET_i*d2r; u2 = LON_TARGET_i*d2r;
    dl = l2-l1; du = u2-u1;
    a_ = sin(dl/2)^2 + cos(l1)*cos(l2)*sin(du/2)^2;
    c_ = 2*atan2(sqrt(a_), sqrt(1-a_));
    d_i = R*c_;
    r_i = sqrt(d_i^2 + (ELEV_TARGET-ELEV_INIT)^2);

    yaw_init_i = azimuth(LAT_INIT_i, LON_INIT_i, LAT_TARGET_i, LON_TARGET_i);
    yaw_i = yaw_init_i*d2r;

    dh_i = abs(ELEV_TARGET - ELEV_INIT);
    FPA_INIT_i = atan(dh_i/d_i);

    seed_w = 1000+i; seed_v = 5000+i;

    trialParams(i).LAT_INIT   = LAT_INIT_i;
    trialParams(i).LON_INIT   = LON_INIT_i;
    trialParams(i).LAT_TARGET = LAT_TARGET_i;
    trialParams(i).LON_TARGET = LON_TARGET_i;
    trialParams(i).seed_w = seed_w;
    trialParams(i).seed_v = seed_v;
    trialParams(i).target_downrange = d_i;

    if haveParsim
        simIn(i) = simIn(i).setVariable('LAT_INIT', LAT_INIT_i);
        simIn(i) = simIn(i).setVariable('LON_INIT', LON_INIT_i);
        simIn(i) = simIn(i).setVariable('LAT_TARGET', LAT_TARGET_i);
        simIn(i) = simIn(i).setVariable('LON_TARGET', LON_TARGET_i);
        simIn(i) = simIn(i).setVariable('yaw_init', yaw_init_i);
        simIn(i) = simIn(i).setVariable('yaw', yaw_i);
        simIn(i) = simIn(i).setVariable('FPA_INIT', FPA_INIT_i);
        simIn(i) = simIn(i).setVariable('d', d_i);
        simIn(i) = simIn(i).setVariable('r', r_i);
        simIn(i) = simIn(i).setBlockParameter(wNoiseBlk, 'seed', mat2str(seed_w));
        simIn(i) = simIn(i).setBlockParameter(vNoiseBlk, 'seed', mat2str(seed_v));
        simIn(i) = simIn(i).setModelParameter('StopTime', stopTimeOverride);
    end
end

%% ---- Run the batch ----
fprintf('Running %d Monte Carlo trials...\n', N);
ticBatch = tic;
if haveParsim
    simOut = parsim(simIn, 'ShowProgress', 'on', 'TransferBaseWorkspaceVariables', 'on');
else
    simOut(1:N) = Simulink.SimulationOutput();
    for i = 1:N
        simOut(i) = sim(mdl, 'StopTime', stopTimeOverride);
    end
end
fprintf('Batch complete in %.1f s\n', toc(ticBatch));

%% ---- Post-process each trial ----
missDistance   = nan(N,1);
finalAltitude  = nan(N,1);
flightTime     = nan(N,1);
overrideFired  = false(N,1);
saturated      = false(N,1);
rmsQError      = nan(N,1);
rmsThetaError  = nan(N,1);
trialOK        = false(N,1);
trajX   = cell(N,1);
trajAlt = cell(N,1);

for i = 1:N
    try
        logs = simOut(i).get('logsout');
        rngSig  = logs.get('RNG_sig').Values;
        warnSig = logs.get('warn_sig').Values;
        xSig    = logs.get('x_sig').Values;
        zSig    = logs.get('z_sig').Values;
        satSig  = logs.get('qcmd_sat_sig').Values;
        qSig    = logs.get('q_sig').Values;
        qhatSig = logs.get('qhat_sig').Values;
        thSig   = logs.get('theta_sig').Values;
        thhSig  = logs.get('thetahat_sig').Values;

        missDistance(i)  = min(rngSig.Data);
        finalAltitude(i) = -zSig.Data(end);
        flightTime(i)    = xSig.Time(end);
        overrideFired(i) = any(warnSig.Data ~= 0);

        satTol = 1e-6; satLimit = 0.5;
        saturated(i) = any(abs(satSig.Data) >= (satLimit - satTol));

        n  = min(numel(qSig.Data), numel(qhatSig.Data));
        rmsQError(i) = sqrt(mean((qSig.Data(1:n) - qhatSig.Data(1:n)).^2));
        n2 = min(numel(thSig.Data), numel(thhSig.Data));
        rmsThetaError(i) = sqrt(mean((thSig.Data(1:n2) - thhSig.Data(1:n2)).^2));

        trajX{i}   = squeeze(xSig.Data);
        trajAlt{i} = -squeeze(zSig.Data);
        trialOK(i) = true;
    catch ME
        fprintf('Trial %d post-processing failed: %s\n', i, ME.message);
    end
end

%% ---- Summary statistics ----
validIdx = trialOK & ~isnan(missDistance);
mMiss = missDistance(validIdx);
meanMiss = mean(mMiss);
stdMiss  = std(mMiss);
cep50    = median(mMiss);
successRate    = mean(mMiss <= missDistThreshold);
overrideRate   = mean(overrideFired(validIdx));
saturationRate = mean(saturated(validIdx));

fprintf('\n=== Monte Carlo Summary (N=%d, %d valid) ===\n', N, sum(validIdx));
fprintf('Mean miss distance    : %.1f m\n', meanMiss);
fprintf('Std miss distance     : %.1f m\n', stdMiss);
fprintf('CEP50 (median miss)   : %.1f m\n', cep50);
fprintf('Success rate (<=%.0fm) : %.1f%%\n', missDistThreshold, 100*successRate);
fprintf('Terrain override fired: %.1f%% of trials\n', 100*overrideRate);
fprintf('Control saturated     : %.1f%% of trials\n', 100*saturationRate);

%% ---- Save raw results table ----
T = table((1:N)', missDistance, finalAltitude, flightTime, overrideFired, ...
    saturated, rmsQError, rmsThetaError, ...
    [trialParams.LAT_INIT]', [trialParams.LON_INIT]', ...
    [trialParams.LAT_TARGET]', [trialParams.LON_TARGET]', ...
    [trialParams.target_downrange]', ...
    'VariableNames', {'trial','miss_distance_m','final_altitude_m','flight_time_s', ...
    'terrain_override_fired','control_saturated','rms_q_error','rms_theta_error', ...
    'LAT_INIT','LON_INIT','LAT_TARGET','LON_TARGET','target_downrange_m'});

writetable(T, 'monte_carlo_results.csv');
save('monte_carlo_results.mat', 'T', 'trialParams', 'missDistance', 'finalAltitude', ...
    'flightTime', 'overrideFired', 'saturated', 'rmsQError', 'rmsThetaError', ...
    'trajX', 'trajAlt', 'meanMiss', 'stdMiss', 'cep50', 'successRate', ...
    'overrideRate', 'saturationRate', 'N', 'missDistThreshold', '-v7');

%% ---- Plot 1: miss distance histogram ----
fh1 = figure('Visible','off','Position',[100 100 800 500]);
histogram(mMiss, 30, 'FaceColor',[0.2 0.4 0.8]);
hold on;
xline(missDistThreshold, 'r--', 'LineWidth', 1.5);
xline(cep50, 'g-', 'LineWidth', 1.5);
xlabel('Miss distance (m)'); ylabel('Number of trials');
title(sprintf('Miss Distance Distribution (N=%d) - mean=%.0fm, CEP50=%.0fm, success=%.1f%%', ...
    sum(validIdx), meanMiss, cep50, 100*successRate));
legend('Trials', sprintf('%.0fm threshold', missDistThreshold), sprintf('CEP50=%.0fm', cep50));
grid on;
saveas(fh1, 'miss_distance_histogram.png');
close(fh1);

%% ---- Plot 2: trajectory overlay ----
fh2 = figure('Visible','off','Position',[100 100 900 600]);
hold on;
for i = 1:N
    if trialOK(i)
        c = [0.3 0.5 0.9];
        if missDistance(i) > missDistThreshold; c = [0.9 0.3 0.3]; end
        plot(trajX{i}/1000, trajAlt{i}, 'Color', [c 0.25], 'LineWidth', 0.75);
    end
end
plot(0, ELEV_INIT, '^', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Launch');
plot(d/1000, ELEV_TARGET, 'p', 'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'Nominal target');
xline(d_obstacle_nom/1000, 'Color', [0.9 0.6 0.1], 'LineStyle', '--', 'LineWidth', 1.2);
xlabel('Downrange distance (km)'); ylabel('Altitude (m)');
title(sprintf('Trajectory Overlay - All %d Monte Carlo Trials (blue=hit, red=miss)', N));
grid on;
ylim([0 max(ELEV_INIT*1.05, 1)]);
saveas(fh2, 'trajectory_overlay.png');
close(fh2);

fprintf('\nDone. Outputs written to:\n  monte_carlo_results.csv\n  monte_carlo_results.mat\n  miss_distance_histogram.png\n  trajectory_overlay.png\n');

%% ---- Helper ----
function d_obs = d_obstacle_calc(lat1, lon1, lat2, lon2, Rearth)
d2r_ = pi/180;
l1 = lat1*d2r_; u1 = lon1*d2r_; l2 = lat2*d2r_; u2 = lon2*d2r_;
dl = l2-l1; du = u2-u1;
a_ = sin(dl/2)^2 + cos(l1)*cos(l2)*sin(du/2)^2;
c_ = 2*atan2(sqrt(a_), sqrt(1-a_));
d_obs = Rearth*c_;
end
