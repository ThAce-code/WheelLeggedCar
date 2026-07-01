% Identify a local four-state balance model and compute candidate BL gains.
%
% Run from repository root in MATLAB:
%   tools/identify_balance_model
%
% Inputs:
%   data/balance_capture_*.csv
%
% Outputs:
%   data/balance_model_summary.csv
%   data/balance_model_fit.png

clear; clc;

dataDir = fullfile("data");
files = dir(fullfile(dataDir, "balance_capture_*.csv"));
if isempty(files)
    error("No balance capture CSV files found under data/.");
end

notes = strings(numel(files), 1);
for i = 1:numel(files)
    one = readtable(fullfile(dataDir, files(i).name));
    if ismember("note", string(one.Properties.VariableNames)) && height(one) > 0
        notes(i) = string(one.note(1));
    end
    if strlength(notes(i)) == 0
        notes(i) = string(files(i).name);
    end
end

uniqueNotes = unique(notes, "stable");
keepFiles = repmat(files(1), numel(uniqueNotes), 1);
for i = 1:numel(uniqueNotes)
    idx = find(notes == uniqueNotes(i));
    [~, newestLocal] = max([files(idx).datenum]);
    keepFiles(i) = files(idx(newestLocal));
end

allX = [];
allY = [];
allU = [];
summary = table();
series = struct([]);

for i = 1:numel(keepFiles)
    file = keepFiles(i);
    raw = readtable(fullfile(dataDir, file.name));
    required = ["elapsed_s", "time_ms", "balance_mode", "feedback_online", ...
        "pitch_deg", "pitch_rate_dps", "balance_rpm", "left_motor_rpm", "right_motor_rpm"];
    if ~all(ismember(required, string(raw.Properties.VariableNames)))
        warning("Skipping %s: missing required model columns.", file.name);
        continue;
    end

    note = string(raw.note(1));
    if strlength(note) == 0
        note = string(file.name);
    end

    active = raw(abs(raw.balance_mode - 2.0) < 0.01, :);
    active = active(active.feedback_online >= 0.5, :);
    finiteMask = isfinite(active.pitch_deg) & ...
        isfinite(active.pitch_rate_dps) & ...
        isfinite(active.balance_rpm) & ...
        isfinite(active.left_motor_rpm) & ...
        isfinite(active.right_motor_rpm) & ...
        isfinite(active.time_ms);
    active = active(finiteMask, :);

    localMask = (abs(active.pitch_deg) <= 18.0) & (abs(active.balance_rpm) <= 140.0);
    active = active(localMask, :);

    if height(active) < 200
        warning("Skipping %s: only %d usable local rows.", file.name, height(active));
        continue;
    end

    dt = [median(diff(active.time_ms)); diff(active.time_ms)] / 1000.0;
    dt(dt <= 0.0) = median(dt(dt > 0.0));
    avgMotorRpm = (active.left_motor_rpm + active.right_motor_rpm) / 2.0;
    wheelPos = cumsum(avgMotorRpm .* dt / 60.0);
    wheelPos = wheelPos - wheelPos(1);

    x = [active.pitch_deg, active.pitch_rate_dps, avgMotorRpm, wheelPos]';
    u = active.balance_rpm';

    x0 = x(:, 1:end-1);
    x1 = x(:, 2:end);
    u0 = u(:, 1:end-1);

    allX = [allX, x0]; %#ok<AGROW>
    allY = [allY, x1]; %#ok<AGROW>
    allU = [allU, u0]; %#ok<AGROW>

    series(end + 1).note = note; %#ok<SAGROW>
    series(end).file = string(file.name);
    series(end).t = active.elapsed_s - active.elapsed_s(1);
    series(end).x = x;
    series(end).u = u;

    summary = [summary; table(note, string(file.name), height(raw), height(active), ...
        min(active.elapsed_s), max(active.elapsed_s), median(diff(active.time_ms)), ...
        local_percentile(abs(active.pitch_deg), 95), ...
        local_percentile(abs(active.pitch_rate_dps), 95), ...
        local_percentile(abs(active.balance_rpm), 95), ...
        "VariableNames", ["note", "file", "rows_total", "rows_model", ...
        "time_start_s", "time_end_s", "dt_median_ms", ...
        "pitch_abs_p95", "pitch_rate_abs_p95", "balance_rpm_abs_p95"])]; %#ok<AGROW>
end

if size(allX, 2) < 500
    error("Not enough usable model samples. Need at least 500; got %d.", size(allX, 2));
end

phi = [allX; allU];
theta = allY * pinv(phi);
A = theta(:, 1:4);
B = theta(:, 5);

rankPhi = rank(phi);
condPhi = cond(phi * phi');
pred = theta * phi;
err = allY - pred;
rmse = sqrt(mean(err .^ 2, 2));

fprintf("Rank phi: %d,  Cond phi: %.3g\n\n", rankPhi, condPhi);

%% LQR candidate sweep — try multiple Q to find usable gains
% Goal: K_angle > 0, K_rate > 0 (positive damping is REQUIRED for inverted pendulum)
% We sweep the pitch_rate weight Q(2,2) from small to large to force damping positive.
qRateList  = [0.2, 0.8, 2.0, 5.0, 10.0, 20.0, 40.0, 80.0, 200.0, 500.0];
qAngle     = 10.0;
qSpeed     = 0.02;
qPos       = 2.0;
R          = 0.08;
clipLims   = [0.0 6.0;  0.0 0.4;  -0.2 0.2;  -1.0 1.0];

nQ = numel(qRateList);
raw_arr = zeros(nQ, 4);
clip_arr = zeros(nQ, 4);
dare_iters_arr = zeros(nQ, 1);

for qi = 1:nQ
    Q_try = diag([qAngle, qRateList(qi), qSpeed, qPos]);
    [K_lqr, ~, iters] = local_dlqr(A, B, Q_try, R); %#ok<AGROW>
    raw = -K_lqr;
    clp = raw;
    for ci = 1:4
        clp(ci) = min(max(clp(ci), clipLims(ci,1)), clipLims(ci,2));
    end
    raw_arr(qi, :) = raw;
    clip_arr(qi, :) = clp;
    dare_iters_arr(qi) = iters;
end

candidates = table(qRateList', dare_iters_arr, ...
    raw_arr(:,1), raw_arr(:,2), raw_arr(:,3), raw_arr(:,4), ...
    clip_arr(:,1), clip_arr(:,2), clip_arr(:,3), clip_arr(:,4), ...
    'VariableNames', {'q_rate', 'dare_iters', ...
    'r_angle', 'r_rate', 'r_speed', 'r_pos', ...
    'c_angle', 'c_rate', 'c_speed', 'c_pos'});

%% Select best candidate: largest q_rate that still has r_rate > 0
validRows = candidates.r_rate > 0.0;
if any(validRows)
    best = find(validRows, 1, 'last');
else
    [~, best] = max(candidates.r_rate);
    fprintf("WARNING: No Q produced positive K_rate. Picking least-negative.\n");
end

fprintf("=== LQR candidate sweep (varying Q(2,2) = pitch_rate penalty) ===\n");
fprintf("q_rate  iters raw_angle raw_rate raw_speed raw_pos  clip_angle clip_rate clip_speed clip_pos\n");
for qi = 1:numel(qRateList)
    mark = "";
    if qi == best, mark = " <-- BEST"; end
    fprintf("%6.1f  %3d    %8.3f %8.3f %8.3f %8.3f   %8.3f  %8.3f  %8.3f  %8.3f%s\n", ...
        candidates.q_rate(qi), candidates.dare_iters(qi), ...
        candidates.r_angle(qi), candidates.r_rate(qi), ...
        candidates.r_speed(qi), candidates.r_pos(qi), ...
        candidates.c_angle(qi), candidates.c_rate(qi), ...
        candidates.c_speed(qi), candidates.c_pos(qi), mark);
end

fprintf("\n=== Best candidate: BL,%.3f,%.3f,%.3f,%.3f ===\n", ...
    candidates.c_angle(best), candidates.c_rate(best), ...
    candidates.c_speed(best), candidates.c_pos(best));
fprintf("(q_rate = %.1f, DARE iters = %d)\n", ...
    candidates.q_rate(best), candidates.dare_iters(best));

writetable(candidates, fullfile(dataDir, "balance_model_summary.csv"));
writetable(summary, fullfile(dataDir, "balance_model_input_summary.csv"));

fprintf("\n--- Model diagnostics ---\n");
disp("Identified A:");
disp(A);
disp("Identified B:");
disp(B);
fprintf("One-step RMSE [pitch, pitch_rate, wheel_rpm, wheel_pos]: ");
fprintf("%.4f  ", rmse);
fprintf("\n");

figure("Name", "Balance model one-step fit", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");
labels = ["pitch deg", "pitch rate dps", "avg motor rpm", "wheel pos rev"];
for stateIndex = 1:4
    nexttile;
    hold on; grid on;
    plot(allY(stateIndex, 1:min(2000, size(allY, 2))), "DisplayName", "measured");
    plot(pred(stateIndex, 1:min(2000, size(pred, 2))), "DisplayName", "one-step fit");
    ylabel(labels(stateIndex));
    if stateIndex == 1
        legend("Location", "best");
    end
end
xlabel("sample");
saveas(gcf, fullfile(dataDir, "balance_model_fit.png"));

if rankPhi < 5
    warning("Regressor rank is %d, expected 5. Collect BI excitation data before trusting gains.", rankPhi);
end
if condPhi > 1.0e8
    warning("Regressor condition is high: %.3g. Gains may be unreliable.", condPhi);
end
bestRaw = [candidates.r_angle(best), candidates.r_rate(best), ...
           candidates.r_speed(best), candidates.r_pos(best)];
bestClip = [candidates.c_angle(best), candidates.c_rate(best), ...
            candidates.c_speed(best), candidates.c_pos(best)];
if any(bestRaw ~= bestClip)
    warning("Gains were clipped. Raw=[%.3f %.3f %.3f %.3f] -> Clip=[%.3f %.3f %.3f %.3f]. Treat as cautious first test.", ...
        bestRaw(1), bestRaw(2), bestRaw(3), bestRaw(4), ...
        bestClip(1), bestClip(2), bestClip(3), bestClip(4));
end

function [K, P, iterations] = local_dlqr(A, B, Q, R)
    P = Q;
    for iterations = 1:1000
        den = R + (B' * P * B);
        K = den \ (B' * P * A);
        nextP = (A' * P * A) - (A' * P * B * K) + Q;
        if norm(nextP - P, "fro") < 1.0e-9
            P = nextP;
            return;
        end
        P = nextP;
    end
end

function value = local_percentile(values, percent)
    values = sort(values(:));
    if isempty(values)
        value = NaN;
        return;
    end

    index = 1 + ((numel(values) - 1) * percent / 100.0);
    low = floor(index);
    high = ceil(index);
    if low == high
        value = values(low);
    else
        weight = index - low;
        value = (values(low) * (1.0 - weight)) + (values(high) * weight);
    end
end
