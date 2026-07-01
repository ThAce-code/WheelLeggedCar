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

Q = diag([10.0, 0.2, 0.02, 2.0]);
R = 0.08;
[K_lqr, P, dareIters] = local_dlqr(A, B, Q, R); %#ok<ASGLU>
firmwareGain = -K_lqr;
clippedGain = firmwareGain;
clippedGain(1) = min(max(clippedGain(1), 0.0), 6.0);
clippedGain(2) = min(max(clippedGain(2), 0.0), 0.4);
clippedGain(3) = min(max(clippedGain(3), -0.2), 0.2);
clippedGain(4) = min(max(clippedGain(4), -1.0), 1.0);

modelSummary = table(rankPhi, condPhi, rmse(1), rmse(2), rmse(3), rmse(4), ...
    firmwareGain(1), firmwareGain(2), firmwareGain(3), firmwareGain(4), ...
    clippedGain(1), clippedGain(2), clippedGain(3), clippedGain(4), dareIters, ...
    "VariableNames", ["rank_phi", "cond_phi", "rmse_pitch_deg", ...
    "rmse_pitch_rate_dps", "rmse_wheel_rpm", "rmse_wheel_pos_rev", ...
    "raw_k_angle", "raw_k_rate", "raw_k_speed", "raw_k_pos", ...
    "clip_k_angle", "clip_k_rate", "clip_k_speed", "clip_k_pos", "dare_iters"]);
writetable(modelSummary, fullfile(dataDir, "balance_model_summary.csv"));
writetable(summary, fullfile(dataDir, "balance_model_input_summary.csv"));

disp("Input captures used for model:");
disp(summary(:, ["note", "file", "rows_model", "pitch_abs_p95", "balance_rpm_abs_p95"]));
disp("Identified A:");
disp(A);
disp("Identified B:");
disp(B);
disp("One-step RMSE [pitch, pitch_rate, wheel_rpm, wheel_pos]:");
disp(rmse');
disp("Raw firmware BL gains:");
disp(firmwareGain);
disp("Clipped firmware BL gains:");
disp(clippedGain);
fprintf("Suggested command: BL,%.3f,%.3f,%.3f,%.3f\n", ...
    clippedGain(1), clippedGain(2), clippedGain(3), clippedGain(4));

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
if clippedGain(1) ~= firmwareGain(1) || clippedGain(2) ~= firmwareGain(2)
    warning("Angle/rate gains were clipped. Treat the command as a cautious first test, not final tuning.");
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
