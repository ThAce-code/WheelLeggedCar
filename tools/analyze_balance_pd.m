% Analyze balance PD tuning captures collected by tools/collect_balance_data.ps1.
%
% Run from repository root in MATLAB:
%   tools/analyze_balance_pd
%
% Inputs:
%   data/balance_capture_*.csv
%
% Outputs:
%   data/balance_pd_summary.csv
%   data/balance_pd_timeseries.png
%   data/balance_pd_score.png
%   data/balance_pd_phase.png

clear; clc;

dataDir = fullfile("data");
files = dir(fullfile(dataDir, "balance_capture_*.csv"));
if isempty(files)
    error("No balance capture CSV files found under data/.");
end

% Keep the newest capture for each note. If note is empty, use filename.
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

summary = table();
series = struct([]);

for i = 1:numel(keepFiles)
    file = keepFiles(i);
    raw = readtable(fullfile(dataDir, file.name));
    note = string(raw.note(1));
    if strlength(note) == 0
        note = string(file.name);
    end

    active = raw(abs(raw.balance_mode - 2.0) < 0.01, :);
    feedbackBadRows = sum(active.feedback_online < 0.5);
    active = active(active.feedback_online >= 0.5, :);

    hasMotorColumns = all(ismember(["left_motor_rpm", "right_motor_rpm", "left_duty", "right_duty"], string(active.Properties.VariableNames)));
    hasGainColumns = all(ismember(["balance_kp", "balance_kd"], string(active.Properties.VariableNames)));

    if hasMotorColumns
        avgMotorRpm = (active.left_motor_rpm + active.right_motor_rpm) / 2.0;
        avgDuty = (active.left_duty + active.right_duty) / 2.0;
    else
        avgMotorRpm = NaN(height(active), 1);
        avgDuty = NaN(height(active), 1);
    end

    if hasGainColumns
        balanceKp = median(active.balance_kp);
        balanceKd = median(active.balance_kd);
    else
        balanceKp = NaN;
        balanceKd = NaN;
    end

    if height(active) < 2
        warning("Skipping %s: not enough active feedback-online rows.", file.name);
        continue;
    end

    t = active.elapsed_s - active.elapsed_s(1);
    pitch = active.pitch_deg;
    pitchRate = active.pitch_rate_dps;
    balanceRpm = active.balance_rpm;

    dtMs = diff(active.time_ms);
    dtMs = dtMs(dtMs > 0);

    pitchAbs = abs(pitch);
    rateAbs = abs(pitchRate);
    rpmAbs = abs(balanceRpm);
    satRows = sum(rpmAbs >= 145.0);

    pitchAbsP95 = local_percentile(pitchAbs, 95);
    rateAbsP95 = local_percentile(rateAbs, 95);
    rpmAbsP95 = local_percentile(rpmAbs, 95);
    pitchRms = sqrt(mean(pitch .^ 2));
    rpmRms = sqrt(mean(balanceRpm .^ 2));
    effortPerPitch = rpmAbsP95 / max(pitchAbsP95, 0.001);

    % Lower score is better: angle containment first, then control effort.
    qualityScore = pitchAbsP95 + (0.04 * rpmAbsP95) + (0.01 * rateAbsP95);

    summary = [summary; table(note, string(file.name), height(raw), height(active), ...
        min(t), max(t), median(dtMs), min(pitch), max(pitch), pitchRms, pitchAbsP95, ...
        rateAbsP95, min(balanceRpm), max(balanceRpm), rpmRms, rpmAbsP95, ...
        effortPerPitch, satRows, feedbackBadRows, ...
        local_percentile(abs(avgMotorRpm), 95), local_percentile(abs(avgDuty), 95), ...
        balanceKp, balanceKd, qualityScore, ...
        "VariableNames", ["note", "file", "rows_total", "rows_active", ...
        "time_start_s", "time_end_s", "dt_median_ms", "pitch_min_deg", ...
        "pitch_max_deg", "pitch_rms_deg", "pitch_abs_p95", ...
        "pitch_rate_abs_p95", "balance_rpm_min", "balance_rpm_max", ...
        "balance_rpm_rms", "balance_rpm_abs_p95", "effort_per_pitch", ...
        "sat_rows", "feedback_bad_rows", ...
        "avg_motor_rpm_abs_p95", "avg_duty_abs_p95", ...
        "balance_kp", "balance_kd", "quality_score"])]; %#ok<AGROW>

    series(end + 1).note = note; %#ok<SAGROW>
    series(end).t = t;
    series(end).pitch = pitch;
    series(end).pitchRate = pitchRate;
    series(end).balanceRpm = balanceRpm;
    series(end).avgMotorRpm = avgMotorRpm;
    series(end).avgDuty = avgDuty;
end

if isempty(summary)
    error("No usable balance captures after filtering.");
end

summary = sortrows(summary, "quality_score", "ascend");
writetable(summary, fullfile(dataDir, "balance_pd_summary.csv"));

disp("Latest capture used per note, sorted by lower quality_score:");
disp(summary(:, ["note", "file", "rows_active", "pitch_abs_p95", ...
    "pitch_rate_abs_p95", "balance_rpm_abs_p95", "effort_per_pitch", ...
    "sat_rows", "quality_score"]));

figure("Name", "Balance PD time series", "Color", "w");
tiledlayout(2, 1, "TileSpacing", "compact");
nexttile;
hold on; grid on;
for i = 1:numel(series)
    plot(series(i).t, series(i).pitch, "DisplayName", series(i).note);
end
yline(0, "k:");
ylabel("pitch (deg)");
title("Pitch response by PD setting");
legend("Location", "eastoutside", "Interpreter", "none");

nexttile;
hold on; grid on;
for i = 1:numel(series)
    plot(series(i).t, series(i).balanceRpm, "DisplayName", series(i).note);
end
yline(0, "k:");
ylabel("balance rpm");
xlabel("elapsed in BALANCE_TEST (s)");
title("Controller effort by PD setting");
saveas(gcf, fullfile(dataDir, "balance_pd_timeseries.png"));

figure("Name", "Balance PD score", "Color", "w");
tiledlayout(2, 1, "TileSpacing", "compact");
nexttile;
bar(categorical(summary.note), summary.pitch_abs_p95);
grid on;
ylabel("95% |pitch| (deg)");
title("Lower pitch spread is better");

nexttile;
bar(categorical(summary.note), summary.balance_rpm_abs_p95);
grid on;
ylabel("95% |balance rpm|");
title("Lower effort is better when pitch spread is comparable");
xlabel("PD setting");
saveas(gcf, fullfile(dataDir, "balance_pd_score.png"));

figure("Name", "Balance PD phase plot", "Color", "w");
hold on; grid on;
for i = 1:numel(series)
    plot(series(i).pitch, series(i).pitchRate, ".", "DisplayName", series(i).note);
end
xlabel("pitch (deg)");
ylabel("pitch rate (deg/s)");
title("Pitch phase plot by PD setting");
legend("Location", "eastoutside", "Interpreter", "none");
saveas(gcf, fullfile(dataDir, "balance_pd_phase.png"));

figure("Name", "Balance motor response", "Color", "w");
tiledlayout(2, 1, "TileSpacing", "compact");
nexttile;
hold on; grid on;
for i = 1:numel(series)
    if isfield(series(i), "avgMotorRpm")
        plot(series(i).t, series(i).avgMotorRpm, "DisplayName", series(i).note);
    end
end
yline(0, "k:");
ylabel("avg motor rpm");
title("Average motor response by PD setting");
legend("Location", "eastoutside", "Interpreter", "none");

nexttile;
hold on; grid on;
for i = 1:numel(series)
    if isfield(series(i), "avgDuty")
        plot(series(i).t, series(i).avgDuty, "DisplayName", series(i).note);
    end
end
yline(0, "k:");
ylabel("avg duty");
xlabel("elapsed in BALANCE_TEST (s)");
title("Average duty effort by PD setting");
saveas(gcf, fullfile(dataDir, "balance_pd_motor_response.png"));

fprintf("Saved summary and figures under data/. Best score: %s\n", summary.note(1));

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
