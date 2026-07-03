% Speed and turn loop step-response analysis for Balance Drive v2.
%
% Run from repository root in MATLAB:
%   tools/tune_drive_loops
%
% Inputs:
%   data/balance_capture_*.csv  (V2 21-channel telemetry)
%
% Outputs:
%   data/drive_tune_report.csv
%   data/drive_speed_step.png
%   data/drive_turn_step.png

clear; clc;

dataDir = fullfile("data");
files = dir(fullfile(dataDir, "balance_capture_*.csv"));
if isempty(files)
    error("No balance capture CSV files found under data/.");
end

% Keep the newest file for each unique note
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

fprintf("Analyzing %d unique captures...\n", numel(keepFiles));

report = table();
allSpeedSteps = {};
allTurnSteps = {};

for fileIdx = 1:numel(keepFiles)
    file = keepFiles(fileIdx);
    raw = readtable(fullfile(dataDir, file.name));
    note = string(raw.note(1));
    if strlength(note) == 0, note = string(file.name); end

    % Check for V2 columns
    v2Cols = ["forward_target_rpm", "forward_actual_rpm", ...
              "speed_pitch_offset_deg", "pitch_setpoint_deg", ...
              "turn_target_dps", "gyro_z_dps", "turn_rpm", ...
              "gyro_z_raw_dps", "turn_error_dps", "turn_integral", ...
              "turn_kp", "turn_ki", "imu_age_ms", "wheel_age_ms"];
    if ~all(ismember(v2Cols, string(raw.Properties.VariableNames)))
        fprintf("  Skipping %s: not a V2 capture.\n", file.name);
        continue;
    end

    active = raw(abs(raw.balance_mode - 2.0) < 0.01, :);
    active = active(active.feedback_online >= 0.5, :);
    if height(active) < 50
        fprintf("  Skipping %s: too few active samples.\n", file.name);
        continue;
    end

    t = active.elapsed_s - active.elapsed_s(1);

    %% --- Speed step analysis ---
    fwdTarget = active.forward_target_rpm;
    fwdActual = active.forward_actual_rpm;
    pitchOff  = active.speed_pitch_offset_deg;
    pitchSp   = active.pitch_setpoint_deg;

    % Find speed step: forward_target rising edge crossing 10 RPM
    stepStart = find(diff(fwdTarget) > 5.0, 1, 'first');
    if isempty(stepStart)
        stepStart = find(fwdTarget > 5.0, 1, 'first');
    end
    if ~isempty(stepStart) && stepStart > 5 && stepStart < height(active) - 20
        stepStart = stepStart - 3;  % back up a few samples
        stepEnd = min(stepStart + 800, height(active));
        stepT = t(stepStart:stepEnd) - t(stepStart);
        stepFwd = fwdActual(stepStart:stepEnd);
        stepTgt = fwdTarget(stepStart:stepEnd);
        stepOff = pitchOff(stepStart:stepEnd);

        % Metrics
        finalTgt = median(stepTgt(end-50:end));
        finalAct = median(stepFwd(end-50:end));
        steadyErr = finalTgt - finalAct;

        % Rise time: time from 10% to 90% of the step response
        stepAmp = max(stepFwd) - stepFwd(1);
        if stepAmp > 2.0
            lo = stepFwd(1) + 0.10 * stepAmp;
            hi = stepFwd(1) + 0.90 * stepAmp;
            t10 = stepT(find(stepFwd >= lo, 1, 'first'));
            t90 = stepT(find(stepFwd >= hi, 1, 'first'));
            if ~isempty(t10) && ~isempty(t90)
                riseTime = t90 - t10;
            else
                riseTime = NaN;
            end

            % Overshoot
            overshootPct = 100.0 * (max(stepFwd) - finalAct) / max(finalAct, 0.01);

            % Speed sensitivity: delta wheel_rpm per delta pitch_offset (steady state)
            offDelta = median(stepOff(end-50:end)) - stepOff(1);
            if abs(offDelta) > 0.1
                rpmPerDeg = finalAct / offDelta;
            else
                rpmPerDeg = NaN;
            end

            fprintf("\n--- %s ---\n", note);
            fprintf("  Final target:  %.1f RPM\n", finalTgt);
            fprintf("  Final actual:  %.1f RPM\n", finalAct);
            fprintf("  Steady error:  %.1f RPM\n", steadyErr);
            fprintf("  Rise time:     %.2f s\n", riseTime);
            fprintf("  Overshoot:     %.1f %%\n", overshootPct);
            fprintf("  RPM per deg:   %.1f\n", rpmPerDeg);
            fprintf("  Pitch offset:  %.2f deg\n", offDelta);
        else
            riseTime = NaN;
            overshootPct = NaN;
            rpmPerDeg = NaN;
            fprintf("\n--- %s ---\n", note);
            fprintf("  Step amplitude too small (%.1f RPM). No metrics.\n", stepAmp);
        end

        report = [report; table(note, string(file.name), finalTgt, finalAct, ...
            steadyErr, riseTime, overshootPct, rpmPerDeg, ...
            'VariableNames', ["note", "file", "target_rpm", "actual_rpm", ...
            "steady_err_rpm", "rise_time_s", "overshoot_pct", "rpm_per_deg"])]; %#ok<AGROW>

        allSpeedSteps{end+1} = struct('note', note, 't', stepT, ...
            'fwd', stepFwd, 'tgt', stepTgt, 'off', stepOff); %#ok<AGROW>
    else
        fprintf("  Skipping %s: no speed step detected.\n", note);
    end

    %% --- Turn step analysis ---
    turnTgt = active.turn_target_dps;
    gyroZ   = active.gyro_z_dps;
    turnRpm = active.turn_rpm;
    gyroRaw = active.gyro_z_raw_dps;
    turnErr = active.turn_error_dps;
    turnInt = active.turn_integral;
    imuAge = active.imu_age_ms;
    wheelAge = active.wheel_age_ms;

    zeroTurn = active(abs(active.turn_target_dps) < 0.5, :);
    rejectReason = "";
    zeroGyroMean = NaN;
    zeroGyroStd = NaN;
    if height(zeroTurn) >= 20
        zeroGyroMean = mean(zeroTurn.gyro_z_dps, "omitnan");
        zeroGyroStd = std(zeroTurn.gyro_z_dps, "omitnan");
        if abs(zeroGyroMean) > 3.0
            rejectReason = rejectReason + "zero gyro bias > 3 dps; ";
        end
        if zeroGyroStd > 8.0
            rejectReason = rejectReason + "zero gyro std > 8 dps; ";
        end
    else
        rejectReason = rejectReason + "less than 20 zero-turn samples; ";
    end

    if max(imuAge, [], "omitnan") > 15.0
        rejectReason = rejectReason + "IMU age > 15 ms; ";
    end
    if max(wheelAge, [], "omitnan") > 30.0
        rejectReason = rejectReason + "wheel feedback age > 30 ms; ";
    end

    stepStart = find(diff(turnTgt) > 3.0, 1, 'first');
    if isempty(stepStart)
        stepStart = find(turnTgt > 5.0, 1, 'first');
    end
    if ~isempty(stepStart) && stepStart > 5 && stepStart < height(active) - 20
        stepStart = stepStart - 3;
        stepEnd = min(stepStart + 800, height(active));
        stepT = t(stepStart:stepEnd) - t(stepStart);
        stepGyro = gyroZ(stepStart:stepEnd);
        stepTgt  = turnTgt(stepStart:stepEnd);
        stepTrpm = turnRpm(stepStart:stepEnd);

        stepErr = turnErr(stepStart:stepEnd);
        stepInt = turnInt(stepStart:stepEnd);
        validSign = (abs(stepTgt) > 1.0) & isfinite(stepGyro);
        if any(validSign)
            turnSameSignPct = 100.0 * mean((stepTgt(validSign) .* stepGyro(validSign)) > 0);
        else
            turnSameSignPct = NaN;
        end
        turnSatPct = 100.0 * mean(abs(stepTrpm) >= 0.98 * 60.0, "omitnan");
        turnErrMean = mean(stepTgt - stepGyro, "omitnan");
        turnErrP95 = prctile(abs(stepTgt - stepGyro), 95);
        tailStart = max(1, numel(stepTgt) - 50);
        turnTargetFinal = median(stepTgt(tailStart:end), "omitnan");
        turnGyroFinal = median(stepGyro(tailStart:end), "omitnan");

        if strlength(rejectReason) == 0
            if turnSameSignPct < 60.0
                rejectReason = rejectReason + "turn sign agreement < 60%; ";
            end
            if turnSatPct > 20.0
                rejectReason = rejectReason + "turn output saturation > 20%; ";
            end
        end

        if strlength(rejectReason) == 0
            suggestedTurnKp = min(2.0, max(0.05, median(abs(stepTrpm), "omitnan") / max(turnErrP95, 1.0)));
            suggestedTurnKi = 0.0;
            if abs(turnErrMean) > 0.30 * max(abs(turnTargetFinal), 1.0)
                suggestedTurnKi = min(0.20, max(0.01, suggestedTurnKp * 0.10));
            end
        else
            suggestedTurnKp = NaN;
            suggestedTurnKi = NaN;
        end

        turnReportRow = table(note, string(file.name), zeroGyroMean, zeroGyroStd, ...
            turnTargetFinal, turnGyroFinal, turnErrMean, turnErrP95, ...
            turnSameSignPct, turnSatPct, suggestedTurnKp, suggestedTurnKi, rejectReason, ...
            'VariableNames', ["note", "file", "zero_gyro_mean", "zero_gyro_std", ...
            "turn_target_dps", "turn_gyro_dps", "turn_err_mean", "turn_err_p95", ...
            "same_sign_pct", "turn_sat_pct", "suggested_turn_kp", "suggested_turn_ki", "reject_reason"]);
        if ~exist("turnReport", "var")
            turnReport = turnReportRow;
        else
            turnReport = [turnReport; turnReportRow]; %#ok<AGROW>
        end

        allTurnSteps{end+1} = struct('note', note, 't', stepT, ...
            'gyro', stepGyro, 'tgt', stepTgt, 'trpm', stepTrpm, ...
            'err', stepErr, 'int', stepInt); %#ok<AGROW>
    end
end

%% --- Plot speed steps ---
if ~isempty(allSpeedSteps)
    figure("Name", "Speed step response", "Color", "w", "Position", [100 100 1000 500]);
    tiledlayout(2, 1, "TileSpacing", "compact");

    nexttile; hold on; grid on;
    for i = 1:numel(allSpeedSteps)
        s = allSpeedSteps{i};
        stairs(s.t, s.tgt, "--", "DisplayName", sprintf("%s target", s.note));
        plot(s.t, s.fwd, "LineWidth", 1.5, "DisplayName", s.note);
    end
    ylabel("Wheel speed (RPM)");
    title("Speed step: target vs actual");
    legend("Location", "best", "Interpreter", "none");

    nexttile; hold on; grid on;
    for i = 1:numel(allSpeedSteps)
        s = allSpeedSteps{i};
        plot(s.t, s.off, "LineWidth", 1.5, "DisplayName", s.note);
    end
    ylabel("Pitch offset (deg)");
    xlabel("Time from step (s)");
    title("Speed-loop pitch offset");
    legend("Location", "best", "Interpreter", "none");

    saveas(gcf, fullfile(dataDir, "drive_speed_step.png"));
end

%% --- Plot turn steps ---
if ~isempty(allTurnSteps)
    figure("Name", "Turn step response", "Color", "w", "Position", [100 100 1000 700]);
    tiledlayout(3, 1, "TileSpacing", "compact");

    nexttile; hold on; grid on;
    for i = 1:numel(allTurnSteps)
        s = allTurnSteps{i};
        stairs(s.t, s.tgt, "--", "DisplayName", sprintf("%s target", s.note));
        plot(s.t, s.gyro, "LineWidth", 1.5, "DisplayName", s.note);
    end
    ylabel("Yaw rate (deg/s)");
    title("Turn step: target vs actual");
    legend("Location", "best", "Interpreter", "none");

    nexttile; hold on; grid on;
    for i = 1:numel(allTurnSteps)
        s = allTurnSteps{i};
        plot(s.t, s.trpm, "LineWidth", 1.5, "DisplayName", s.note);
    end
    ylabel("Turn RPM differential");
    title("Turn-loop output");
    legend("Location", "best", "Interpreter", "none");

    nexttile; hold on; grid on;
    for i = 1:numel(allTurnSteps)
        s = allTurnSteps{i};
        plot(s.t, s.err, "LineWidth", 1.2, "DisplayName", s.note);
    end
    ylabel("Turn error (deg/s)");
    xlabel("Time from step (s)");
    title("Turn-loop error");
    legend("Location", "best", "Interpreter", "none");

    saveas(gcf, fullfile(dataDir, "drive_turn_step.png"));
end

%% --- Gain recommendations ---
if ~isempty(report)
    writetable(report, fullfile(dataDir, "drive_tune_report.csv"));

    fprintf("\n========================================\n");
    fprintf("        Gain Recommendations\n");
    fprintf("========================================\n");

    for i = 1:height(report)
        r = report(i, :);
        fprintf("\n[%s]\n", r.note);

        % Speed loop: P-only heuristic
        % Desired: steady error < 30% of target, rise time < 1s
        if ~isnan(r.rpm_per_deg) && r.rpm_per_deg > 0
            % kp * error_max = pitch_limit
            % With target error = 0.3 * target:
            desiredKp = 4.0 / (0.3 * r.target_rpm);  % 4 deg offset for 30% error band
            desiredKp = max(0.05, min(1.0, desiredKp));

            fprintf("  Speed loop:\n");
            fprintf("    rpm/deg sensitivity: %.1f\n", r.rpm_per_deg);
            fprintf("    Recommended kp: %.3f  (for ~%.0f RPM with %.1f deg offset)\n", ...
                desiredKp, desiredKp * 0.3 * r.target_rpm * r.rpm_per_deg / desiredKp, 4.0);
            fprintf("    Recommended ki: 0.0  (start P-only, add ki only if steady error persists)\n");
            fprintf("    BD,%.3f,0.000,<turn_kp>\n", desiredKp);
        else
            fprintf("  Speed loop: insufficient data\n");
        end
    end
else
    fprintf("\nNo speed step data found. Collect a capture with C,<speed>,0 first.\n");
end

if exist("turnReport", "var") && height(turnReport) > 0
    writetable(turnReport, fullfile(dataDir, "drive_turn_tune_report.csv"));
    fprintf("\n========================================\n");
    fprintf("        Turn Gain Recommendations\n");
    fprintf("========================================\n");
    for i = 1:height(turnReport)
        r = turnReport(i, :);
        fprintf("\n[%s]\n", r.note);
        if strlength(string(r.reject_reason)) > 0
            fprintf("  Rejected: %s\n", string(r.reject_reason));
        else
            fprintf("  zero gyro mean/std: %.2f / %.2f dps\n", r.zero_gyro_mean, r.zero_gyro_std);
            fprintf("  final target/gyro: %.2f / %.2f dps\n", r.turn_target_dps, r.turn_gyro_dps);
            fprintf("  same-sign: %.1f %%  saturation: %.1f %%\n", r.same_sign_pct, r.turn_sat_pct);
            fprintf("  Recommended: BT,%.3f,%.3f\n", r.suggested_turn_kp, r.suggested_turn_ki);
        end
    end
end

fprintf("\nSaved: %s, %s\n", fullfile(dataDir, "drive_tune_report.csv"), ...
    fullfile(dataDir, "drive_speed_step.png"));
fprintf("Open in MATLAB: open tools/tune_drive_loops.m\n");
