% Balance Fast Blend Tuning & Analysis
%
% Run from repository root in MATLAB:
%   tools/tune_drive_loops_fast
%
% Inputs:
%   data/balance_capture_*.csv  (38-channel telemetry with fast blend)
%
% Outputs:
%   data/fast_tune_report.csv        — speed-step metrics per capture
%   data/fast_term_budget.csv        — RPM budget breakdown at peak speed
%   data/fast_ceiling_analysis.csv   — theoretical vs actual speed ceiling
%   data/fast_speed_step.png         — speed step overlay with fast_blend
%   data/fast_blend_transition.png   — Ks / pitch_limit / blend ramp
%   data/fast_term_stack.png         — term contributions stacked
%   data/fast_rpm_budget.png         — budget utilization vs target

clear; clc;

dataDir = fullfile("data");
files = dir(fullfile(dataDir, "balance_capture_*.csv"));
if isempty(files)
    error("No balance capture CSV files found under data/.");
end

% --- Deduplicate by note, keep newest per note ---
notes = strings(numel(files), 1);
for i = 1:numel(files)
    try
        one = readtable(fullfile(dataDir, files(i).name));
        if ismember("note", string(one.Properties.VariableNames)) && height(one) > 0
            notes(i) = string(one.note(1));
        end
    catch
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

% --- Config (mirrors app_config.h defaults) ---
cfg.pitch_kp       = 18.0;
cfg.pitch_rate_kd  = 8.0;
cfg.low_ks         = 3.0;
cfg.fast_ks        = 0.8;
cfg.low_pitch_lim  = 8.0;
cfg.fast_pitch_lim = 12.0;
cfg.blend_start    = 40.0;
cfg.blend_full     = 90.0;
cfg.balance_limit  = 300.0;

% --- Output tables ---
speedReport = table();
budgetReport = table();
ceilingReport = table();
turnReport = table();

allSteps   = {};  % speed step structs for plotting
fastCaps   = {};  % B,3 captures for blend plotting

for fileIdx = 1:numel(keepFiles)
    file = keepFiles(fileIdx);
    raw = readtable(fullfile(dataDir, file.name));
    note = string(raw.note(1));
    if strlength(note) == 0, note = string(file.name); end

    % Detect V3 (fast blend) columns
    hasFast = all(ismember(["fast_blend", "wheel_speed_ks", "pitch_term_rpm", ...
        "rate_term_rpm", "speed_term_rpm", "ff_term_rpm"], ...
        string(raw.Properties.VariableNames)));

    % Use B,2 and B,3 as active modes
    active = raw((abs(raw.balance_mode - 2.0) < 0.01) | (abs(raw.balance_mode - 3.0) < 0.01), :);
    active = active(active.feedback_online >= 0.5, :);
    if height(active) < 50
        fprintf("  Skipping %s: too few active samples.\n", note);
        continue;
    end

    isFast = (max(active.balance_mode) > 2.5);  % B,3 has mode=3
    t = active.elapsed_s - active.elapsed_s(1);

    %% ===== Safety gates =====
    imuMax  = max(active.imu_age_ms, [], "omitnan");
    whlMax  = max(active.wheel_age_ms, [], "omitnan");
    pitchRange = [min(active.pitch_deg), max(active.pitch_deg)];
    balSat = sum(abs(active.balance_rpm) >= 0.98 * cfg.balance_limit);
    balSatPct = 100.0 * balSat / height(active);

    fprintf("\n===== %s =====\n", note);
    if isFast, modeStr = "B,3 (fast)"; else, modeStr = "B,2 (low)"; end
    fprintf("  Mode: %s  Samples: %d\n", modeStr, height(active));
    fprintf("  imu_age max: %.0f ms  wheel_age max: %.0f ms\n", imuMax, whlMax);
    fprintf("  pitch: [%.2f, %.2f] deg\n", pitchRange(1), pitchRange(2));
    fprintf("  balance_rpm sat@%.0f: %.1f%% (%d samples)\n", cfg.balance_limit, balSatPct, balSat);

    %% ===== Speed step analysis =====
    fwdTgt   = active.forward_target_rpm;
    fwdAct   = active.forward_actual_rpm;
    pitchOff = active.speed_pitch_offset_deg;
    balRpm   = active.balance_rpm;

    stepStart = find(diff(fwdTgt) > 5.0, 1, 'first');
    if isempty(stepStart)
        stepStart = find(fwdTgt > 5.0, 1, 'first');
    end

    if ~isempty(stepStart) && stepStart > 5 && stepStart < height(active) - 20
        stepStart = stepStart - 3;
        stepEnd = min(stepStart + 800, height(active));
        sT    = t(stepStart:stepEnd) - t(stepStart);
        sFwd  = fwdAct(stepStart:stepEnd);
        sTgt  = fwdTgt(stepStart:stepEnd);
        sOff  = pitchOff(stepStart:stepEnd);
        sBal  = balRpm(stepStart:stepEnd);

        tail = max(1, numel(sTgt) - 50):numel(sTgt);
        finalTgt = median(sTgt(tail));
        finalAct = median(sFwd(tail));
        steadyErr = finalTgt - finalAct;
        peakAct = max(sFwd);
        stepAmp = peakAct - sFwd(1);

        if stepAmp > 2.0
            lo = sFwd(1) + 0.10 * stepAmp;
            hi = sFwd(1) + 0.90 * stepAmp;
            t10 = sT(find(sFwd >= lo, 1, 'first'));
            t90 = sT(find(sFwd >= hi, 1, 'first'));
            if ~isempty(t10) && ~isempty(t90)
                riseTime = t90 - t10;
            else
                riseTime = NaN;
            end
            overshootPct = 100.0 * (peakAct - finalAct) / max(finalAct, 0.01);
            offDelta = median(sOff(tail)) - sOff(1);
            if abs(offDelta) > 0.1
                rpmPerDeg = finalAct / offDelta;
            else
                rpmPerDeg = NaN;
            end

            fprintf("  --- Speed step ---\n");
            fprintf("    target: %.1f  actual: %.1f  steady-err: %.1f RPM\n", finalTgt, finalAct, steadyErr);
            fprintf("    peak: %.1f  overshoot: %.1f%%  rise(10-90): %.2f s\n", peakAct, overshootPct, riseTime);
            fprintf("    pitch-offset delta: %.2f deg  rpm/deg: %.1f\n", offDelta, rpmPerDeg);
        else
            riseTime = NaN; overshootPct = NaN; rpmPerDeg = NaN; peakAct = NaN;
            fprintf("  --- Speed step: amplitude too small (%.1f RPM) ---\n", stepAmp);
        end

        % Save step struct for plotting
        step.t = sT; step.fwd = sFwd; step.tgt = sTgt; step.off = sOff; step.bal = sBal;
        if hasFast
            step.blend = active.fast_blend(stepStart:stepEnd);
            step.ks    = active.wheel_speed_ks(stepStart:stepEnd);
            step.plim  = active.speed_pitch_limit_deg(stepStart:stepEnd);
            step.pterm = active.pitch_term_rpm(stepStart:stepEnd);
            step.rterm = active.rate_term_rpm(stepStart:stepEnd);
            step.sterm = active.speed_term_rpm(stepStart:stepEnd);
            step.ffterm = active.ff_term_rpm(stepStart:stepEnd);
        end
        allSteps{end+1} = struct('note', note, 'step', step, 'isFast', isFast); %#ok<AGROW>

        speedReport = [speedReport; table(note, string(file.name), isFast, finalTgt, finalAct, ...
            steadyErr, peakAct, riseTime, overshootPct, rpmPerDeg, offDelta, balSatPct, ...
            'VariableNames', ["note", "file", "is_fast", "target_rpm", "actual_rpm", ...
            "steady_err_rpm", "peak_rpm", "rise_time_s", "overshoot_pct", "rpm_per_deg", ...
            "pitch_offset_delta_deg", "balance_sat_pct"])]; %#ok<AGROW>
    else
        fprintf("  No speed step detected.\n");
    end

    %% ===== Fast blend analysis (V3 captures only) =====
    if hasFast && isFast
        fastCaps{end+1} = struct('note', note, 'active', active, 't', t); %#ok<AGROW>

        blend   = active.fast_blend;
        ks      = active.wheel_speed_ks;
        plim    = active.speed_pitch_limit_deg;
        ffRpm   = active.speed_ff_rpm;
        pTerm   = active.pitch_term_rpm;
        rTerm   = active.rate_term_rpm;
        sTerm   = active.speed_term_rpm;
        posTerm = active.pos_term_rpm;
        ffTerm  = active.ff_term_rpm;

        % Find peak actual speed
        [peakSpd, peakIdx] = max(abs(fwdAct));
        if peakIdx > 0 && peakIdx <= height(active)
            p = peakIdx(1);
            effKs   = ks(p);
            effPlim = plim(p);
            effBlend = blend(p);

            % Theoretical speed ceiling at this blend state
            theoCeiling = (cfg.pitch_kp / max(effKs, 0.01)) * effPlim;

            % Term breakdown at peak
            pt = pTerm(p); rt = rTerm(p); st = sTerm(p);
            pot = posTerm(p); ft = ffTerm(p); bal = pt + rt + st + pot + ft;

            pctPitch = 100 * abs(pt) / max(cfg.balance_limit, 0.01);
            pctRate  = 100 * abs(rt) / max(cfg.balance_limit, 0.01);
            pctSpeed = 100 * abs(st) / max(cfg.balance_limit, 0.01);
            pctFF    = 100 * abs(ft) / max(cfg.balance_limit, 0.01);
            budgetUsed = 100 * abs(bal) / max(cfg.balance_limit, 0.01);

            fprintf("  --- Blend state at peak (%.0f RPM) ---\n", peakSpd);
            fprintf("    fast_blend: %.3f  eff_Ks: %.2f  pitch_limit: %.1f deg\n", effBlend, effKs, effPlim);
            fprintf("    theo ceiling: %.0f RPM  actual: %.0f RPM\n", theoCeiling, peakSpd);
            fprintf("  --- RPM budget at peak ---\n");
            fprintf("    pitch: %+.1f (%5.1f%%)  rate: %+.1f (%5.1f%%)\n", pt, pctPitch, rt, pctRate);
            fprintf("    speed: %+.1f (%5.1f%%)  pos:  %+.1f         ff: %+.1f (%5.1f%%)\n", st, pctSpeed, pot, ft, pctFF);
            fprintf("    total: %+.1f (%5.1f%% of %.0f limit)\n", bal, budgetUsed, cfg.balance_limit);

            budgetReport = [budgetReport; table(note, peakSpd, effBlend, effKs, effPlim, ...
                theoCeiling, pt, rt, st, ft, bal, budgetUsed, ...
                'VariableNames', ["note", "peak_rpm", "fast_blend", "eff_ks", "eff_pitch_limit_deg", ...
                "theo_ceiling_rpm", "pitch_term", "rate_term", "speed_term", "ff_term", ...
                "balance_rpm", "budget_used_pct"])]; %#ok<AGROW>
        end

        % Scan across the speed range for ceiling analysis
        speedBins = [0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 250];
        for bi = 2:numel(speedBins)
            lo = speedBins(bi-1);
            hi = speedBins(bi);
            binRows = active(abs(fwdAct) >= lo & abs(fwdAct) < hi, :);
            if height(binRows) >= 5
                avgSpd = mean(abs(binRows.forward_actual_rpm));
                avgKs  = mean(binRows.wheel_speed_ks);
                avgPlim = mean(binRows.speed_pitch_limit_deg);
                avgSTerm = mean(abs(binRows.speed_term_rpm));
                avgBal  = mean(abs(binRows.balance_rpm));
                ceiling = (cfg.pitch_kp / max(avgKs, 0.01)) * avgPlim;
                ceilingReport = [ceilingReport; table(note, avgSpd, avgKs, avgPlim, ...
                    ceiling, avgSTerm, avgBal, ...
                    'VariableNames', ["note", "speed_rpm", "eff_ks", "eff_pitch_limit_deg", ...
                    "theo_ceiling_rpm", "speed_term_rpm", "balance_rpm"])]; %#ok<AGROW>
            end
        end
    end

    %% ===== Turn loop analysis =====
    turnTgt = active.turn_target_dps;
    gyroZ   = active.gyro_z_dps;
    turnRpm = active.turn_rpm;
    imuAge  = active.imu_age_ms;
    wheelAge = active.wheel_age_ms;

    zeroTurn = active(abs(active.turn_target_dps) < 0.5, :);
    reject = "";
    zgMean = NaN; zgStd = NaN;

    if height(zeroTurn) >= 20
        zgMean = mean(zeroTurn.gyro_z_dps, "omitnan");
        zgStd  = std(zeroTurn.gyro_z_dps, "omitnan");
        if abs(zgMean) > 3.0,  reject = reject + "gyro bias>3; "; end
        if zgStd > 8.0,       reject = reject + "gyro std>8; "; end
    else
        reject = reject + "zero-turn samples<20; ";
    end

    if max(imuAge, [], "omitnan") > 15.0,   reject = reject + "imu age>15ms; "; end
    if max(wheelAge, [], "omitnan") > 30.0, reject = reject + "wheel age>30ms; "; end

    % Turn step metrics
    tsStart = find(diff(turnTgt) > 3.0, 1, 'first');
    if isempty(tsStart), tsStart = find(turnTgt > 5.0, 1, 'first'); end

    sameSignPct = NaN; satPct = NaN; finalTurnTgt = NaN; finalTurnGyro = NaN;
    if ~isempty(tsStart) && tsStart > 5 && tsStart < height(active) - 20
        tsStart = tsStart - 3;
        tsEnd = min(tsStart + 800, height(active));
        stTgt  = turnTgt(tsStart:tsEnd);
        stGyro = gyroZ(tsStart:tsEnd);
        stRpm  = turnRpm(tsStart:tsEnd);

        valid = (abs(stTgt) > 1.0) & isfinite(stGyro);
        if any(valid)
            sameSignPct = 100.0 * mean((stTgt(valid) .* stGyro(valid)) > 0);
        end
        satPct = 100.0 * mean(abs(stRpm) >= 0.98 * 60.0, "omitnan");

        tl = max(1, numel(stTgt)-50):numel(stTgt);
        finalTurnTgt  = median(stTgt(tl), "omitnan");
        finalTurnGyro = median(stGyro(tl), "omitnan");

        if strlength(reject) == 0
            if sameSignPct < 60.0, reject = reject + "sign<60%; "; end
            if satPct > 20.0,     reject = reject + "sat>20%; "; end
        end
    end

    fprintf("  --- Turn loop ---\n");
    fprintf("    zero gyro: %.2f / %.2f dps (mean/std)\n", zgMean, zgStd);
    fprintf("    same-sign: %.1f%%  saturation: %.1f%%\n", sameSignPct, satPct);
    if strlength(reject) > 0
        fprintf("    REJECTED: %s\n", reject);
    else
        fprintf("    healthy\n");
    end

    turnReport = [turnReport; table(note, string(file.name), zgMean, zgStd, ...
        finalTurnTgt, finalTurnGyro, sameSignPct, satPct, reject, ...
        'VariableNames', ["note", "file", "zero_gyro_mean", "zero_gyro_std", ...
        "turn_target_final", "turn_gyro_final", "same_sign_pct", "turn_sat_pct", "reject_reason"])]; %#ok<AGROW>
end

%% ================================================================
%  PLOTS
%% ================================================================

%% --- Plot 1: Speed step response with fast_blend ---
if ~isempty(allSteps)
    figure("Name", "Speed step + blend", "Color", "w", "Position", [50 50 1200 800]);
    tcl = tiledlayout(3, 1, "TileSpacing", "compact");

    nexttile; hold on; grid on;
    colors = lines(numel(allSteps));
    for i = 1:numel(allSteps)
        s = allSteps{i}.step; c = colors(i,:);
        stairs(s.t, s.tgt, "--", "Color", c*0.6, "DisplayName", sprintf("%s tgt", allSteps{i}.note));
        plot(s.t, s.fwd, "LineWidth", 1.5, "Color", c, "DisplayName", allSteps{i}.note);
    end
    ylabel("Wheel speed (RPM)"); title("Speed step: target vs actual");
    legend("Location", "best", "Interpreter", "none");

    nexttile; hold on; grid on;
    for i = 1:numel(allSteps)
        s = allSteps{i}.step; c = colors(i,:);
        plot(s.t, s.off, "LineWidth", 1.5, "Color", c, "DisplayName", allSteps{i}.note);
    end
    ylabel("Pitch offset (deg)"); title("Speed-loop pitch offset");
    legend("Location", "best", "Interpreter", "none");

    nexttile; hold on; grid on;
    hasBlend = false;
    for i = 1:numel(allSteps)
        if allSteps{i}.isFast && isfield(allSteps{i}.step, 'blend')
            hasBlend = true;
            s = allSteps{i}.step; c = colors(i,:);
            plot(s.t, s.blend, "LineWidth", 1.5, "Color", c, "DisplayName", allSteps{i}.note);
        end
    end
    if hasBlend
        ylabel("fast_blend"); xlabel("Time from step (s)");
        title("Fast blend ramp");
        ylim([-0.05, 1.05]);
        legend("Location", "best", "Interpreter", "none");
    else
        title("No fast blend data (B,2 only)");
    end

    saveas(gcf, fullfile(dataDir, "fast_speed_step.png"));
    fprintf("\nSaved: %s\n", fullfile(dataDir, "fast_speed_step.png"));
end

%% --- Plot 2: Blend transition detail (Ks / pitch_limit vs speed) ---
if ~isempty(fastCaps)
    figure("Name", "Blend transition", "Color", "w", "Position", [50 50 1200 500]);
    tcl = tiledlayout(1, 3, "TileSpacing", "compact");

    nexttile; hold on; grid on;
    nexttile; hold on; grid on;
    nexttile; hold on; grid on;

    % Collect all B,3 captures in one plot
    ax1 = nexttile(1); hold on; grid on;
    ax2 = nexttile(2); hold on; grid on;
    ax3 = nexttile(3); hold on; grid on;

    colors = lines(numel(fastCaps));
    for i = 1:numel(fastCaps)
        fc = fastCaps{i};
        spd = abs(fc.active.forward_actual_rpm);

        plot(ax1, spd, fc.active.fast_blend, ".", "Color", colors(i,:), "DisplayName", fc.note);
        plot(ax2, spd, fc.active.wheel_speed_ks, ".", "Color", colors(i,:), "DisplayName", fc.note);
        plot(ax3, spd, fc.active.speed_pitch_limit_deg, ".", "Color", colors(i,:), "DisplayName", fc.note);
    end

    % Theoretical curves
    spdRef = linspace(0, 250, 500);
    tRef = min(1, max(0, (spdRef - cfg.blend_start) ./ (cfg.blend_full - cfg.blend_start)));
    blendRef = tRef.^2 .* (3 - 2*tRef);
    ksRef  = cfg.low_ks + (cfg.fast_ks - cfg.low_ks) .* blendRef;
    plimRef = cfg.low_pitch_lim + (cfg.fast_pitch_lim - cfg.low_pitch_lim) .* blendRef;

    plot(ax1, spdRef, blendRef, "k-", "LineWidth", 1.5, "DisplayName", "smoothstep(40,90)");
    xlabel(ax1, "|wheel speed| (RPM)"); ylabel(ax1, "fast_blend"); title(ax1, "Blend factor");
    legend(ax1, "Location", "best", "Interpreter", "none");

    plot(ax2, spdRef, ksRef, "k-", "LineWidth", 1.5, "DisplayName", "lerp(3.0,0.8)");
    xlabel(ax2, "|wheel speed| (RPM)"); ylabel(ax2, "eff wheel_speed_ks"); title(ax2, "Speed damping");
    legend(ax2, "Location", "best", "Interpreter", "none");

    plot(ax3, spdRef, plimRef, "k-", "LineWidth", 1.5, "DisplayName", "lerp(8,12)");
    xlabel(ax3, "|wheel speed| (RPM)"); ylabel(ax3, "pitch_limit (deg)"); title(ax3, "Pitch offset limit");
    legend(ax3, "Location", "best", "Interpreter", "none");

    saveas(gcf, fullfile(dataDir, "fast_blend_transition.png"));
    fprintf("Saved: %s\n", fullfile(dataDir, "fast_blend_transition.png"));
end

%% --- Plot 3: Term contributions at peak speed ---
if ~isempty(budgetReport)
    figure("Name", "RPM budget at peak speed", "Color", "w", "Position", [50 50 1000 600]);
    tcl = tiledlayout(1, 2, "TileSpacing", "compact");

    % Bar chart: term breakdown
    nexttile; hold on; grid on;
    notes = string(budgetReport.note);
    pitchVals = budgetReport.pitch_term;
    rateVals  = budgetReport.rate_term;
    speedVals = budgetReport.speed_term;
    ffVals    = budgetReport.ff_term;
    barData = [pitchVals, rateVals, speedVals, ffVals];
    bh = bar(barData);
    bh(1).FaceColor = [0.2 0.6 0.8];  % pitch
    bh(2).FaceColor = [0.8 0.4 0.2];  % rate
    bh(3).FaceColor = [0.4 0.8 0.2];  % speed
    bh(4).FaceColor = [0.6 0.4 0.8];  % ff
    set(gca, "XTickLabel", notes);
    ylabel("RPM"); title("Term contributions at peak speed");
    legend("pitch", "rate", "speed", "ff", "Location", "best");
    yline(cfg.balance_limit, "r--", "LineWidth", 1.2);
    yline(-cfg.balance_limit, "r--", "LineWidth", 1.2);

    % Budget utilization %
    nexttile; hold on; grid on;
    budgetPct = budgetReport.budget_used_pct;
    barColors = zeros(numel(budgetPct), 3);
    for i = 1:numel(budgetPct)
        if budgetPct(i) > 80
            barColors(i,:) = [0.9 0.2 0.2];
        elseif budgetPct(i) > 50
            barColors(i,:) = [0.9 0.7 0.2];
        else
            barColors(i,:) = [0.3 0.7 0.3];
        end
    end
    for i = 1:numel(budgetPct)
        bar(i, budgetPct(i), "FaceColor", barColors(i,:));
    end
    set(gca, "XTick", 1:numel(notes), "XTickLabel", notes);
    ylabel("% of 300 RPM limit"); title("RPM budget utilization");
    yline(100, "r--", "LineWidth", 1.5);

    saveas(gcf, fullfile(dataDir, "fast_term_stack.png"));
    fprintf("Saved: %s\n", fullfile(dataDir, "fast_term_stack.png"));
end

%% --- Plot 4: Speed ceiling scan ---
if ~isempty(ceilingReport)
    figure("Name", "Speed ceiling", "Color", "w", "Position", [50 50 900 500]);
    hold on; grid on;
    uniqueCeilNotes = unique(string(ceilingReport.note), "stable");
    colors = lines(numel(uniqueCeilNotes));
    for i = 1:numel(uniqueCeilNotes)
        cr = ceilingReport(string(ceilingReport.note) == uniqueCeilNotes(i), :);
        plot(cr.speed_rpm, cr.theo_ceiling_rpm, "o-", "Color", colors(i,:), ...
            "LineWidth", 1.5, "DisplayName", uniqueCeilNotes(i));
    end
    plot([0 250], [0 250], "k:", "DisplayName", "1:1 (actual=ceiling)");
    xlabel("Actual wheel speed (RPM)"); ylabel("Theoretical ceiling (RPM)");
    title("Speed ceiling: theory vs actual");
    legend("Location", "best", "Interpreter", "none");

    saveas(gcf, fullfile(dataDir, "fast_ceiling_analysis.png"));
    fprintf("Saved: %s\n", fullfile(dataDir, "fast_ceiling_analysis.png"));
end

%% ================================================================
%  RECOMMENDATIONS
%% ================================================================
fprintf("\n========================================\n");
fprintf("        Recommendations\n");
fprintf("========================================\n");

if ~isempty(budgetReport)
    % Group: find the highest target with <50% budget usage
    goodSpeeds = budgetReport(budgetReport.budget_used_pct < 50, :);
    badSpeeds  = budgetReport(budgetReport.budget_used_pct >= 80, :);

    if ~isempty(goodSpeeds)
        [~, idx] = max(goodSpeeds.peak_rpm);
        fprintf("\n  Safe max speed: %.0f RPM (%s, %.0f%% budget)\n", ...
            goodSpeeds.peak_rpm(idx), goodSpeeds.note(idx), goodSpeeds.budget_used_pct(idx));
    end
    if ~isempty(badSpeeds)
        fprintf("  Budget saturation at:");
        for i = 1:height(badSpeeds)
            fprintf(" %s(%.0f%%),", badSpeeds.note(i), badSpeeds.budget_used_pct(i));
        end
        fprintf("\n");
    end

    % Ks recommendation: target speed_term < 40% of budget
    worst = budgetReport(budgetReport.budget_used_pct == max(budgetReport.budget_used_pct), :);
    if ~isempty(worst) && worst.budget_used_pct(1) > 60
        targetBudget = 40;  % want speed_term under 40% of limit
        maxSpeedTerm = (targetBudget / 100) * cfg.balance_limit;
        for i = 1:height(worst)
            suggestedKs = maxSpeedTerm / max(worst.peak_rpm(i), 1.0);
            fprintf("\n  [%s] Suggested fast_ks: %.2f (currently %.2f)\n", ...
                worst.note(i), suggestedKs, cfg.fast_ks);
            fprintf("    At %.0f RPM: speed_term drops from %.0f to ~%.0f\n", ...
                worst.peak_rpm(i), worst.speed_term(i), suggestedKs * worst.peak_rpm(i));
        end
    end
end

% Turn recommendations
if ~isempty(turnReport)
    fprintf("\n  Turn loop:\n");
    cleanTurns = turnReport(strlength(string(turnReport.reject_reason)) == 0, :);
    if ~isempty(cleanTurns)
        zgStdAvg = mean(cleanTurns.zero_gyro_std, "omitnan");
        fprintf("    avg zero-gyro std: %.2f dps — ", zgStdAvg);
        if zgStdAvg < 4.0,  fprintf("quiet, Kp can go higher\n");
        elseif zgStdAvg < 6.0, fprintf("acceptable\n");
        else, fprintf("noisy, consider lowering Kp\n");
        end
    end
    badTurns = turnReport(strlength(string(turnReport.reject_reason)) > 0, :);
    for i = 1:height(badTurns)
        fprintf("    [%s] rejected: %s\n", badTurns.note(i), badTurns.reject_reason(i));
    end
end

%% ================================================================
%  SAVE CSV REPORTS
%% ================================================================
if ~isempty(speedReport)
    writetable(speedReport, fullfile(dataDir, "fast_tune_report.csv"));
end
if ~isempty(budgetReport)
    writetable(budgetReport, fullfile(dataDir, "fast_term_budget.csv"));
end
if ~isempty(ceilingReport)
    writetable(ceilingReport, fullfile(dataDir, "fast_ceiling_analysis.csv"));
end
if ~isempty(turnReport)
    writetable(turnReport, fullfile(dataDir, "fast_turn_tune_report.csv"));
end

fprintf("\nReports saved to %s/\n", fullfile(dataDir));
fprintf("  fast_tune_report.csv       — speed step metrics\n");
fprintf("  fast_term_budget.csv       — RPM budget at peak\n");
fprintf("  fast_ceiling_analysis.csv  — ceiling vs speed scan\n");
fprintf("  fast_turn_tune_report.csv  — turn loop metrics\n");

