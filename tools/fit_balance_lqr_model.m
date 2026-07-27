% Identify the current four-state balance model and export raw LQR candidates.
%
% Inputs:  data/balance_lqr_id_*.csv only
% Outputs: balance_lqr_ab.mat, balance_lqr_model_quality.csv,
%          balance_lqr_candidates.csv, and balance_lqr_fit.png

clear; clc;

dataDir = "data";
files = dir(fullfile(dataDir, "balance_lqr_id_*.csv"));
if isempty(files)
    error("No balance_lqr_id_*.csv files found under data/.");
end

required = ["elapsed_s", "time_ms", "balance_mode", "feedback_online", ...
    "pitch_deg", "pitch_rate_dps", "balance_rpm", "left_motor_rpm", ...
    "right_motor_rpm", "firmware_frame_sequence", "imu_age_ms", ...
    "balance_output_limit_rpm", "last_command"];
captures = struct("name", {}, "profile", {}, "x", {}, "u", {}, ...
    "sequence", {}, "transition", {}, "x0", {}, "x1", {}, "u0", {});
qualityRows = struct("file", {}, "profile", {}, "rows_total", {}, ...
    "rows_healthy", {}, "rows_rejected", {}, "gap_edges_rejected", {}, ...
    "transition_rows", {}, "rejection_reason", {});

for fileIndex = 1:numel(files)
    filePath = fullfile(dataDir, files(fileIndex).name);
    raw = readtable(filePath);
    if ~all(ismember(required, string(raw.Properties.VariableNames)))
        error("%s is missing one or more required balance columns.", files(fileIndex).name);
    end
    tokens = regexp(files(fileIndex).name, "balance_lqr_id_(short|medium|long)_", "tokens", "once");
    if isempty(tokens)
        error("Cannot determine BI profile from %s.", files(fileIndex).name);
    end
    profile = string(tokens{1});

    elapsed_s = double(raw.elapsed_s);
    time_ms = double(raw.time_ms);
    balance_mode = double(raw.balance_mode);
    feedback_online = double(raw.feedback_online);
    pitch_deg = double(raw.pitch_deg);
    pitch_rate_dps = double(raw.pitch_rate_dps);
    balance_rpm = double(raw.balance_rpm);
    left_motor_rpm = double(raw.left_motor_rpm);
    right_motor_rpm = double(raw.right_motor_rpm);
    firmware_frame_sequence = round(double(raw.firmware_frame_sequence));
    imu_age_ms = double(raw.imu_age_ms);
    balance_output_limit_rpm = double(raw.balance_output_limit_rpm);
    last_command = string(raw.last_command); %#ok<NASGU>

    finiteMask = isfinite(elapsed_s) & isfinite(time_ms) & ...
        isfinite(pitch_deg) & isfinite(pitch_rate_dps) & ...
        isfinite(balance_rpm) & isfinite(left_motor_rpm) & ...
        isfinite(right_motor_rpm) & isfinite(firmware_frame_sequence) & ...
        isfinite(imu_age_ms) & isfinite(balance_output_limit_rpm);
    commandLimit = 0.9 .* max(abs(balance_output_limit_rpm), 1.0);
    healthy = (balance_mode == 2) & (feedback_online >= 0.5) & ...
        (imu_age_ms <= 15.0) & (abs(pitch_deg) <= 15.0) & ...
        (abs(balance_rpm) < commandLimit) & finiteMask;
    rowsHealthy = nnz(healthy);
    if rowsHealthy < 8
        error("%s has only %d healthy balance rows.", files(fileIndex).name, rowsHealthy);
    end

    elapsed_s = elapsed_s(healthy);
    time_ms = time_ms(healthy);
    pitch_deg = pitch_deg(healthy);
    pitch_rate_dps = pitch_rate_dps(healthy);
    balance_rpm = balance_rpm(healthy);
    left_motor_rpm = left_motor_rpm(healthy);
    right_motor_rpm = right_motor_rpm(healthy);
    firmware_frame_sequence = firmware_frame_sequence(healthy);
    dt_s = [median(diff(time_ms)) / 1000.0; diff(time_ms) / 1000.0];
    validDt = dt_s > 0.0 & isfinite(dt_s);
    if any(validDt)
        dt_s(~validDt) = median(dt_s(validDt));
    else
        dt_s(:) = 0.01;
    end
    avgMotorRpm = (left_motor_rpm + right_motor_rpm) / 2.0;
    wheel_position_rev = cumsum(avgMotorRpm .* dt_s / 60.0);
    wheel_position_rev = wheel_position_rev - wheel_position_rev(1);
    x = [pitch_deg, pitch_rate_dps, avgMotorRpm, wheel_position_rev]';
    u = balance_rpm';
    transition = diff(firmware_frame_sequence) == 1 & diff(time_ms) > 0.0;
    x0 = x(:, 1:end-1);
    x1 = x(:, 2:end);
    u0 = u(1:end-1);
    x0 = x0(:, transition);
    x1 = x1(:, transition);
    u0 = u0(transition);
    gapEdgesRejected = sum(~transition);

    capture = struct("name", string(files(fileIndex).name), "profile", profile, ...
        "x", x, "u", u, "sequence", firmware_frame_sequence, ...
        "transition", transition, "x0", x0, "x1", x1, "u0", u0);
    captures(end + 1) = capture; %#ok<SAGROW>
    qualityRows(end + 1) = struct("file", string(files(fileIndex).name), ...
        "profile", profile, "rows_total", height(raw), "rows_healthy", rowsHealthy, ...
        "rows_rejected", height(raw) - rowsHealthy, ...
        "gap_edges_rejected", gapEdgesRejected, "transition_rows", size(x0, 2), ...
        "rejection_reason", "unhealthy rows and non-contiguous sequence edges"); %#ok<SAGROW>
end

requiredProfiles = ["short", "medium", "long"];
captureProfiles = string({captures.profile});
if ~all(ismember(requiredProfiles, captureProfiles))
    error("All short, medium, and long BI profiles are required before fitting.");
end

captureCount = numel(captures);
trainCount = max(1, min(captureCount - 1, ceil(captureCount / 2)));
trainIndexes = 1:trainCount;
validationIndexes = (trainCount + 1):captureCount;
trainX0 = []; trainX1 = []; trainU = [];
validationX0 = []; validationX1 = []; validationU = [];
for index = trainIndexes
    trainX0 = [trainX0, captures(index).x0]; %#ok<AGROW>
    trainX1 = [trainX1, captures(index).x1]; %#ok<AGROW>
    trainU = [trainU, captures(index).u0]; %#ok<AGROW>
end
for index = validationIndexes
    validationX0 = [validationX0, captures(index).x0]; %#ok<AGROW>
    validationX1 = [validationX1, captures(index).x1]; %#ok<AGROW>
    validationU = [validationU, captures(index).u0]; %#ok<AGROW>
end
if isempty(trainU) || isempty(validationU)
    error("File-level training and validation splits must both contain transitions.");
end

phi = [trainX0; trainU];
rankPhi = rank(phi);
if rankPhi ~= 5
    error("Balance regressor rank is %d; expected 5.", rankPhi);
end
condition_number = cond(phi * phi');
theta = trainX1 / phi;
A = theta(:, 1:4);
B = theta(:, 5);
trainPrediction = theta * phi;
validationPhi = [validationX0; validationU];
validationPrediction = theta * validationPhi;
train_rmse = state_rmse(trainX1, trainPrediction);
validation_rmse = state_rmse(validationX1, validationPrediction);
open_loop_eigenvalues = eig(A);

multi_step_errors = [];
for index = validationIndexes
    capture = captures(index);
    transition = capture.transition;
    segmentStart = 1;
    for edge = 1:(numel(transition) + 1)
        edgeBreak = edge > numel(transition) || ~transition(edge);
        if edgeBreak
            segmentEnd = edge;
            if segmentEnd - segmentStart >= 2
                xhat = capture.x(:, segmentStart);
                for stateIndex = segmentStart:(segmentEnd - 1)
                    xhat = A * xhat + B * capture.u(stateIndex);
                    multi_step_errors(:, end + 1) = capture.x(:, stateIndex + 1) - xhat; %#ok<AGROW>
                end
            end
            segmentStart = edge + 1;
        end
    end
end
if isempty(multi_step_errors)
    error("No contiguous validation segment is available for multi-step validation.");
end
multi_step_rmse = state_rmse(multi_step_errors, zeros(size(multi_step_errors)));

qPitch = [10, 30, 60];
qRate = [0.5, 2, 10, 40];
qSpeed = [0.005, 0.02, 0.08];
qPosition = [0.2, 1, 4];
rInput = [0.02, 0.05, 0.1];
candidateRows = struct("q_pitch", {}, "q_rate", {}, "q_speed", {}, ...
    "q_position", {}, "r_input", {}, "k_pitch", {}, "k_rate", {}, ...
    "k_speed", {}, "k_position", {}, "spectral_radius", {}, ...
    "dare_iterations", {}, "method", {});
candidateIndex = 0;
for pitchWeight = qPitch
    for rateWeight = qRate
        for speedWeight = qSpeed
            for positionWeight = qPosition
                for inputWeight = rInput
                    Q = diag([pitchWeight, rateWeight, speedWeight, positionWeight]);
                    R = inputWeight;
                    method = "local_dare";
                    dare_iterations = NaN;
                    if exist("dlqr", "file") == 2
                        try
                            [K, ~, ~] = dlqr(A, B, Q, R);
                            method = "dlqr";
                            dare_iterations = 0;
                        catch
                            [K, ~, dare_iterations, converged] = local_dare(A, B, Q, R);
                            if ~converged
                                error("Local DARE did not converge for an LQR candidate.");
                            end
                        end
                    else
                        [K, ~, dare_iterations, converged] = local_dare(A, B, Q, R);
                        if ~converged
                            error("Local DARE did not converge for an LQR candidate.");
                        end
                    end
                    closedLoopEigenvalues = eig(A - B * K);
                    spectralRadius = max(abs(closedLoopEigenvalues));
                    if ~all(isfinite(K)) || ~isfinite(spectralRadius)
                        error("Non-finite LQR candidate encountered.");
                    end
                    candidateIndex = candidateIndex + 1;
                    candidateRows(candidateIndex) = struct("q_pitch", pitchWeight, ...
                        "q_rate", rateWeight, "q_speed", speedWeight, ...
                        "q_position", positionWeight, "r_input", inputWeight, ...
                        "k_pitch", K(1), "k_rate", K(2), "k_speed", K(3), ...
                        "k_position", K(4), "spectral_radius", spectralRadius, ...
                        "dare_iterations", dare_iterations, "method", method); %#ok<SAGROW>
                end
            end
        end
    end
end
lqr_candidates = struct2table(candidateRows);
model_quality = struct2table(qualityRows);
model_quality.rank_phi = repmat(rankPhi, height(model_quality), 1);
model_quality.condition_number = repmat(condition_number, height(model_quality), 1);
model_quality.train_rmse_pitch = repmat(train_rmse(1), height(model_quality), 1);
model_quality.train_rmse_rate = repmat(train_rmse(2), height(model_quality), 1);
model_quality.train_rmse_speed = repmat(train_rmse(3), height(model_quality), 1);
model_quality.train_rmse_position = repmat(train_rmse(4), height(model_quality), 1);
model_quality.validation_rmse_pitch = repmat(validation_rmse(1), height(model_quality), 1);
model_quality.validation_rmse_rate = repmat(validation_rmse(2), height(model_quality), 1);
model_quality.validation_rmse_speed = repmat(validation_rmse(3), height(model_quality), 1);
model_quality.validation_rmse_position = repmat(validation_rmse(4), height(model_quality), 1);
model_quality.multi_step_rmse_pitch = repmat(multi_step_rmse(1), height(model_quality), 1);
model_quality.multi_step_rmse_rate = repmat(multi_step_rmse(2), height(model_quality), 1);
model_quality.multi_step_rmse_speed = repmat(multi_step_rmse(3), height(model_quality), 1);
model_quality.multi_step_rmse_position = repmat(multi_step_rmse(4), height(model_quality), 1);
for eigenIndex = 1:4
    model_quality.(sprintf("A_eigen_%d_real", eigenIndex)) = repmat(real(open_loop_eigenvalues(eigenIndex)), height(model_quality), 1);
    model_quality.(sprintf("A_eigen_%d_imag", eigenIndex)) = repmat(imag(open_loop_eigenvalues(eigenIndex)), height(model_quality), 1);
end

save(fullfile(dataDir, "balance_lqr_ab.mat"), "A", "B", "train_rmse", ...
    "validation_rmse", "multi_step_rmse", "open_loop_eigenvalues", ...
    "condition_number", "rankPhi", "model_quality");
writetable(model_quality, fullfile(dataDir, "balance_lqr_model_quality.csv"));
writetable(lqr_candidates, fullfile(dataDir, "balance_lqr_candidates.csv"));

figure("Name", "Balance A B model fit", "Color", "w");
tiledlayout(4, 1, "TileSpacing", "compact");
labels = ["pitch deg", "pitch rate dps", "wheel rpm", "wheel position rev"];
for stateIndex = 1:4
    nexttile;
    plot(validationX1(stateIndex, 1:min(2000, size(validationX1, 2))), "DisplayName", "measured"); hold on; grid on;
    plot(validationPrediction(stateIndex, 1:min(2000, size(validationPrediction, 2))), "DisplayName", "one-step fit");
    ylabel(labels(stateIndex));
    if stateIndex == 1, legend("Location", "best"); end
end
xlabel("validation sample");
saveas(gcf, fullfile(dataDir, "balance_lqr_fit.png"));

fprintf("Balance model written from %d capture(s). Rank=%d, condition=%.3g.\n", ...
    captureCount, rankPhi, condition_number);
disp("A ="); disp(A);
disp("B ="); disp(B);
disp("One-step validation RMSE ="); disp(validation_rmse);
disp("Multi-step validation RMSE ="); disp(multi_step_rmse);

function values = state_rmse(actual, predicted)
    values = sqrt(mean((actual - predicted) .^ 2, 2));
end

function [K, P, iterations, converged] = local_dare(A, B, Q, R)
    P = Q;
    converged = false;
    K = zeros(1, size(A, 1));
    for iterations = 1:1000
        denominator = R + B' * P * B;
        K = denominator \ (B' * P * A);
        nextP = A' * P * A - A' * P * B * K + Q;
        if norm(nextP - P, "fro") < 1.0e-9
            P = nextP;
            converged = true;
            return;
        end
        P = nextP;
    end
end
