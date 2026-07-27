% Fit signed first-order BLDC actuator models from current hardware captures.
%
% Inputs:  data/motor_ident_*.csv
% Outputs: data/motor_actuator_model.csv, .mat, and _fit.png

clear; clc;

dataDir = "data";
files = dir(fullfile(dataDir, "motor_ident_*.csv"));
if isempty(files)
    error("No motor identification CSV files found under data/.");
end

required = ["elapsed_s", "expected_mode", "expected_open_duty", ...
    "feedback_online", "left_motor_rpm", "right_motor_rpm", ...
    "left_duty", "right_duty"];
captures = struct("name", {}, "dt_s", {}, "left", {}, "right", {});

for fileIndex = 1:numel(files)
    filePath = fullfile(dataDir, files(fileIndex).name);
    raw = readtable(filePath);
    if ~all(ismember(required, string(raw.Properties.VariableNames)))
        warning("Skipping %s: required motor columns are missing.", files(fileIndex).name);
        continue;
    end

    mode = string(raw.expected_mode);
    duty = double(raw.expected_open_duty);
    leftDuty = double(raw.left_duty);
    rightDuty = double(raw.right_duty);
    online = double(raw.feedback_online);
    elapsed = double(raw.elapsed_s);
    leftRpm = double(raw.left_motor_rpm);
    rightRpm = double(raw.right_motor_rpm);
    finiteMask = isfinite(duty) & isfinite(leftDuty) & isfinite(rightDuty) & ...
        isfinite(online) & isfinite(elapsed) & isfinite(leftRpm) & isfinite(rightRpm);
    keep = (mode == "duty") & (online >= 0.5) & finiteMask;
    if nnz(keep) < 8
        warning("Skipping %s: only %d usable duty rows.", files(fileIndex).name, nnz(keep));
        continue;
    end

    elapsed = elapsed(keep);
    duty = duty(keep);
    leftRpm = leftRpm(keep);
    rightRpm = rightRpm(keep);
    dt = diff(elapsed);
    dt = dt(isfinite(dt) & dt > 0.0);
    if isempty(dt)
        dt_s = 0.01;
    else
        dt_s = median(dt);
    end
    captures(end + 1).name = string(files(fileIndex).name); %#ok<SAGROW>
    captures(end).dt_s = dt_s;
    captures(end).left = struct("rpm", leftRpm, "duty", duty);
    captures(end).right = struct("rpm", rightRpm, "duty", duty);
end

if isempty(captures)
    error("No usable motor identification captures remain after health filtering.");
end

trainLeft = struct("rpm", [], "duty", [], "dt_s", 0.01);
trainRight = trainLeft;
validationLeft = trainLeft;
validationRight = trainRight;

if numel(captures) >= 2
    trainCount = max(1, floor(numel(captures) / 2));
    trainIndexes = 1:trainCount;
    validationIndexes = (trainCount + 1):numel(captures);
    for index = trainIndexes
        trainLeft.rpm = [trainLeft.rpm; captures(index).left.rpm]; %#ok<AGROW>
        trainLeft.duty = [trainLeft.duty; captures(index).left.duty]; %#ok<AGROW>
        trainLeft.dt_s = captures(index).dt_s;
        trainRight.rpm = [trainRight.rpm; captures(index).right.rpm]; %#ok<AGROW>
        trainRight.duty = [trainRight.duty; captures(index).right.duty]; %#ok<AGROW>
        trainRight.dt_s = captures(index).dt_s;
    end
    for index = validationIndexes
        validationLeft.rpm = [validationLeft.rpm; captures(index).left.rpm]; %#ok<AGROW>
        validationLeft.duty = [validationLeft.duty; captures(index).left.duty]; %#ok<AGROW>
        validationLeft.dt_s = captures(index).dt_s;
        validationRight.rpm = [validationRight.rpm; captures(index).right.rpm]; %#ok<AGROW>
        validationRight.duty = [validationRight.duty; captures(index).right.duty]; %#ok<AGROW>
        validationRight.dt_s = captures(index).dt_s;
    end
else
    one = captures(1);
    count = numel(one.left.rpm);
    cut = max(4, min(count - 3, floor(0.70 * count)));
    trainLeft = struct("rpm", one.left.rpm(1:cut), "duty", one.left.duty(1:cut), "dt_s", one.dt_s);
    trainRight = struct("rpm", one.right.rpm(1:cut), "duty", one.right.duty(1:cut), "dt_s", one.dt_s);
    validationLeft = struct("rpm", one.left.rpm(cut + 1:end), "duty", one.left.duty(cut + 1:end), "dt_s", one.dt_s);
    validationRight = struct("rpm", one.right.rpm(cut + 1:end), "duty", one.right.duty(cut + 1:end), "dt_s", one.dt_s);
end

leftResult = fit_motor_side(trainLeft, validationLeft, "left");
rightResult = fit_motor_side(trainRight, validationRight, "right");

summary = [leftResult.summary; rightResult.summary];
writetable(summary, fullfile(dataDir, "motor_actuator_model.csv"));

model = struct();
model.left = leftResult;
model.right = rightResult;
model.capture_names = string({captures.name})';
save(fullfile(dataDir, "motor_actuator_model.mat"), "model");

figure("Name", "Signed motor actuator fit", "Color", "w");
tiledlayout(2, 1, "TileSpacing", "compact");
nexttile;
plot(leftResult.train.rpm, "DisplayName", "left measured"); hold on; grid on;
plot(leftResult.train.prediction, "DisplayName", "left one-step fit");
ylabel("left RPM"); legend("Location", "best");
nexttile;
plot(rightResult.train.rpm, "DisplayName", "right measured"); hold on; grid on;
plot(rightResult.train.prediction, "DisplayName", "right one-step fit");
ylabel("right RPM"); xlabel("sample"); legend("Location", "best");
saveas(gcf, fullfile(dataDir, "motor_actuator_model_fit.png"));

fprintf("Motor model written for %d capture(s).\n", numel(captures));
disp(summary);

function result = fit_motor_side(training, validation, sideName)
    rpm = double(training.rpm(:));
    duty = double(training.duty(:));
    if numel(rpm) < 4 || numel(duty) ~= numel(rpm)
        error("Not enough %s wheel samples for signed actuator fit.", sideName);
    end
    if ~any(duty > 0.0) || ~any(duty < 0.0)
        error("%s wheel capture is missing one signed duty direction.", sideName);
    end

    % Signed discrete model: rpm(k+1) = a*rpm(k) + b*duty(k) + c*sign(duty(k)).
    phi = [rpm(1:end-1)' ; duty(1:end-1)' ; sign(duty(1:end-1))'];
    y = rpm(2:end)';
    if rank(phi) < 3
        error("Signed %s motor regressor rank is %d; expected 3.", sideName, rank(phi));
    end
    theta = y / phi;
    a = theta(1);
    b = theta(2);
    c = theta(3);
    prediction = theta * phi;
    residual = y - prediction;
    rmse = sqrt(mean(residual .^ 2));
    fitPercent = 100.0 * (1.0 - norm(residual) / max(norm(y - mean(y)), eps));
    conditionNumber = cond(phi * phi');
    positive = duty(1:end-1) > 0.0;
    negative = duty(1:end-1) < 0.0;
    positiveResidualRmse = sqrt(mean(residual(positive) .^ 2));
    negativeResidualRmse = sqrt(mean(residual(negative) .^ 2));
    if abs(a) > 0.0 && abs(a) < 1.0
        tau_s = -training.dt_s / log(abs(a));
    else
        tau_s = NaN;
    end
    dcGain = b / max(1.0 - a, eps);
    deadzoneDuty = abs(c / max(abs(b), eps));
    lambda_s = max(4.0 * training.dt_s, tau_s / 3.0);
    candidateKp = tau_s / (dcGain * lambda_s);
    candidateKi = 1.0 / (dcGain * lambda_s);

    validationResult = validate_motor_side(validation, theta);
    if exist("iddata", "file") == 2 && exist("tfest", "file") == 2
        try
            iddata(rpm, duty, training.dt_s); %#ok<NASGU>
            tfest(iddata(rpm, duty, training.dt_s), 1); %#ok<NASGU>
        catch toolboxError
            warning("%s toolbox cross-check failed: %s", sideName, toolboxError.message);
        end
    end

    result = struct();
    result.A_motor = a;
    result.B_motor = b;
    result.deadzone_duty = deadzoneDuty;
    result.tau_s = tau_s;
    result.condition_number = conditionNumber;
    result.rank = rank(phi);
    result.train = struct("rpm", y, "prediction", prediction, "residual", residual);
    result.validation = validationResult;
    result.summary = table(string(sideName), a, b, training.dt_s, tau_s, dcGain, ...
        deadzoneDuty, rank(phi), conditionNumber, rmse, fitPercent, ...
        positiveResidualRmse, negativeResidualRmse, validationResult.rmse, ...
        validationResult.fit_percent, candidateKp, candidateKi, ...
        'VariableNames', {'side', 'A_motor', 'B_motor', 'sample_time_s', ...
        'tau_s', 'dc_gain_rpm_per_duty', 'deadzone_duty', 'rank', ...
        'condition_number', 'train_rmse', 'train_fit_percent', ...
        'positive_residual_rmse', 'negative_residual_rmse', ...
        'validation_rmse', 'validation_fit_percent', ...
        'candidate_kp_duty_per_rpm', 'candidate_ki_duty_per_rpm_s'});
end

function result = validate_motor_side(validation, theta)
    rpm = double(validation.rpm(:));
    duty = double(validation.duty(:));
    if numel(rpm) < 3
        result = struct("rmse", NaN, "fit_percent", NaN, "prediction", []);
        return;
    end
    phi = [rpm(1:end-1)' ; duty(1:end-1)' ; sign(duty(1:end-1))'];
    y = rpm(2:end)';
    prediction = theta * phi;
    residual = y - prediction;
    result = struct("rmse", sqrt(mean(residual .^ 2)), ...
        "fit_percent", 100.0 * (1.0 - norm(residual) / max(norm(y - mean(y)), eps)), ...
        "prediction", prediction);
end
