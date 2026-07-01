% Analyze open-loop BLDC wheel step response collected by collect_motor_steps.ps1.
%
% Run from repository root in MATLAB:
%   tools/analyze_motor_steps

clear; clc;

csvPath = fullfile("data", "motor_step_dynamic.csv");
data = readtable(csvPath);
names = string(data.Properties.VariableNames);
if ismember("target", names) && ~ismember("target_motor_rpm", names)
    data.Properties.VariableNames{strcmp(data.Properties.VariableNames, "target")} = "target_motor_rpm";
end
if ismember("left_speed", names) && ~ismember("left_motor_rpm", names)
    data.Properties.VariableNames{strcmp(data.Properties.VariableNames, "left_speed")} = "left_motor_rpm";
end
if ismember("right_speed", names) && ~ismember("right_motor_rpm", names)
    data.Properties.VariableNames{strcmp(data.Properties.VariableNames, "right_speed")} = "right_motor_rpm";
end

data = data(data.mode == 2 & data.feedback_online == 1, :);
duties = unique(data.command_duty);

summary = table();

figure("Name", "Motor open-loop step response");
tiledlayout(numel(duties), 1, "TileSpacing", "compact");

for i = 1:numel(duties)
    duty = duties(i);
    stepData = data(data.command_duty == duty, :);
    stepData.t = (stepData.time_ms - stepData.time_ms(1)) / 1000.0;

    tailStart = floor(height(stepData) * 0.8);
    tail = stepData(tailStart:end, :);

    leftSS = mean(tail.left_motor_rpm);
    rightSS = mean(tail.right_motor_rpm);

    summary = [summary; table(duty, height(stepData), leftSS, rightSS, ...
        mean(tail.left_duty), mean(tail.right_duty), ...
        "VariableNames", ["duty", "samples", "left_ss", "right_ss", "left_duty", "right_duty"])]; %#ok<AGROW>

    nexttile;
    plot(stepData.t, stepData.left_motor_rpm, "r"); hold on;
    plot(stepData.t, stepData.right_motor_rpm, "b");
    yline(leftSS, "r--");
    yline(rightSS, "b--");
    grid on;
    ylabel("motor rpm");
    title("D," + duty);
    if i == numel(duties)
        xlabel("time (s)");
    end
end

disp(summary);

leftFit = polyfit(summary.duty, summary.left_ss, 1);
rightFit = polyfit(summary.duty, summary.right_ss, 1);
avgFit = polyfit(summary.duty, (summary.left_ss + summary.right_ss) / 2, 1);

fprintf("left motor rpm ~= %.4f * duty %+ .2f\n", leftFit(1), leftFit(2));
fprintf("right motor rpm ~= %.4f * duty %+ .2f\n", rightFit(1), rightFit(2));
fprintf("avg motor rpm ~= %.4f * duty %+ .2f\n", avgFit(1), avgFit(2));

% Fit one first-order transfer function per wheel using all concatenated data.
% Input is the magnitude of actual duty, output is motor-axis RPM feedback.
Ts = median(diff(data.time_ms)) / 1000.0;
uLeft = abs(data.left_duty);
uRight = abs(data.right_duty);
yLeft = data.left_motor_rpm;
yRight = data.right_motor_rpm;

leftIdData = iddata(yLeft, uLeft, Ts);
rightIdData = iddata(yRight, uRight, Ts);

leftSys = tfest(leftIdData, 1);
rightSys = tfest(rightIdData, 1);

disp("Left wheel model:");
disp(leftSys);
disp("Right wheel model:");
disp(rightSys);

figure("Name", "Model comparison");
compare(leftIdData, leftSys);
title("Left wheel model comparison");

figure("Name", "Right model comparison");
compare(rightIdData, rightSys);
title("Right wheel model comparison");

% Open MATLAB PID Tuner for a PI speed controller.
% If PID Tuner is unavailable, use pidtune(leftSys, "PI") in the command window.
pidTuner(leftSys, "PI");
