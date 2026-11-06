% buildFuzzyControllerMamdani2.m
% Creates a Mamdani‐style FIS matching the Python FuzzyController with offset_interval

%% Parameters
target_distance  = 90;   % ← set this to your desired TARGET_DISTANCE
offset_interval  = 0.1;    % ← set this to your desired OFFSET_INTERVAL
fisName          = 'FuzzyControllerMamdani2';

%% Create new Mamdani FIS
fis = newfis(fisName, 'mamdani');

%% Input 1: offset_angle (degrees)
fis = addvar(fis, 'input', 'offset_angle', [-30 30]);
fis = addmf(fis, 'input', 1, 'left_far',   'trimf', [-30 -20 -10]);
fis = addmf(fis, 'input', 1, 'left_near',  'trimf', [-15  -5  -offset_interval]);
fis = addmf(fis, 'input', 1, 'center',     'trimf', [-offset_interval, 0, offset_interval]);
fis = addmf(fis, 'input', 1, 'right_near', 'trimf', [ offset_interval, 5, 15]);
fis = addmf(fis, 'input', 1, 'right_far',  'trimf', [ 10, 20, 30]);

%% Input 2: distance (cm)
fis = addvar(fis, 'input', 'distance', [0 target_distance*6]);
fis = addmf(fis, 'input', 2, 'very_close','trimf', [0, target_distance/3, target_distance*2/3]);
fis = addmf(fis, 'input', 2, 'ideal',     'trimf', [target_distance/2, target_distance, target_distance*1.5]);
fis = addmf(fis, 'input', 2, 'far',       'trimf', [target_distance, target_distance*2, target_distance*3]);
fis = addmf(fis, 'input', 2, 'very_far',  'trimf', [target_distance*2, target_distance*4, target_distance*6]);

%% Output 1: left_motor_speed (%)
fis = addvar(fis, 'output', 'left_motor', [-100 100]);
fis = addmf(fis, 'output', 1, 'reverse_fast',  'trimf', [-100, -70, -40]);
fis = addmf(fis, 'output', 1, 'reverse_slow',  'trimf', [ -60, -30,   0]);
fis = addmf(fis, 'output', 1, 'stop',          'trimf', [ -20,   0,  20]);
fis = addmf(fis, 'output', 1, 'forward_slow',  'trimf', [   0,  30,  60]);
fis = addmf(fis, 'output', 1, 'forward_fast',  'trimf', [  40,  70, 100]);

%% Output 2: right_motor_speed (%)
fis = addvar(fis, 'output', 'right_motor', [-100 100]);
fis = addmf(fis, 'output', 2, 'reverse_fast',  'trimf', [-100, -70, -40]);
fis = addmf(fis, 'output', 2, 'reverse_slow',  'trimf', [ -60, -30,   0]);
fis = addmf(fis, 'output', 2, 'stop',          'trimf', [ -20,   0,  20]);
fis = addmf(fis, 'output', 2, 'forward_slow',  'trimf', [   0,  30,  60]);
fis = addmf(fis, 'output', 2, 'forward_fast',  'trimf', [  40,  70, 100]);

%% Rule list
% offset_angle: 1:left_far,2:left_near,3:center,4:right_near,5:right_far
% distance:     1:very_close,2:ideal,3:far,4:very_far
% left_motor/right_motor MFs: 1:reverse_fast,2:reverse_slow,3:stop,4:forward_slow,5:forward_fast
ruleList = [
    3 1 2 2 1 1;
    3 2 3 3 1 1;
    3 3 4 4 1 1;
    3 4 5 5 1 1;
    
    2 1 1 2 1 1;
    2 2 3 4 1 1;
    2 3 4 5 1 1;
    2 4 5 5 1 1;
    
    1 1 1 3 1 1;
    1 2 2 4 1 1;
    1 3 3 5 1 1;
    1 4 4 5 1 1;
    
    4 1 2 1 1 1;
    4 2 4 3 1 1;
    4 3 5 4 1 1;
    4 4 5 5 1 1;
    
    5 1 3 1 1 1;
    5 2 4 2 1 1;
    5 3 5 3 1 1;
    5 4 5 4 1 1
];
fis = addrule(fis, ruleList);

%% Save and launch Designer
writefis(fis, fisName);
fprintf('Mamdani FIS saved as %s.fis\n', fisName);

% To open in Designer, use:
 %fis = readfis('FuzzyControllerMamdani2.fis');
 %fuzzyLogicDesigner(fis);
