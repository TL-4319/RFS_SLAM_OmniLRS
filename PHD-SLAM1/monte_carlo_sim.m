close all;
clear;
clc;

% Define number of MC runs
num_run = 1;

% Visualization
draw = false;

% add path to util functions. 
addpath('../utils/')

% Select dataset
load ('dataset/straight.mat');


%% Define sensor parameters to be used to generate measurements
% For cartesian model, meas_vector = [x, y, z]'. 
% For range-bearing-elevation, meas_vector = [range_m, bearing_rad, elevation_rad]', 
sensor_params.meas_model = 'range-bearing-elevation'; %[cartesian, range-bearing-elevation]
sensor_params.FOV_bound = dataset.scan_pattern.FOV_bounds;
sensor_params.HFOV = deg2rad(70);
sensor_params.VFOV = deg2rad(30);
sensor_params.max_range = 15;
sensor_params.min_range = 0.4;
sensor_params.detect_prob = 0.95;
sensor_params.sensor_rate = 0.8;
sensor_params.measurement_std = [0.1, 0.01, 0.01];  
sensor_params.avg_num_clutter = 5;
sensor_params.pos_body_sensor = dataset.pos_body_sensor;
sensor_params.quat_body_sensor = dataset.quat_body_sensor;


sensor_params.meas_area = 20;
sensor_params.clutter_density = sensor_params.avg_num_clutter / ...
    sensor_params.meas_area;

%% Define odometry configurations 
% Motion covariance = [cov_x, cov_y, cov_z, cov_phi, cov_theta, cov_psi]
odom_params.motion_sigma = [0.5; 0.5; 0; 0.001; 0.001; 0.5]; 

%% Defind filter parameters
% Detector type
filter_params.detector = 'crater'; %[crater, peak, combined]

% Sensor params exposed to filter
filter_params.sensor = sensor_params; % Copy sensor parameter set so filter has different parameters for robust analysis
filter_params.sensor.detect_prob = 0.7;
filter_params.sensor.measurement_std = [0.1, 0.05, 0.05];
filter_params.sensor.avg_num_clutter = 5;

% Particle filter params
filter_params.num_particle = 1;
filter_params.resample_threshold = 0.1; % Percentage of num_particle for resample to trigger
filter_params.likelihood_method = 'single-cluster'; %['empty', 'single-feature, 'single-cluster']

% Motion covariance = [cov_x, cov_y, cov_z, cov_phi, cov_theta, cov_psi]
% For 2D, cov_z, cov_phi and cov_theta = 0
filter_params.motion_model = 'truth'; % [odometry, random-walk, truth]
filter_params.motion_sigma = [0.5; 0.5; 0; 0.001; 0.001; 0.5];

% Map PHD config

filter_params.birthGM_intensity =  0.05;             % Default intensity of GM component when birth
filter_params.birthGM_std = 0.5;                  % Default standard deviation in position of GM component when birth
filter_params.map_std = 0;
filter_params.adaptive_birth_dist_thres = 1;
filter_params.GM_inten_thres = 0.5;                % Threshold to use a component for importance weight calc and plotting
filter_params.pruning_thres = 10^-5;
filter_params.merge_dist = 4;
filter_params.num_GM_cap = 5000;
filter_params.inner_filter = 'ekf';
filter_params.map_est_method = 'exp';           % Method to extract map est. 'exp' or 'thres'

% NO INPUT REQUIRED for the rest of the section
% Calculate corresponding matrices
filter_params.sensor.clutter_density = filter_params.sensor.avg_num_clutter / ...
    filter_params.sensor.meas_area;
filter_params.sensor.R = diag(filter_params.sensor.measurement_std.^2);
filter_params.birthGM_cov = diag([filter_params.birthGM_std, ...
    filter_params.birthGM_std, filter_params.birthGM_std].^2);
filter_params.map_Q = diag([filter_params.map_std, filter_params.map_std,...
    filter_params.map_std].^2);

%% Setup struct to save datas
simulation.type = "PHD-SLAM1";
simulation.sensor_params = sensor_params;
simulation.filter_params = filter_params;
simulation.odom_params = odom_params;

results = cell(num_run,1);
file_name = strcat('../sim_result/',sprintf('sim-%s.mat', datestr(now,'yyyymmdd-HHMM')));

for ii = 1:num_run
    disp(ii)
    [results{ii,1}, truth] = phd_slam1_3d_instance(dataset,sensor_params, odom_params, filter_params, draw);
    if mod(ii,10)==0
        % Save every 10 run
        simulation.result = results;
        save(file_name,"simulation",'-v7.3');
    end
    if ii == 1
        simulation.truth = truth; % Only need one copy of truth data
    end
end

simulation.result = results;
save(file_name,"simulation",'-v7.3');