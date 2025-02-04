close all;
clear; 
clc;

addpath('../util/')

%% Select simulation
% Parse simulation
%[file,location] = uigetfile;
%load (strcat(location,file));
load("sim_result/sim-20250130-2115-crater.mat");

%% 
time_vec = simulation.truth.time_vec;
dt = time_vec(2) - time_vec(1);
sensor_time_vec = simulation.truth.sensor_time_vec;
sensor_dt = sensor_time_vec(2) - sensor_time_vec(1);

% Pre allocate traj metrics
slam_pos_error = zeros(3,size(simulation.truth.pos,2));
slam_eul_error = slam_pos_error;
odom_pos_error = slam_pos_error;
odom_eul_error = slam_pos_error;

% Compute time
compute_time = zeros(size(simulation.result,1), size(time_vec,2));

% Iterate through sim results to find avg errors
for ii = 1:size(simulation.result,1)
%for ii = 1:60
    disp(ii)
    if ii == 1
        slam_pos_error = abs(simulation.truth.pos -...
            simulation.result{ii,1}.filter_est.pos);
        odom_pos_error = abs(simulation.truth.pos -...
            simulation.result{ii,1}.odom_est.pos);

        true_eul = transpose(quat2eul(simulation.truth.quat));
        slam_eul = transpose(quat2eul(simulation.result{ii,1}.filter_est.quat));
        odom_eul = transpose(quat2eul(simulation.result{ii,1}.odom_est.quat));

        slam_eul_error = abs(true_eul - slam_eul);
        odom_eul_error = abs(true_eul - odom_eul);

    else
        slam_pos_error = ((slam_pos_error * (ii-1)) + abs(simulation.truth.pos -...
            simulation.result{ii,1}.filter_est.pos))/ii;
        odom_pos_error = ((odom_pos_error * (ii-1)) + abs(simulation.truth.pos -...
            simulation.result{ii,1}.odom_est.pos))/ii;

        true_eul = transpose(quat2eul(simulation.truth.quat));
        slam_eul = transpose(quat2eul(simulation.result{ii,1}.filter_est.quat));
        odom_eul = transpose(quat2eul(simulation.result{ii,1}.odom_est.quat));

        slam_eul_error = ((slam_eul_error * (ii-1)) + abs(true_eul -...
           slam_eul))/ii;
        odom_eul_error = ((odom_eul_error * (ii-1)) + abs(true_eul -...
           odom_eul))/ii;
    end
    
   
    
    compute_time(ii,:) = simulation.result{ii,1}.filter_est.compute_time';
end
avg_compute_time = mean(compute_time,1);

dist_travel = simulation.truth.pos;
dist_travel(:,2:end) = simulation.truth.pos(:,2:end) - simulation.truth.pos(:,1:end-1);
dist_travel = vecnorm(dist_travel);
dist_travel = cumsum(dist_travel);

error1.filter_pos_error = slam_pos_error;
error1.filter_eul_error = slam_eul_error;
error1.odom_pos_error = odom_pos_error;
error1.odom_eul_error = odom_eul_error;
error1.time_vec = time_vec;
error1.dist_travel = dist_travel;
error1.compute_time = avg_compute_time;


%%
load("sim_result/sim-20250131-0729-peak.mat");

%% 
time_vec = simulation.truth.time_vec;
dt = time_vec(2) - time_vec(1);
sensor_time_vec = simulation.truth.sensor_time_vec;
sensor_dt = sensor_time_vec(2) - sensor_time_vec(1);

% Pre allocate traj metrics
slam_pos_error = zeros(3,size(simulation.truth.pos,2));
slam_eul_error = slam_pos_error;
odom_pos_error = slam_pos_error;
odom_eul_error = slam_pos_error;

% Compute time
compute_time = zeros(size(simulation.result,1), size(time_vec,2));

% Iterate through sim results to find avg errors
for ii = 1:size(simulation.result,1)
%for ii = 1:60
    disp(ii)
    if ii == 1
        slam_pos_error = abs(simulation.truth.pos -...
            simulation.result{ii,1}.filter_est.pos);
        odom_pos_error = abs(simulation.truth.pos -...
            simulation.result{ii,1}.odom_est.pos);

        true_eul = transpose(quat2eul(simulation.truth.quat));
        slam_eul = transpose(quat2eul(simulation.result{ii,1}.filter_est.quat));
        odom_eul = transpose(quat2eul(simulation.result{ii,1}.odom_est.quat));

        slam_eul_error = abs(true_eul - slam_eul);
        odom_eul_error = abs(true_eul - odom_eul);

    else
        slam_pos_error = ((slam_pos_error * (ii-1)) + abs(simulation.truth.pos -...
            simulation.result{ii,1}.filter_est.pos))/ii;
        odom_pos_error = ((odom_pos_error * (ii-1)) + abs(simulation.truth.pos -...
            simulation.result{ii,1}.odom_est.pos))/ii;

        true_eul = transpose(quat2eul(simulation.truth.quat));
        slam_eul = transpose(quat2eul(simulation.result{ii,1}.filter_est.quat));
        odom_eul = transpose(quat2eul(simulation.result{ii,1}.odom_est.quat));

        slam_eul_error = ((slam_eul_error * (ii-1)) + abs(true_eul -...
           slam_eul))/ii;
        odom_eul_error = ((odom_eul_error * (ii-1)) + abs(true_eul -...
           odom_eul))/ii;
    end
    
   
    
    compute_time(ii,:) = simulation.result{ii,1}.filter_est.compute_time';
end
avg_compute_time = mean(compute_time,1);

dist_travel = simulation.truth.pos;
dist_travel(:,2:end) = simulation.truth.pos(:,2:end) - simulation.truth.pos(:,1:end-1);
dist_travel = vecnorm(dist_travel);
dist_travel = cumsum(dist_travel);

error2.filter_pos_error = slam_pos_error;
error2.filter_eul_error = slam_eul_error;
error2.odom_pos_error = odom_pos_error;
error2.odom_eul_error = odom_eul_error;
error2.time_vec = time_vec;
error2.dist_travel = dist_travel;
error2.compute_time = avg_compute_time;

%%
load("sim_result/sim-20250131-0850-comb.mat");

%% 
time_vec = simulation.truth.time_vec;
dt = time_vec(2) - time_vec(1);
sensor_time_vec = simulation.truth.sensor_time_vec;
sensor_dt = sensor_time_vec(2) - sensor_time_vec(1);

% Pre allocate traj metrics
slam_pos_error = zeros(3,size(simulation.truth.pos,2));
slam_eul_error = slam_pos_error;
odom_pos_error = slam_pos_error;
odom_eul_error = slam_pos_error;

% Compute time
compute_time = zeros(size(simulation.result,1), size(time_vec,2));

% Iterate through sim results to find avg errors
for ii = 1:size(simulation.result,1)
%for ii = 1:60
    disp(ii)
    if ii == 1
        slam_pos_error = abs(simulation.truth.pos -...
            simulation.result{ii,1}.filter_est.pos);
        odom_pos_error = abs(simulation.truth.pos -...
            simulation.result{ii,1}.odom_est.pos);

        true_eul = transpose(quat2eul(simulation.truth.quat));
        slam_eul = transpose(quat2eul(simulation.result{ii,1}.filter_est.quat));
        odom_eul = transpose(quat2eul(simulation.result{ii,1}.odom_est.quat));

        slam_eul_error = abs(true_eul - slam_eul);
        odom_eul_error = abs(true_eul - odom_eul);

    else
        slam_pos_error = ((slam_pos_error * (ii-1)) + abs(simulation.truth.pos -...
            simulation.result{ii,1}.filter_est.pos))/ii;
        odom_pos_error = ((odom_pos_error * (ii-1)) + abs(simulation.truth.pos -...
            simulation.result{ii,1}.odom_est.pos))/ii;

        true_eul = transpose(quat2eul(simulation.truth.quat));
        slam_eul = transpose(quat2eul(simulation.result{ii,1}.filter_est.quat));
        odom_eul = transpose(quat2eul(simulation.result{ii,1}.odom_est.quat));

        slam_eul_error = ((slam_eul_error * (ii-1)) + abs(true_eul -...
           slam_eul))/ii;
        odom_eul_error = ((odom_eul_error * (ii-1)) + abs(true_eul -...
           odom_eul))/ii;
    end
    
   
    
    compute_time(ii,:) = simulation.result{ii,1}.filter_est.compute_time';
end
avg_compute_time = mean(compute_time,1);

dist_travel = simulation.truth.pos;
dist_travel(:,2:end) = simulation.truth.pos(:,2:end) - simulation.truth.pos(:,1:end-1);
dist_travel = vecnorm(dist_travel);
dist_travel = cumsum(dist_travel);

error3.filter_pos_error = slam_pos_error;
error3.filter_eul_error = slam_eul_error;
error3.odom_pos_error = odom_pos_error;
error3.odom_eul_error = odom_eul_error;
error3.time_vec = time_vec;
error3.dist_travel = dist_travel;
error3.compute_time = avg_compute_time;


%% Plot
figure("Position",[1,1,700,600])
subplot (3,1,1)
plot (time_vec, odom_pos_error(1,:),'k','DisplayName','Odometry','LineWidth',2)
hold on 
plot (time_vec, error1.filter_pos_error(1,:),'g','DisplayName',"Crater",'LineWidth',2)
plot (time_vec, error2.filter_pos_error(1,:),'b','DisplayName',"Peak",'LineWidth',2)
plot (time_vec, error3.filter_pos_error(1,:),'r','DisplayName',"Combined",'LineWidth',2)
xlabel("Time (s)")
ylabel("X Error (m)")
grid on
legend
ax=gca;
        set(ax,'FontName','Times','Fontsize',15)
        set(gca, 'FontName', 'Arial')
        set(gca,'color','w');
ylim([0 0.6])

subplot (3,1,2)
plot (time_vec, odom_pos_error(1,:),'k','DisplayName','Odometry','LineWidth',2)
hold on 
plot (time_vec, error1.filter_pos_error(2,:),'g','DisplayName',"Crater",'LineWidth',2)
plot (time_vec, error2.filter_pos_error(2,:),'b','DisplayName',"Peak",'LineWidth',2)
plot (time_vec, error3.filter_pos_error(2,:),'r','DisplayName',"Peak",'LineWidth',2)
xlabel("Time (s)")
ylabel("Y Error (m)")
grid on
ax=gca;
        set(ax,'FontName','Times','Fontsize',15)
        set(gca, 'FontName', 'Arial')
        set(gca,'color','w');
ylim([0 1.5])

subplot (3,1,3)
plot (time_vec, odom_pos_error(1,:),'k','DisplayName','Odometry','LineWidth',2)
hold on 
plot (time_vec, error1.filter_pos_error(3,:),'g','DisplayName',"Crater",'LineWidth',2)
plot (time_vec, error2.filter_pos_error(3,:),'b','DisplayName',"Peak",'LineWidth',2)
plot (time_vec, error3.filter_pos_error(3,:),'r','DisplayName',"Peak",'LineWidth',2)
xlabel("Time (s)")
ylabel("Z Error (m)")
grid on
ax=gca;
        set(ax,'FontName','Times','Fontsize',15)
        set(gca, 'FontName', 'Arial')
        set(gca,'color','w');
ylim([0 0.15])

figure("Position",[1,1,700,600])
subplot (3,1,1)
plot (time_vec, odom_eul_error(3,:) * 180/pi,'k','DisplayName','Odometry','LineWidth',2)
hold on 
plot (time_vec, error1.filter_eul_error(3,:) * 180/pi,'g','DisplayName',"Crater",'LineWidth',2)
plot (time_vec, error2.filter_eul_error(3,:) * 180/pi,'b','DisplayName',"Peak",'LineWidth',2)
plot (time_vec, error3.filter_eul_error(3,:) * 180/pi,'r','DisplayName',"Combined",'LineWidth',2)
xlabel("Time (s)")
ylabel("Roll Error (^o)")
grid on
legend
ax=gca;
        set(ax,'FontName','Times','Fontsize',15)
        set(gca, 'FontName', 'Arial')
        set(gca,'color','w');
%ylim([0 0.6])

subplot (3,1,2)
plot (time_vec, odom_eul_error(2,:) * 180/pi,'k','DisplayName','Odometry','LineWidth',2)
hold on 
plot (time_vec, error1.filter_eul_error(2,:) * 180/pi,'g','DisplayName',"Crater",'LineWidth',2)
plot (time_vec, error2.filter_eul_error(2,:) * 180/pi,'b','DisplayName',"Peak",'LineWidth',2)
plot (time_vec, error3.filter_eul_error(2,:) * 180/pi,'r','DisplayName',"Peak",'LineWidth',2)
xlabel("Time (s)")
ylabel("Pitch Error (^o)")
grid on
ax=gca;
        set(ax,'FontName','Times','Fontsize',15)
        set(gca, 'FontName', 'Arial')
        set(gca,'color','w');
%ylim([0 0.6])

subplot (3,1,3)
plot (time_vec, odom_eul_error(1,:) * 180/pi,'k','DisplayName','Odometry','LineWidth',2)
hold on 
plot (time_vec, error1.filter_eul_error(1,:) * 180/pi,'g','DisplayName',"Crater",'LineWidth',2)
plot (time_vec, error2.filter_eul_error(1,:) * 180/pi,'b','DisplayName',"Peak",'LineWidth',2)
plot (time_vec, error3.filter_eul_error(1,:) * 180/pi,'r','DisplayName',"Peak",'LineWidth',2)
xlabel("Time (s)")
ylabel("Heading Error (^o)")
grid on
ax=gca;
        set(ax,'FontName','Times','Fontsize',15)
        set(gca, 'FontName', 'Arial')
        set(gca,'color','w');
%ylim([0 20])

%% Util functions
function Xc= get_comps(X,c)
    if isempty(X)
        Xc= [];
    else
        Xc= X(c,:);
    end
end

