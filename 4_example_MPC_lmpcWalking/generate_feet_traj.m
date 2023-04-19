% generate_feet_traj.m
% This script is to generate feet trajectory using interpolation.

%% 
addpath ./poly6

rfoot_tra = zeros(desired_walking_time, 3); % x y z in world frame
lfoot_tra = zeros(desired_walking_time, 3); % x y z

% foot x z trajectory interpolation
S_foot_x = getPolyCoeff(step_time,            0, 0, 0,   0.5*step_length,  step_length, 0, 0);
S_foot_z = getPolyCoeff(step_time, ankle_height, 0, 0, ankle_height+0.05, ankle_height, 0, 0);

tspan = delta_t * (1:no_steps_per_T);
foot_x_tspan = getSixOrderPoly(S_foot_x, tspan');
foot_y_tspan = half_step_width * ones(no_steps_per_T, 1);
foot_z_tspan = getSixOrderPoly(S_foot_z, tspan');

S_foot_x_2steps = getPolyCoeff(step_time,  0, 0, 0,   step_length,  2*step_length, 0, 0);
foot_x_tspan_2steps = getSixOrderPoly(S_foot_x_2steps, tspan');

%% rfoot
j = 0;
for i = 1:no_desired_steps
    if ismember(i, [1, 2])
        rfoot_tra(j+(1:no_steps_per_T), :) = [desiredFoot_steps(i, 1), -half_step_width, ankle_height] .* ones(no_steps_per_T, 1);
    elseif i == 3
        if flag_beginSwgFoot == 'r' % right foot
            rfoot_tra(j+(1:no_steps_per_T), :) = [foot_x_tspan, -foot_y_tspan, foot_z_tspan];
        else % left foot
            rfoot_tra(j+(1:no_steps_per_T), :) = [desiredFoot_steps(i, 1), -half_step_width, ankle_height] .* ones(no_steps_per_T, 1);
        end
    elseif i == no_desired_steps || i == no_desired_steps-1 % last two steps to stop
        rfoot_tra(j+(1:no_steps_per_T), :) = [desiredFoot_steps(i-1, 1), -half_step_width, ankle_height] .* ones(no_steps_per_T, 1);
    else
        if flag_beginSwgFoot == 'r' % right foot move first
            if mod(i - 3, 2) == 1
                rfoot_tra(j+(1:no_steps_per_T), :) = [desiredFoot_steps(i, 1), -half_step_width, ankle_height] .* ones(no_steps_per_T, 1);
            else
                rfoot_tra(j+(1:no_steps_per_T), :) = [foot_x_tspan_2steps + desiredFoot_steps(i-1, 1), -foot_y_tspan, foot_z_tspan];
                if i == no_desired_steps-2 && mod(no_desired_steps, 2) == 1
                    rfoot_tra(j+(1:no_steps_per_T), :) = [foot_x_tspan + desiredFoot_steps(i-1, 1), -foot_y_tspan, foot_z_tspan];
                end
            end
        else % left foot move first
            if mod(i - 3, 2) == 1
                rfoot_tra(j+(1:no_steps_per_T), :) = [foot_x_tspan_2steps + desiredFoot_steps(i-1, 1), -foot_y_tspan, foot_z_tspan];
                if i == no_desired_steps-2 && mod(no_desired_steps, 2) == 0
                    rfoot_tra(j+(1:no_steps_per_T), :) = [foot_x_tspan + desiredFoot_steps(i-1, 1), -foot_y_tspan, foot_z_tspan];
                end
            else
                rfoot_tra(j+(1:no_steps_per_T), :) = [desiredFoot_steps(i, 1), -half_step_width, ankle_height] .* ones(no_steps_per_T, 1);
            end
        end
    end
    
    j = j + no_steps_per_T;
end

%% lfoot
j = 0;
for i = 1:no_desired_steps
    if ismember(i, [1, 2])
        lfoot_tra(j+(1:no_steps_per_T), :) = [desiredFoot_steps(i, 1), half_step_width, ankle_height] .* ones(no_steps_per_T, 1);
    elseif i == 3
        if flag_beginSwgFoot == 'r' % right foot
            lfoot_tra(j+(1:no_steps_per_T), :) = [desiredFoot_steps(i, 1), half_step_width, ankle_height] .* ones(no_steps_per_T, 1);
        else % left foot
            lfoot_tra(j+(1:no_steps_per_T), :) = [foot_x_tspan, foot_y_tspan, foot_z_tspan];
        end
    elseif i == no_desired_steps || i == no_desired_steps-1 % last two steps to stop
        lfoot_tra(j+(1:no_steps_per_T), :) = [desiredFoot_steps(i-1, 1), half_step_width, ankle_height] .* ones(no_steps_per_T, 1);
    else
        if flag_beginSwgFoot == 'r' % right foot move first           
            if mod(i - 3, 2) == 1
                lfoot_tra(j+(1:no_steps_per_T), :) = [foot_x_tspan_2steps + desiredFoot_steps(i-1, 1), foot_y_tspan, foot_z_tspan];
                if i == no_desired_steps-2 && mod(no_desired_steps, 2) == 0
                    lfoot_tra(j+(1:no_steps_per_T), :) = [foot_x_tspan + desiredFoot_steps(i-1, 1), foot_y_tspan, foot_z_tspan];
                end
            else
                lfoot_tra(j+(1:no_steps_per_T), :) = [desiredFoot_steps(i, 1), half_step_width, ankle_height] .* ones(no_steps_per_T, 1);
            end
        else % left foot move first            
            if mod(i - 3, 2) == 1
                lfoot_tra(j+(1:no_steps_per_T), :) = [desiredFoot_steps(i, 1), half_step_width, ankle_height] .* ones(no_steps_per_T, 1);
            else
                lfoot_tra(j+(1:no_steps_per_T), :) = [foot_x_tspan_2steps + desiredFoot_steps(i-1, 1), foot_y_tspan, foot_z_tspan];
                if i == no_desired_steps-2 && mod(no_desired_steps, 2) == 1
                    lfoot_tra(j+(1:no_steps_per_T), :) = [foot_x_tspan + desiredFoot_steps(i-1, 1), foot_y_tspan, foot_z_tspan];
                end
            end
        end
    end
    
    j = j + no_steps_per_T;
end

rmpath ./poly6