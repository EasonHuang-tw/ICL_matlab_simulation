clear;
iteration = 100000
% learning rate
K = [1 0 ; 0 1]
gamma = 1.1
kcl = 0.000005

%time param
step_size = 0.0004 %second
delta_T = 0.01 %second
queue_size = delta_T/step_size
N = 20

%theta
theta_true = [5 10 15 20]'
theta_hat = [2.5 5 12 12]'

%state
x_true = [0 0]'
x_array = zeros(2,1,N+queue_size);

%integral
y_array = zeros(2,4,queue_size);
y_sum = zeros(2,4,N);

u_array = zeros(2,1,queue_size);
u_sum = zeros(2,1,N);

%whole
whole_x = zeros(100000,2,1);
whole_desired_x = zeros(100000,2,1);
whole_theta_hat = zeros(100000,4,1);
whole_theta_true = zeros(100000,4,1);

%noise
noise = wgn(iteration, 2, -30);

%% main

for i = 1:iteration
    i
	T = i * step_size; %true time
    % desired state
    x_desired_values =  x_desire(T);
    x_dot_desired_value = x_dot_desired(T);
    
    %calc error
    x_error_value = x_true - x_desired_values;
    
    whole_x(i,:,:) =  x_true;
    whole_desired_x(i,:,:) = x_desired_values;
    
    
    %% calculate and integral integral 'Y'
    y_current_value = [x_true(1,1)^2 sin(x_true(2,1)) 0 0; 0 x_true(2,1)*sin(T) x_true(1,1) x_true(1,1)*x_true(2,1)];
    y_array_last = y_array(:,:,1);
    for j = 1:queue_size-1
        y_array(:,:,j) = y_array(:,:,j+1);
    end
    y_array(:,:,queue_size) = y_current_value;
    for j = 1:N-1
        y_sum(:,:,j) = y_sum(:,:,j+1);
    end
    y_sum(:,:,N) = y_sum(:,:,N-1)+y_current_value-y_array_last;
    
     %% queue for 'x'
    for j = 1:N+queue_size-1
        x_array(:,:,j) = x_array(:,:,j+1);
    end
    x_array(:,:,N+queue_size) = x_true;

    %% calculate and intrgral input 'u' 
    control_input_value = x_dot_desired_value - y_current_value*theta_hat- K * x_error_value; %(5)
    u_array_last = u_array(:,:,1);
    for j = 1:queue_size-1
        u_array(:,:,j) = u_array(:,:,j+1);
    end
    u_array(:,:,queue_size) = control_input_value;
    for j = 1:N-1
        u_sum(:,:,j) = u_sum(:,:,j+1);
    end
    u_sum(:,:,N) = u_sum(:,:,N-1)+control_input_value-u_array_last;
    
    %% calculate 'kcl' term
    kcl_sum = zeros(4,1);
    for j = 1:N
        kcl_sum = kcl_sum + y_sum(:,:,j)'*(x_array(:,:,queue_size+j)-x_array(:,:,j)-u_sum(:,:,j)-y_sum(:,:,j)*theta_hat);
    end
    
    %% calculate & update theta_hat
    theta_hat_dot = gamma*y_current_value'*x_error_value;
    theta_hat_dot = theta_hat_dot+kcl*gamma*kcl_sum;
    theta_hat = theta_hat+theta_hat_dot*step_size;
    whole_theta_hat(i,:,:) = theta_hat;
    whole_theta_true(i,:,:) = theta_true;
    
    %% update x_true
    x_dot = y_current_value*theta_true+ control_input_value;
    %disp(x_dot)
    x_true = x_true+x_dot*step_size;%+noise(i)';
end
    
%% disp(whole_x(1:1000,1,1));
figure
plot(1:iteration,whole_x(1:iteration,:,1))
hold on
plot(1:iteration,whole_desired_x(1:iteration,:,1))
legend('x1','x2','x1_desired','x2_desired')

figure
plot(1:iteration,whole_theta_hat(1:iteration,:,1))
hold on
plot(1:iteration,whole_theta_true(1:iteration,:,1))

legend('1','2','3','4','1_desire','2_desire','3_desire','4_desire')

%% traj
function x_desired_value = x_desire(t)
    x_desired_value = 10*(1-exp(-0.1*t))*[sin(2*t) ; 0.4*cos(3*t)];
end

function x_dot_desired_value = x_dot_desired(t)
    %x_desired_dot_value =  [    sin(2*t)*exp(-t/10) - 2*cos(2*t)*(10*exp(-t/10) - 10);
    %                            (6*sin(3*t)*(10*exp(-t/10) - 10))/5 + (2*cos(3*t)*exp(-t/10))/5];
    x_dot_desired_value = [sin(2*t)*exp(-t/10) - 2*cos(2*t)*(10*exp(-t/10) - 10);(6*sin(3*t)*(10*exp(-t/10) - 10))/5 + (2*cos(3*t)*exp(-t/10))/5];
end