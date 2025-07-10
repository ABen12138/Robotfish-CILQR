clc
clear all
load("f_function.mat");
load("fx_function.mat");
load("fu_function.mat");
h=0.001;
iter_num=50;
cost_improvement=0.001;

final_state=zeros(12,N_total+1,iter_num);
final_control=zeros(1,N_total,iter_num);
cost_fun=zeros(1,iter_num);

initial_state = zeros(12,1);
initial_control = zeros(1,N_total);
state_temp = zeros(12,N_total+1);
state_temp(:,1) = initial_state;
control_temp=initial_control;
line_alpha = 10 .^ linspace(0, -3, 11);
line_alpha=[line_alpha,-0.001];

for i=2:(N_total+1)
    for j =1:12
        derivate_of_state=f_function{j,1}(state_temp(1,i-1),state_temp(2,i-1),state_temp(3,i-1),state_temp(4,i-1),state_temp(5,i-1),state_temp(6,i-1),...
            state_temp(7,i-1),state_temp(8,i-1),state_temp(9,i-1), state_temp(10,i-1),state_temp(11,i-1),state_temp(12,i-1),...
            control_temp(1,i-1));
        state_temp(j,i)=state_temp(j,i-1)+derivate_of_state*h;
    end
end
lx_k=zeros(12,1,N_total);
lxx_k=zeros(12,12,N_total);
lu_k=zeros(1,1,N_total);
luu_k=zeros(1,1,N_total);
lxu_k=zeros(12,1,N_total);
fx_k=zeros(12,12,N_total);
fu_k=zeros(12,1,N_total);

alpha_reach_max_flag=false;
for iter = 1:iter_num
    if alpha_reach_max_flag
        break;
    end
    iter
    I_12_8 = zeros(12,1);
    I_12_8(8) = 1;
    I_12_8(12) = -1;
    A_12_8 = zeros(12,12);
    A_12_8(8,8) = 1;
    A_12_8(8,12) = -1;
    A_12_8(12,8) = -1;
    A_12_8(12,12) = 1;
    Vx_N = -2*h*miu*(state_temp(8,N_total+1)-state_temp(12,N_total))*I_12_8 + constriant_get_der(1,state_temp(:,N_total+1),lamda) ;
    Vxx_N = -2*h*miu*A_12_8 + constriant_get_der(2,state_temp(:,N_total+1),lamda);
    for i = 1:N_total
        lx_k(:,1,i) = -2*h*miu*(state_temp(8,i)-state_temp(12,i-1))*I_12_8+constriant_get_der(1,state_temp(:,i),lamda);
        lxx_k(:,:,i) = -2*h*miu*A_12_8+constriant_get_der(2,state_temp(:,i),lamda);
        lu_k(1,1,i) = 0;
        luu_k(1,1,i) = 0;
        lxu_k(:,1,i) = zeros(12,1);
        state_tt(:,i) = check_zero(state_temp(:,i));
        for j=1:12
            for k=1:12
                fx_k(j,k,i)=h*fx_function{j,k}(state_tt(1,i),state_tt(2,i),state_tt(3,i),state_tt(4,i),state_tt(5,i),state_tt(6,i),...
                    state_tt(7,i),state_tt(8,i),state_tt(9,i), state_tt(10,i),state_tt(11,i),state_tt(12,i),...
                    control_temp(1,i));
            end
            fu_k(j,1,i)=h*fu_function{j,1}(state_tt(1,i),state_tt(2,i),state_tt(3,i),state_tt(4,i),state_tt(5,i),state_tt(6,i),...
                state_tt(7,i),state_tt(8,i),state_tt(9,i), state_tt(10,i),state_tt(11,i),state_tt(12,i),...
                control_temp(1,i));
        end
        fx_k(:,:,i) =  fx_k(:,:,i) + eye(size(fx_k,1));
    end
    Vx_temp =Vx_N;
    Vxx_temp = Vxx_N;
    alpha = 0.0005;
    forward_success_flag = false;
    while ~forward_success_flag
        if alpha>=10e20
            alpha_reach_max_flag=true;
            break;
        end
        K=zeros(N_total,12);
        d=zeros(N_total,1);
        success_flag=false;
        while (~success_flag)
            alpha = alpha * 5;
            tmp_flag=true;
            for i=N_total:-1:1
                Qx=lx_k(:,1,i)+(fx_k(:,:,i)')*Vx_temp;
                Qu=lu_k(1,1,i)+(fu_k(:,1,i)')*Vx_temp;
                Qxx=lxx_k(:,:,i)+(fx_k(:,:,i)')*Vxx_temp*fx_k(:,:,i);
                Quu=luu_k(1,1,i) +(fu_k(:,1,i)')*Vxx_temp*fu_k(:,1,i);
                Qxu=lxu_k(:,1,i)+(fx_k(:,:,i)')*Vxx_temp*fu_k(:,1,i);
                Qux=Qxu';
                Quu = Quu + alpha * eye(size(Quu,1));
                while true
                    try
                        R = chol(Quu);
                        if(i==1)
                            success_flag=true;
                        end
                        break;
                    catch
                        tmp_flag=false;
                        success_flag=false;
                        break;
                    end
                end
                if(tmp_flag==true)
                    K_temp = -1*inv(Quu)*Qux;
                    K(i,:) = K_temp;
                    d(i,:) = -1*inv(Quu)*Qu;
                    Vx_temp = Qx+(K_temp')*Qu;
                    Vxx_temp=Qxx-(K_temp')*Quu*K_temp;
                else
                    break;
                end
            end
        end    
    final_state(:,:,iter) = state_temp;
    final_control(:,:,iter)=control_temp;
end

function result = cost_function(state_input,h,miu)
result=0;
for i=2:size(state_input,2)
    result = result + (-1)*h*miu*(state_input(8,i)-state_input(12,i-1))*(state_input(8,i)-state_input(12,i-1));
end
end

function result = check_zero(state_input)
result = state_input;
for i=1:size(state_input,1)
    if abs(result(i,1)) <= 1e-6
        result(i,1) = 1e-6;
    end
end
end

function result = constriant_get_cost(state_input,lamda)
result=0;
for i=1:size(state_input,2)
    if(state_input(4,i)*state_input(4,i)>(pi^2/4))
        result = result +lamda * (state_input(4,i)*state_input(4,i)-(pi^2/4))^4;
    end
    if(state_input(5,i)*state_input(5,i)>(pi^2/4))
        result = result + lamda * (state_input(5,i)*state_input(5,i)-(pi^2/4))^4;
    end
    if(state_input(11,i)*state_input(11,i)>(pi^2/4))
        result = result +lamda * (state_input(11,i)*state_input(11,i)-(pi^2/4))^4;
    end
    if(state_input(12,i)*state_input(12,i)>(pi^2*100/9))
        result = result + lamda * (state_input(12,i)*state_input(12,i)-(pi^2*100/9))^4;
    end
end
end
