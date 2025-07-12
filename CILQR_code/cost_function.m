function result = cost_function(state_input,h,miu)
result=0;
for i=2:size(state_input,2)
    result = result + (-1)*h*miu*(state_input(8,i)-state_input(12,i-1))*(state_input(8,i)-state_input(12,i-1));
end
end