function result = check_zero(state_input)
result = state_input;
for i=1:size(state_input,1)
    if abs(result(i,1)) <= 1e-6
        result(i,1) = 1e-6;
    end
end
end