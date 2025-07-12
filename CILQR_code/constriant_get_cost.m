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