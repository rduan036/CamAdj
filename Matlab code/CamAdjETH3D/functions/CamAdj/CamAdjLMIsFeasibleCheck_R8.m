function infeasible_index = CamAdjLMIsFeasibleCheck_R8(point_u, point_up, lb, ub, point_number, check_index)

if nargin < 5
    point_number = size(point_u,2);
    check_index = ones(point_number,1);
elseif nargin < 6
    check_index = ones(point_number,1);
end

infeasible_index = check_index;
pair_i = 1;

while pair_i <= point_number
    
    if check_index(pair_i) == 1
    
        % get Qi for polynomials
        u = point_u(:,pair_i);
        up = point_up(:,pair_i);
        G = computeG_R8(u,up);

        % get bound
        varMinMax = [lb, ub];

        Gn = getGn(varMinMax);
        n_Gn = size(Gn,3);

        % LMIs system
        setlmis([]);
        
        t = [];
        
        for i = 1:3
            t(i) = lmivar(1,[1 1]);
        end

        s = [];

        for i = 1:n_Gn
            s(i) = lmivar(1,[1 1]);
            my_lmi = newlmi;
            lmiterm([-my_lmi 1 1 s(i)], 1, 1);
        end

        my_lmi = newlmi;

        for i = 1:3
            lmiterm([-my_lmi 1 1 t(i)], 1, G(:,:,i));
        end

    %     lmiterm([my_lmi 1 1 s0], 1, 1);

        for i = 1:n_Gn
            lmiterm([my_lmi 1 1 s(i)], 1, Gn(:,:,i));
        end

        lmis = getlmis;

        [tmin,xfeas] = feasp(lmis,[0,20,0,0,1]);

        if tmin < 0
            infeasible_index(pair_i) = 0;
        end
    
    end
    
    pair_i = pair_i + 1;
    
end

end