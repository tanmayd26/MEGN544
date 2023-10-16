load('linklist.mat');
load('paramlist.mat');

H = eye(4);

    
    % Loop through each link in the linkList
    for i = 1:length(linkList)
        % Extract DH parameters from the link
        isRotary = linkList(i).isRotary;
        if isRotary == 1
            a = linkList(i).a;
            d = linkList(i).d;
            alpha = linkList(i).alpha;
            offset = linkList(i).offset;
            

             % Extract the joint variable from paramList
            theta = paramList(i);

        % Calculate the transformation matrix for this link
  
            % Rotary joint
            A = dhTransform(a,d,alpha,theta+offset);

        elseif isRotary == 0
            a = linkList(i).a;
            alpha = linkList(i).alpha;
            theta = linkList(i).theta;
            offset = linkList(i).offset;
            

             % Extract the joint variable from paramList
            d = paramList(i);
            % Prismatic joint
            A = dhTransform(a,d+offset,alpha,theta);
        else
            % Undefined joint type
            error('Undefined joint type for link %d', i);
        end

        % Update the transformation matrix H
        H = H * A;
    end