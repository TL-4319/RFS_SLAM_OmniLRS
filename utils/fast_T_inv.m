function inv_T = fast_T_inv (T)
    R = T(1:3,1:3);
    t = T(1:3,t);

    inv_T = eye(4);
    inv_T(1:3,1:3) = R';
    inv_T(1:3,4) = - R' * t;
end