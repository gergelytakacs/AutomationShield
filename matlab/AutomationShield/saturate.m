function u = saturate(u,umin,umax)
    if u>=umax
         u=umax;
    elseif u<=umin
         u=umin;
    end    
end