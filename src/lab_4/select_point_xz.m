function select_point_xz
    iterative_velocity_motion([0 0 0], [100 0 195])
    old_point = [100 0 195];
    
    for i = 1:3
        [x_pos, z_pos] = ginput

        iterative_velocity_motion(old_point, [x_pos, 0, z_pos])

        old_point = [x_pos 0 z_pos];

    end
end
