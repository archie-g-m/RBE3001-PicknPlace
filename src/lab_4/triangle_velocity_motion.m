function triangle_motion()
    point1 = [128, -100, 77];
    point2 = [64, 110, 105];
    point3 = [137, -13, 223];
    points = [point1; point2; point3];

    move_arm(false, points(3, :))

    for i = 1:3
        velocity_motion(points(i, :))
    end

end
