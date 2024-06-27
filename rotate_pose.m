function adjusted_pose = rotate_pose(pose_matrix, target_angle_deg, axis)
    % Extract rotation matrix and translation vector from pose matrix
    rotationMatrix = pose_matrix(1:3, 1:3);
    translationVector = pose_matrix(1:3, 4);

    % Convert the rotation matrix to Euler angles
    eulerAngle = rotm2eul(rotationMatrix, 'XYZ');
    angles_deg = rad2deg(eulerAngle);

    % Calculate the angle difference in radians based on the specified axis
    if axis == 'x'
        theta_rad = deg2rad(target_angle_deg - angles_deg(1));
        R = [1, 0, 0;
             0, cos(theta_rad), -sin(theta_rad);
             0, sin(theta_rad), cos(theta_rad)];
    elseif axis == 'y'
        theta_rad = deg2rad(target_angle_deg - angles_deg(2));
        R = [cos(theta_rad), 0, sin(theta_rad);
             0, 1, 0;
             -sin(theta_rad), 0, cos(theta_rad)];
    elseif axis == 'z'
        theta_rad = deg2rad(target_angle_deg - angles_deg(3));
        R = [cos(theta_rad), -sin(theta_rad), 0;
             sin(theta_rad), cos(theta_rad), 0;
             0, 0, 1];
    else
        error("Incorrect axis. Please select 'x', 'y', or 'z'.")
    end

    % Update the orientation
    new_orientation = R * rotationMatrix;
    
    % % Update the location of the camera position based on the new orientation
    % updated_translation = R * translationVector;

    % Construct the new pose matrix
    adjusted_pose = eye(4);
    adjusted_pose(1:3, 1:3) = new_orientation;
    adjusted_pose(1:3, 4) = translationVector;
end
