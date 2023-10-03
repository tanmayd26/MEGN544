function interpolated_pose = interpolateTransformationMatrix(initial_pose, final_pose, q_curr, q_next, dt)
    % Use a linear interpolation between initial and final poses based on joint angles
    interpolated_pose = initial_pose + (final_pose - initial_pose) * (q_next - q_curr) / dt;
end