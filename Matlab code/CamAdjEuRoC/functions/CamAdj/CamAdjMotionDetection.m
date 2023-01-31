function motion_flag = CamAdjMotionDetection(mp, mp_tracked, noiseLevel)
motion_flag = 0;
shift_vector = vecnorm(abs(mp - mp_tracked)');
shift_vector = sort(shift_vector,'descend');
shift_distance = mean(shift_vector(1:round(0.2*length(shift_vector))));
if shift_distance > noiseLevel
    motion_flag = 1;
end
return