function [pose, image_path] = extract_camera_pose(image_name, gt_data)

N = length(gt_data);
pose.q = [1 0 0 0];
pose.t = [0 0 0];
image_path = [];

for ii = 5:2:N
    raw_data = gt_data{ii};
    celled_data = split(raw_data,' ');
    img_name = celled_data{10};
    if contains(img_name, image_name)
        image_path = celled_data{10};
        qw = str2double(celled_data{2});
        qx = str2double(celled_data{3});
        qy = str2double(celled_data{4});
        qz = str2double(celled_data{5});
        tx = str2double(celled_data{6});
        ty = str2double(celled_data{7});
        tz = str2double(celled_data{8});
        pose.q = [qw,qx,qy,qz];
        pose.t = [tx,ty,tz];
        break;
    end
end

return