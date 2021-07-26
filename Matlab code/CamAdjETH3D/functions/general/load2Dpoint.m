function [point2D, CamID] = load2Dpoint(image_name, gt_data)

N = length(gt_data);
point2D = [];
CamID = [];

for ii = 5:2:N
    raw_data = gt_data{ii};
    celled_data = split(raw_data,' ');
    img_name = celled_data{10};
    if contains(img_name, image_name)
        CamID = str2double(celled_data{1});
        point2D_raw_data = gt_data{ii+1};
        point2D_list = split(point2D_raw_data,' ');
        for jj = 1:3:length(point2D_list)
            point2D = [point2D; str2double(point2D_list{jj}), str2double(point2D_list{jj+1})];
        end
        break;
    end
end

return