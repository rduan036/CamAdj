function [point3D, colors] = load3DpointOfTwoView(Cam1_id, Cam2_id, data, headerlinesIn)

point3D = [];
colors = [];
N = length(data);
for ii = headerlinesIn+1 : N
    data_row = data{ii};
    celled_data = split(data_row,' ');
    data_array = [];
    for jj = 1:length(celled_data)
        data_array = [data_array; str2double(celled_data{jj})];
    end
    cams_id = data_array(9:2:end);
    if ~isempty(find(cams_id == Cam1_id)) && ~isempty(find(cams_id == Cam2_id))
        point3D = [point3D, data_array(2:4)];
        colors = [colors, data_array(5:7)];
    end
    
end

point3D = point3D';
colors = uint8(colors');

return