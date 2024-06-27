function save_markers(Markers , output_file)
% This function saves a struct to csv file

% It expects as struct with multiple parameters with multiple matrices
% matrices
% Markers.orientation = 3x3 matrix
% Markers.transformation_to_master = 4x4 matrix
% output_file = name of the output file

for i=1:size(Markers,2)
    temp1 = mat2str(Markers(i).orientation,4);
    temp1 = strrep(temp1,' ',',');
    % temp1 = strrep(temp1,';','|');
    Markers(i).orientation = temp1;
    temp2 = mat2str(Markers(i).transformation_to_master,4);
    temp2 = strrep(temp2,' ', ',');
    % temp2 = strrep(temp2,';','|');
    Markers(i).transformation_to_master = temp2;
end

writetable(struct2table(Markers),output_file,"Delimiter",',');

end