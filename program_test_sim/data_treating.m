%% get_data文件后的数据处理
% 处理get_data文件中的数据，将数据导入到formula_derivation文件中进行处理，构建方程组
data_intrinsic = data_list_intrinsic_parameter;
data_extrinsic = data_list_extrinsic_parameter;
data_intrinsic(1:4,:) = data_intrinsic(1:4,:) * 1000;
data_extrinsic(1:4,:) = data_extrinsic(1:4,:) * 1000;
data_intrinsic = data_intrinsic';
data_extrinsic = data_extrinsic';
data = [data_intrinsic;data_extrinsic];
