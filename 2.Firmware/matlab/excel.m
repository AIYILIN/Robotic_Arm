% 配置输入输出文件名
excel_file = 'JointAngles_20250512_193927.xlsx';   % 输入的Excel文件名
txt_file   = 'output.txt';   % 输出的文本文件名

% 读取Excel数据（跳过标题行，提取数值部分）
[~, ~, raw] = xlsread(excel_file);
data = cell2mat(raw(2:end, 2:7));  % 提取6个关节角度数据

% 创建并打开输出文件
fid = fopen(txt_file, 'w', 'n', 'UTF-8');

% 写入数组声明
fprintf(fid, 'const float joint_angles[%d][6] = \r\n{\r\n', size(data,1));

% 逐行写入数据
for i = 1:size(data,1)
    fprintf(fid, '    { %.8f, %.8f, %.8f, %.8f, %.8f, %.8f }', data(i,:));
    if i ~= size(data,1)
        fprintf(fid, ',');  % 最后一行不加逗号
    end
    fprintf(fid, '\r\n');
end

% 闭合数组
fprintf(fid, '};\r\n');

% 关闭文件
fclose(fid);
disp(['转换完成！已保存至：' txt_file]);