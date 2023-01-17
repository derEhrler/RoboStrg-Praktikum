clear all;
fileID = fopen('data5.txt');
formatSpec = '%f';
dataVector = fscanf(fileID, formatSpec);
t_Schritt = 1/100;
t = zeros(size(dataVector, 1), 1);
for i = 1:size(dataVector)
    t(i,1) = (i-1)*t_Schritt;
end
plot(t, dataVector);
