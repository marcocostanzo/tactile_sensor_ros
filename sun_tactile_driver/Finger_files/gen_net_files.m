clear all,close all,clc

%system('rm *.txt');

load('H004_denkmit_NET_90  90  90  90  90  90_decim_5')
%load net3025_fs50_05_09
net = train_out.net;
settings_i = train_out.settings_i;
settings_t = train_out.settings_t;


numLayers = net.numLayers;

sizeLayers = zeros(numLayers,1);

try
remove = net.inputs{1}.processSettings{1}.remove;
catch
   remove = []; 
end

for jj = 0:(numLayers-1)
    
    if(jj==0)
       W = net.IW{1};
       for(uu = remove)
          W = [W(:,1:remove-1) , zeros(size(W,1),1) , W(:,remove:end)]; 
       end
       dimInput = size(W,2);
    else
       W = net.LW{jj+1,jj}; 
    end
    
    b = net.b{jj+1};
    
    sizeLayers(jj+1) = length(b);
    
    eval(['save -ascii -double W' num2str(jj) '.txt W'])
    eval(['save -ascii -double b' num2str(jj) '.txt b'])
    
end



mMi = double([settings_i.xmin settings_i.xmax]);
mMi(remove,1) = -1;
mMi(remove,2) = 1;
save -ascii -double mM_i.txt mMi

mMo = double([settings_t.xmin settings_t.xmax]);
save -ascii -double mM_o.txt mMo

fileID = fopen('meta.txt','w');
fprintf(fileID,'%d\n', numLayers);
fprintf(fileID,'%d\n', dimInput);
fclose(fileID);

fileID = fopen('sizeLayers.txt','w');
for ii = 1:numLayers
    fprintf(fileID,'%d\n', sizeLayers(ii));
end
fclose(fileID);

fileID = fopen('pca.txt','w');
if train_out.pca
    fprintf(fileID,'%d\n', size(train_out.Ureduce,2));
    fclose(fileID);
    pca_mean = train_out.pca_mean;
    save -ascii -double pca_mean.txt pca_mean
    Ureduce = train_out.Ureduce;
    save -ascii -double Ureduce.txt Ureduce
else
    fprintf(fileID,'%d\n', 0);
    fclose(fileID);
end



