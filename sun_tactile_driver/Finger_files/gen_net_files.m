clear all, close all, clc

%Insert here cells removed before the train process
%This vector must be sorted
removed_cells = [];

output_file = 'NET_FILE';

load('H004_rigid_NET_89  89  89  89  89  89_decim_11')

%% Extract net
net = train_out.net;

%Case of bad cells
try
remove = net.inputs{1}.processSettings{1}.remove;
catch
   remove = []; 
end

W_cell = {};
b_cell = {};
activation_fun_cell = {};

%For all the layers
for jj = 1:net.numLayers
   
    %Input Layer
    if(jj==1)
       W_tmp = net.IW{1};
       for uu = remove
          W_tmp = [W_tmp(:,1:remove-1) , zeros(size(W_tmp,1),1) , W_tmp(:,remove:end)]; 
       end
       W_cell{1} = W_tmp;
    else
       W_cell{jj} = net.LW{jj,jj-1}; 
    end
    
    b_cell{jj} = net.b{jj};
    
    activation_fun_cell{jj} = net.layers{jj}.transferFcn;
    
end

%% Fix min max remove
train_out.settings_i.xmin(remove) = -1;
train_out.settings_i.xmax(remove) = 1;

%% File

NO_LAYER_CODE = 0;
PCA_LAYER_CODE = 1;
MAPMINMAX_LAYER_CODE = 2;
MAPMINMAX_REVERSE_LAYER_CODE = 3;
FULLY_CONNECTED_LAYER_CODE = 4;

SIGMOID_FUNCTION_CODE = 1;
LINEAR_FUNCTION_CODE = 2;


print_format = '%.10E\n';

fid = fopen([output_file '.txt'], 'wt');

%% First Layer is pca?
if train_out.pca
   %removed cells
   for c = removed_cells
       train_out.pca_mean = [train_out.pca_mean(1:c-1); 0; train_out.pca_mean(c:end) ];
       train_out.Ureduce = [train_out.Ureduce(1:c-1,:); zeros(1,size(train_out.Ureduce,2)); train_out.Ureduce(c:end,:)];
   end
   %Code PCA
   fprintf(fid, print_format,  PCA_LAYER_CODE  );
   %NumElements of pca_mean = size of input
   fprintf(fid, print_format,  numel(train_out.pca_mean) );
   %NumCols of Ureduce = NumRows of Ureduce.T() = size of output
   fprintf(fid, print_format,  size(train_out.Ureduce,2) );
   %pca_mean
   fprintf(fid, print_format,  train_out.pca_mean );
   %Ureduce (fprintf is column-w so i will transpose)
   fprintf(fid, print_format,  train_out.Ureduce' );
end

%% MapMinMax Inputs
%Code MapMinMax
fprintf(fid, '%.10E\n',  MAPMINMAX_LAYER_CODE  );
%Num Elements
fprintf(fid, '%.10E\n',  numel(train_out.settings_i.xmin)  );
%Min
fprintf(fid, '%.10E\n',  train_out.settings_i.xmin );
%Max
fprintf(fid, '%.10E\n',  train_out.settings_i.xmax  );

%% Fully Connected Layers
for i=1:numel(W_cell)
    
    %Code FULLY_CONNECTED_LAYER
    fprintf(fid, '%.10E\n',  FULLY_CONNECTED_LAYER_CODE  );
    %Size Input
    fprintf(fid, '%.10E\n',  size(W_cell{i},2)  );
    %Num Neurons
    fprintf(fid, '%.10E\n',  size(W_cell{i},1)  );
    %Activation Function Type
    switch activation_fun_cell{i}
        case 'tansig'
            %Sigmoid
            fprintf(fid, '%.10E\n',  SIGMOID_FUNCTION_CODE  );
        case 'purelin'
            %Linear
            fprintf(fid, '%.10E\n',  LINEAR_FUNCTION_CODE  );
        otherwise
            fclose(fid);
            error(['Non valid activation fun ', activation_fun_cell{i} ])
    end
    
    %w (fprintf is column-w so i will transpose)
    fprintf(fid, print_format,  W_cell{i}' );
    %bias
    fprintf(fid, print_format,  b_cell{i} );
    
end

%% MapMinMax Reverse
%Code MapMinMax
fprintf(fid, '%.10E\n',  MAPMINMAX_REVERSE_LAYER_CODE  );
%Num Elements
fprintf(fid, '%.10E\n',  numel(train_out.settings_t.xmin)  );
%Min
fprintf(fid, '%.10E\n',  train_out.settings_t.xmin );
%Max
fprintf(fid, '%.10E\n',  train_out.settings_t.xmax  );

%% End Code
%Code MapMinMax
fprintf(fid, '%.10E\n',  NO_LAYER_CODE  );

fclose(fid);