%% Examples of images used for the project
%Im = imread('./Images/A2@2x.png'); % A2-sized paper x2
%Im = imread('./Images/A4_big.png'); % A4-sized paper centered

%% Plotting points from image
Im = imread('./Images/A4_big.png'); % For testing
CoordinateMatrix = pic2points(Im,0.5);
scatter(CoordinateMatrix (:,1), CoordinateMatrix (:,2),'.');

%% To extract points data to excel files
%xlswrite('./excel_data/F.xlsx',{'x','y'} ,'Sheet 1','A1')
%xlswrite('./excel_data/F.xlsx', [(CoordinateMatrix (:,1)),(CoordinateMatrix (:,2))],'Sheet 1','A2')
