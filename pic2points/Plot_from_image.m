%Im = imread('./Images/thin-outline3.png'); % For Project

Im = imread('./Images/A2-Eiman.png'); % For testing
CoordinateMatrix = pic2points(Im,0.5);
scatter(CoordinateMatrix (:,1), CoordinateMatrix (:,2),'.');

%xlswrite('./excel_data/F.xlsx',{'x','y'} ,'Sheet 1','A1')
%xlswrite('./excel_data/F.xlsx', [(CoordinateMatrix (:,1)),(CoordinateMatrix (:,2))],'Sheet 1','A2')

