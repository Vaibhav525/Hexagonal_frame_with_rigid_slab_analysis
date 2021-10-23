
%Master script for program execution

%STEP1: Take input of geometry of structure: [Beam_length ,Coulumn Height]
%Executing app for taking layout info
app_Layout=Layout; 
waitfor(app_Layout);
fprintf(L);
%L,H now contain Beam Length and Column Height

%STEP2: Take input for section geometry of beam and column
%Executing beamprop
app_beamprop=beamprop;
waitfor(app_beamprop);
Area


