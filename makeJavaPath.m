% Script file to create Java table for a path
thePath = input('Enter the name of the path: ','s');
disp('Select the text file to convert.');
[fn,dn] = uigetfile('*.*');

% Open output file for Java 
fid2 = fopen([thePath,'.java'],'w');

% Open the file and read one line at a time until end
fid = fopen([dn fn]);

% File format - what computer you got?
if ispc
    ending = [10 13];
else
    ending = 10;
end

fgetl(fid); % This is the label line from the file
tline = fgetl(fid); % Next line
while ( -1 ~= tline )  % while loop works on valid lines
    vect = str2num(tline);
    nextString = [thePath,sprintf('.add(new TargetPosition2D(%f,%f,Math.toRadians(%f),%f));',vect)];
    disp(nextString);
    fwrite(fid2,[nextString,ending]); % adds cr/lf for PCs
    tline = fgetl(fid); % Next line
end

fclose(fid);
fclose(fid2);
clear thePath fn dn fid tline vect nextString ending fid2 ans   
