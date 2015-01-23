clc
clear
format long % displays values with higher precision

file = '~/Desktop/slang/test.txt';

content = fileread(file); % reads the file into a long string

% extracts "heading: ....."
regexpResult  = regexp(content, 'heading: (-?\d+.?\d+)', 'match'); 
 
sizeOfTokens = size(regexpResult);
valuesOfHeadings = [];
for i = 1:sizeOfTokens(1,2)
    temp = textscan(regexpResult{i}, '%*s%n'); % extracts value of heading
    valuesOfHeadings = [valuesOfHeadings, temp{1}];
end


sizeOfX = size(valuesOfHeadings);
x = linspace(1,sizeOfX(1,2), sizeOfX(1,2));
scatter(x,valuesOfHeadings);







