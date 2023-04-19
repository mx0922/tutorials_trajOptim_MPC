function DrawPolygon(vert,face,col)

if nargin == 2
    color = [0.5 0.5 0.5];
    h = patch('faces',face','vertices',vert','FaceColor',color);   
else
    color = ones(size(face,2),1) * col;
    faceColor = [190 190 190] / 255;
    h = patch('Vertices',vert','Faces',face','FaceVertexCData',color,'FaceColor',faceColor);
end

end
