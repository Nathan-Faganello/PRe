boxHeight = 30;
boxDimension = 28;
boxWidth = boxDimension;
boxDepth = boxDimension;
boxThickness = 3;
guideHeight = 20;
guideAngle = 30;

guideLength = guideHeight/cos(guideAngle);
guideWidth = guideHeight*tan(guideAngle);

translate([0,(boxDepth+guideWidth+boxThickness*cos(guideAngle))/2,boxHeight+(guideHeight+boxThickness*sin(guideAngle))/2])
rotate(-guideAngle,[1,0,0])
cube([boxWidth,boxThickness,guideLength],center = true);

translate([0,-(boxDepth+guideWidth+boxThickness*cos(guideAngle))/2,boxHeight+(guideHeight+boxThickness*sin(guideAngle))/2])
rotate(guideAngle,[1,0,0])
cube([boxWidth,boxThickness,guideLength],center = true);

translate([(boxWidth+guideWidth+boxThickness*cos(guideAngle))/2,0,boxHeight+(guideHeight+boxThickness*sin(guideAngle))/2])
rotate(guideAngle,[0,1,0])
cube([boxThickness,boxDepth,guideLength],center = true);

translate([-(boxWidth+guideWidth+boxThickness*cos(guideAngle))/2,0,boxHeight+(guideHeight+boxThickness*sin(guideAngle))/2])
rotate(-guideAngle,[0,1,0])
cube([boxThickness,boxDepth,guideLength],center = true);
