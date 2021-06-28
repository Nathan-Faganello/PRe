$fn = 20;
width = 150;
height = 75;
depth = 75;
thickness = 3;
legThickness = 2;
legMargin = 10;
translate([0,0,-thickness/2])
cube([width, depth, thickness], center = true);

translate([-width/2+legMargin, -depth/2+legMargin,-(height-thickness)/2-thickness])
cylinder(h = height-thickness, d = legThickness, center = true);
translate([width/2-legMargin, -depth/2+legMargin,-(height-thickness)/2-thickness])
cylinder(h = height-thickness, d = legThickness, center = true);
translate([-width/2+legMargin, depth/2-legMargin,-(height-thickness)/2-thickness])
cylinder(h = height-thickness, d = legThickness, center = true);
translate([width/2-legMargin, depth/2-legMargin,-(height-thickness)/2-thickness])
cylinder(h = height-thickness, d = legThickness, center = true);