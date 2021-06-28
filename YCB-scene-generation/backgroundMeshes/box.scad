squareDimension = 28;
width = squareDimension;
depth = squareDimension;
height = 30;
thickness = 3;
open = false;


translate([0,0,thickness/2])
cube([width, depth, thickness], center = true);

translate([0,(depth-thickness)/2,height/2])
cube([width, thickness, height], center = true);
if(open) {} else {
    translate([0,-(depth-thickness)/2,height/2])
    cube([width, thickness, height], center = true);
}
translate([(width-thickness)/2,0,height/2])
cube([thickness, depth, height], center = true);
translate([-(width-thickness)/2,0,height/2])
cube([thickness, depth, height], center = true);