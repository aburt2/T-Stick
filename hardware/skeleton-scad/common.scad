// units are in mm in keeping with OpenSCAD STL export behavior
// multiplying a dimension in inches by the `inch` variable
// produces the equivalent dimension in mm
inch = 25.4;

thick = 0.6; // 3D printer layer height / line thickness. This should not be changed.

// ABS pipe standard dimensions for nominal 1.25 inch pipe
// diameters
pipe_OD = 1.660 * inch;
pipe_ID = 1.385 * inch;
// radii
pipe_OR = pipe_OD/2;
pipe_IR = pipe_ID/2;

// PCB dimensions and mounting hole spacing
PCBX = 6.400 * inch;
PCBY = 1.250 * inch;
PCBZ = 1.6; // thickness, confirm with PCB fab service
mount_hole_spacing = 0.500 * inch;
mount_hole_start = 0.250 * inch;

// to facilitate 3D printing the skeleton for a single T-Stick
// is broken into several parts, which I here refer to as "modules"
skeleton_module_length = 6 * inch;
skeleton_face_thickness = 3 * thick; // 1.8mm
connector_thickness = 3 * thick; // 1.8mm
connector_width = 15;
connector_length = skeleton_module_length*2/3;
// right angle triangle with base PCBY/2 and hypotenuse pipe_IR has height...
cylinder_top_cut_height_offset = sqrt(pipe_IR*pipe_IR - (PCBY/2)*(PCBY/2));
skeleton_module_total_thickness = pipe_IR - cylinder_top_cut_height_offset;

// hardware
// these should probably be oversized or parameterized
// to allow press fit vs loose fit
M3_hole_diameter = 3;
M3_hole_diameter_loose = 3.3;
M3_nut_height = 2.4;
M3_head_diameter = 7;
M3_head_height = 3;
M3_nut_circumcircle_diameter = 6;
heatshrink_thickness = 5/8;

module M3_nut_volume(rotation=0, scaling=1)
{
    scale([scaling, scaling, scaling]) rotate([0,0,rotation])
        cylinder(h = M3_nut_height, d = M3_nut_circumcircle_diameter, $fn=6);
}

module pipe_volume(radius, length=skeleton_module_length)
{
    translate([0,0,cylinder_top_cut_height_offset])
            rotate([0,90,0])
                cylinder(h = length, r = radius);
}

module pipe_ID_volume(length=skeleton_module_length)
{
    pipe_volume(pipe_IR, length);
}

module pipe_OD_volume(length=skeleton_module_length)
{
    pipe_volume(pipe_OR, length);
}

module pipe_top_area()
{
    translate([-500,-500,0])
        cube([1000,1000,1000]);
}

module skeleton_module_volume(length=skeleton_module_length)
{
    skeleton_fit_shrink = 0.4; // so that the skeleton fits flush inside the pipe
    difference()
    {
        pipe_volume(pipe_IR-skeleton_fit_shrink, length);
        pipe_top_area();
    }
}

module screw_holes(diameter=M3_hole_diameter)
{
    union()
    {
        for(x = [mount_hole_start : mount_hole_spacing : skeleton_module_length - 1])
        {
            translate([x, 0, -500])
                cylinder(h = 1000, d = diameter, $fn=20);
        }
    }
}

axial_screw_hole_offset = cylinder_top_cut_height_offset-pipe_ID/2+M3_head_diameter/2;
module axial_screw_hole(diameter=M3_hole_diameter, length=1000, center=true, xoffset=0)
{
    translate([xoffset,0,axial_screw_hole_offset])
        rotate([0,90,0])
            cylinder(length, r=diameter/2, $fn=20, center=center);
}

module top_axial()
{
    translate([0,0,cylinder_top_cut_height_offset])
        rotate([180,0,0])
            translate([0,0,-cylinder_top_cut_height_offset])
                children();
}
