include <common.scad>

skeleton_connector_tolerance = 0.8;

module nuts(z, r=0, s=1)
{
    union()
    {
        for(x = [mount_hole_start : mount_hole_spacing : skeleton_module_length - 1])
        {
            translate([x, 0, z]) 
                M3_nut_volume(r,s);
        }
    }
}

module middle_nuts()
{
    nuts(-(M3_nut_height + skeleton_face_thickness - thick));
}

module connector_nuts()
{
    translate([0,0,-(skeleton_module_total_thickness-M3_nut_height-0.5)]) scale([1,1,1000]) mirror([0,0,1]) nuts(0,90,1.1);
}

module connector_axial_nut_slot()
{
    scale([1.1,1.1,1]) hull()
    {
        translate([0, 0, -(skeleton_module_total_thickness-M3_head_diameter/2)]) rotate([0,90,0]) M3_nut_volume();
        translate([0, 0, -1000]) rotate([0,90,0]) M3_nut_volume();
    }
}

module connector_axial_nut_slots(length)
{
    wall=2*thick;
    union()
    {
        translate([wall,0,0]) connector_axial_nut_slot();
        translate([length - wall,0,0]) mirror([1,0,0]) connector_axial_nut_slot();
    }
}

module bottom_holes()
{
    union()
    {
        mirror([0,0,1]) for(x = [mount_hole_start : mount_hole_spacing : skeleton_module_length - 1])
        {
            translate([ x-M3_nut_circumcircle_diameter/2
                      , -connector_width/2
                      , skeleton_face_thickness + connector_thickness])
                cube([M3_nut_circumcircle_diameter, connector_width, 1000]);
        }
    }
}

module connector_volume(length=skeleton_module_length/2)
{
    width = connector_width;
    thickness = M3_head_diameter+1;
    vertical_offset = -(skeleton_module_total_thickness);
    intersection()
    {
        translate([0,-width/2,vertical_offset]) cube([length, width, thickness]);
        skeleton_module_volume();
    }
}

module connector(length)
{
    difference()
    {
        connector_volume(length);
        screw_holes($fn=20);
        connector_nuts();
        connector_axial_nut_slots(length);
        axial_screw_hole();
    }    
}

module skeleton()
{
    tol = skeleton_connector_tolerance;
    difference()
    {
        skeleton_module_volume();
        translate([0,0,-tol]) connector_volume(skeleton_module_length);
        translate([tol,0,-tol]) connector_volume(skeleton_module_length);
        translate([0,tol,-tol]) connector_volume(skeleton_module_length);
        translate([0,0,0]) connector_volume(skeleton_module_length);
        translate([tol,0,0]) connector_volume(skeleton_module_length);
        translate([0,tol,0]) connector_volume(skeleton_module_length);
        screw_holes($fn=20);
        middle_nuts();
    }
}

rotate([180,0,0]) skeleton();
//translate([0,40,0]) rotate([180,0,0]) connector(inch);
