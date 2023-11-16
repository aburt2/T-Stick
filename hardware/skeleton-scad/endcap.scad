include <common.scad>

endcap_min_wall = 4*thick;
endcap_IR_fit_tolerance = 0.3;
endcap_OR = pipe_OR + 8*thick;
endcap_IR = pipe_OR + heatshrink_thickness + endcap_IR_fit_tolerance;

module endcap_volume()
{
    translate([-endcap_min_wall,0,0]) intersection()
    {
        pipe_volume(endcap_OR);
        translate([0, -500, -500]) cube([inch + endcap_min_wall, 1000, 1000]);
    }
}

module endcap_cavity()
{
    pipe_volume(endcap_IR);
}

module plain_endcap()
{
    difference()
    {
        endcap_volume();
        axial_screw_hole();
        endcap_cavity();
    }
}
