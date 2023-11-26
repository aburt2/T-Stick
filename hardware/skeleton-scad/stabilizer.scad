include <common.scad>

stabilizer_min_wall = 4*thick;
stabilizer_thickness = M3_head_diameter+stabilizer_min_wall;
stabilizer_pcb_clearance = 10;

module spring_ring()
{
    difference()
    {
        pipe_ID_volume();
        scale([1,0.95,0.95]) pipe_ID_volume();
    }
}

module column()
{
     translate([-stabilizer_thickness/2,0,0]) intersection()
    {
        translate([0,-stabilizer_thickness/2,PCBZ])
            cube([stabilizer_thickness, stabilizer_thickness, 1000]);
        pipe_ID_volume();
    }
}

module spring_leaf()
{
    translate([-stabilizer_thickness/2,0,0]) intersection()
    {
        spring_ring();
        translate([0,-500,stabilizer_pcb_clearance])
            cube([stabilizer_thickness,1000,1000]);
    }
}

module column_screw_hole()
{
    union()
    {
        cylinder(h = 1000, d = M3_hole_diameter, $fn=10);
        translate([0,0,stabilizer_min_wall+PCBZ]) intersection()
        {
            cylinder(h = 1000, d = M3_head_diameter);
            pipe_top_area();
        }
    }
}

module stabilizer()
{
    difference()
    {
        union()
        {
            spring_leaf();
            column();
        }
        column_screw_hole();
    }
}

stabilizer();