include <endcap.scad>

buttonX = 8;
buttonY = buttonX;
buttonZ = 4;
button_vertical_offset = 8;
button_leg_slot_offset = 2; // offset in from left/right side of button recess
button_leg_slot_width = 2;
button_leg_slot_length = buttonY+2;
button_pocket_min_wall = 1*thick;

USB_cutout_width = 9;
USB_cutout_length = 15;
USB_cutout_corner_radius = 3;
USB_cutout_vertical_offset = 17;
USB_cutout_horizontal_offset = -0.5*inch/10;

module USB_cutout()
{
    d = 2*USB_cutout_corner_radius;
    r = USB_cutout_corner_radius;
    y = USB_cutout_length-d;
    translate([0,USB_cutout_horizontal_offset,USB_cutout_vertical_offset]) minkowski()
    {
        translate([-500,-y/2,-(USB_cutout_width-d)/2]) cube([1000,y,USB_cutout_width-d]);
        sphere(r);
    }
}

module button_slot()
{
    translate([ -(endcap_min_wall+thick)
              , -buttonX/2
              , button_vertical_offset-buttonY/2
              ])
    cube([buttonZ+thick, buttonX, buttonY]);
}

module button_leg_slot()
{
    translate([ buttonZ-(endcap_min_wall+button_pocket_min_wall)
              , button_leg_slot_offset-(button_leg_slot_width/2)
              , button_vertical_offset-(button_leg_slot_length/2)
              ])
        cube([ 2*button_pocket_min_wall
             , button_leg_slot_width
             , button_leg_slot_length
             ]);
}

module button_pocket()
{
    m = button_pocket_min_wall;
    x = buttonX + 2 * m;
    y = buttonY + 2 * m;
    z = buttonZ + m;
    difference()
    {
        translate([ -(endcap_min_wall)
                  , -x/2
                  , button_vertical_offset-y/2
                  ])
            cube([z, x, y]);
        button_slot();
        button_leg_slot();
        mirror([0,1,0]) button_leg_slot();
    }
}

module FSR_wire_slot()
{
    w = 8;
    t = 3*thick;
    translate([0,-w/2,endcap_IR+cylinder_top_cut_height_offset-1]) cube([1000,w,t+1]);
}

module anti_rotation_screw_hole()
{
    translate([0.5*inch, 0, -1000]) cylinder(d=M3_hole_diameter, h=1000, $fn=20);
    translate([0.5*inch, 0, -1000 - endcap_OR + cylinder_top_cut_height_offset + 4*thick]) cylinder(d=M3_head_diameter, h=1000, $fn=20);
}

module proximal_endcap()
{
    union()
    {
        difference()
        {
            plain_endcap();
            USB_cutout();
            button_slot();
            FSR_wire_slot();
            anti_rotation_screw_hole();
        }
        button_pocket();
    }
}

translate([0,0,endcap_min_wall]) rotate([0,-90,0]) proximal_endcap();
//proximal_endcap();
