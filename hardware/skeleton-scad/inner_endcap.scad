include <common.scad>

cap_ring_thickness = 1/4*inch;
cap_cap_spacing = 0.1;

buttonX = 8;
buttonY = buttonX;
buttonZ = 4;
button_vertical_offset = 8;
button_horizontal_offset = 12;
button_leg_slot_offset = 2; // offset in from left/right side of button recess
button_leg_slot_width = 2;
button_leg_slot_length = buttonY+2;
button_pocket_min_wall = 1*thick;
endcap_min_wall = 4*thick;

USB_cutout_width = 7.4;
USB_cutout_length = 13.5;
USB_cutout_corner_radius = 3;
USB_cutout_vertical_offset = 13.3;
USB_cutout_horizontal_offset = 0;//-0.5*inch/10;

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

module button_leg_slot()
{
    translate([ buttonZ-(button_pocket_min_wall)
              , button_leg_slot_offset-(button_leg_slot_width/2)
              , -(button_leg_slot_length/2)
              ])
        cube([ 100
             , button_leg_slot_width
             , button_leg_slot_length
             ]);
}

module button_leg_slots()
{
    button_leg_slot();
    mirror([0,1,0]) button_leg_slot();
}

module button_body_slot()
{
    translate([ -(100)
          , -buttonX/2
          , -buttonY/2
          ])
        cube([buttonZ+100, buttonX, buttonY]);
}

module button_slot()
{
    translate([ 0
              , button_horizontal_offset
              , button_vertical_offset])
        union()
        {
            button_body_slot();
            button_leg_slots();
        }
}

module button_pocket()
{
    m = button_pocket_min_wall;
    x = buttonX + 2 * m;
    y = buttonY + 2 * m;
    z = buttonZ + m;
    difference()
    {
        translate([ 0
                  , button_horizontal_offset-x/2
                  , button_vertical_offset-y/2
                  ])
            cube([z, x, y]);
        button_slot();
    }
}

module endcap_ring()
{
    pipe_volume(pipe_OR, cap_ring_thickness);
}

module screw_tab()
{
    r = M3_head_diameter/2+0.5;
    difference() { union() {
        translate([0,0,axial_screw_hole_offset]) rotate([0,90,0]) cylinder(cap_ring_thickness, r, r, $fn=25);
        translate([0,-r,axial_screw_hole_offset]) mirror([0,0,1])
            cube([cap_ring_thickness, 2*r, pipe_OR-abs(axial_screw_hole_offset)-cylinder_top_cut_height_offset-2]);
    } 
        translate([cap_cap_spacing+cap_ring_thickness-M3_nut_height, 0,axial_screw_hole_offset])
            rotate([0,90,0]) scale([1,1,2]) M3_nut_volume(90);
        axial_screw_hole();
    }
}

module screw_tabs()
{
    screw_tab();
    top_axial() screw_tab();
}

module inner_endcap_alignment_bumps()
{
    translate([cap_ring_thickness,0,0]) intersection()
    {
        difference()
        {
            pipe_volume(pipe_IR, thick);
            pipe_volume(pipe_IR-thick, thick);
        }
        screw_tabs();
    }
}

module inner_endcap()
{
    union()
    {
        difference()
        {
            endcap_ring();
            pipe_volume(pipe_IR);
        }
        screw_tabs();
        inner_endcap_alignment_bumps();
    }
}

module outer_endcap_screw_hole()
{
    union()
    {
        axial_screw_hole(M3_hole_diameter_loose);
        axial_screw_hole(M3_head_diameter, M3_head_height, false, -cap_ring_thickness-cap_cap_spacing);
    }
}

module plain_outer_endcap()
{
    difference()
    {
        translate([-cap_ring_thickness-cap_cap_spacing,0,0])
            endcap_ring();
        outer_endcap_screw_hole();
        top_axial() outer_endcap_screw_hole();
    }
}

module pcb_fixture_bar()
{
    l = 0.5*inch;
    w = 8;
    h = 4;
    translate([-cap_cap_spacing,-w/2,PCBZ])
        cube([l+cap_ring_thickness+cap_cap_spacing,w,h]); // confirm by measure if this will fit before printing it!
}

module proximal_outer_endcap()
{
    union()
    {
        difference()
        {
            union()
            {
                plain_outer_endcap();
                pcb_fixture_bar();
            }
            USB_cutout();
            
            translate([cap_ring_thickness+heatshrink_thickness,0,0])
                screw_holes();
            button_slot();
        }
        translate([-cap_ring_thickness-cap_cap_spacing,0,0])
            button_pocket();
    }
}

//proximal_outer_endcap();
inner_endcap();
//translate([12*inch,0,0]) rotate([0,0,180])
//{
//    translate([-cap_cap_spacing,0,0]) plain_outer_endcap();
//    inner_endcap();
//}
