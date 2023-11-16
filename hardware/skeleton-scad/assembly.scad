include <common.scad>
use <skeleton.scad>
use <endcap.scad>
use <connector.scad>

N_modules = 2;

proximal_endcap();
skeleton();

for(i = [1:1:N_modules])
{
    translate([skeleton_module_length,0,0]) skeleton();
    translate([connector_length,0,0]) connector(connector_length);
}

translate([N_modules*skeleton_module_length,0,0]) rotate([0,0,180])
{
    distal_endcap();
    distal_endcap_connector();
}