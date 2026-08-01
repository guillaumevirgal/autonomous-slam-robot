include <common.scad>
use <deck_1.scad>
use <deck_2.scad>
use <deck_3.scad>
use <standoff.scad>

module Idefix() {
    union() {
        color("silver")
        deck_1_base();
        
        color("gold")
        translate([0, 0, floor_gap]) 
        deck_2_base();
        
        color("tomato")
        translate([0, 0, 2 * floor_gap]) 
        deck_3_base();
        standoff();
    }
}


Idefix();



//Those are just to visualize the main components, comment for the STL

%translate([deck_radius - notch_depth_x/2,
               0,
               deck_thickness/2])
rotate([0, 90, 0])
cylinder(h = 20, d = 55);

%translate([-deck_radius - notch_depth_x/2,
               0,
               deck_thickness/2])
rotate([0, 90, 0])
cylinder(h = 20, d = 55);


%translate([battery_x, battery_y, deck_thickness + battery_height/2])
     cube([battery_length, battery_width, battery_height], center = true);  // Battery

%translate([-50, -50, deck_thickness + pcb_standoff_h], center = true)
     cube([100, 100, 5]);  // rough PCB envelope
     
     

%translate([pi_x - 12, pi_y, floor_gap + deck_thickness + pcb_standoff_h])
    cube([85, 58, 3], center = true);  // Pi 5 board envelope
 

%translate([0, 125, floor_gap + deck_thickness + 13.5])
    cube([90, 17, 27], center = true);  // Camera
     
%translate([0, 0, 2 * floor_gap + deck_thickness + 41.3/2])
     cube([55.6, 55.6, 41.3], center = true);  // LiDAR body envelope