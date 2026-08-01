// Shared parameters for the Idefix chassis
// EVERY value in this project lives here. Change once, rebuild everywhere.


//  URDF-LOCKED (do not change without updating URDF) 
wheel_radius       = 22.0;    // mm
track_width        = 216.0;   // mm, centreline to centreline
deck_radius        = 125.0;   // mm, base_radius in URDF
deck_diameter      = 2 * deck_radius;
third_deck_diameter= deck_radius;
floor_gap          = 45.0;


//  Print / material 
deck_thickness     = 3.0;     // mm
$fn                = 120;      // circle resolution 

//  Fastener clearance holes (through the printed part) 
m2_5_clear         = 2.9;     // mm
m3_clear           = 3.4;     // mm
m4_clear           = 4.5;     // mm



// Standoff columns
standoff_r            = 115;     // mm, from deck centre
standoff_r_lidar      = 45;     // mm, from deck centre
standoff_angles       = [90, 210, 330];   // degrees
standoff_angles_lidar = [120, 230, 360];   // degrees
standoff_hole_d       = m3_clear;
standoff_base         = 3; //mm




// DECK 1 (bottom) component placements

// Caster position
caster_x                 = 0;   
caster_y                 = 110;
caster_hole_spacing      = 40;     // mm
caster_hole_d            = m4_clear;

// Caster geometry (for the pedestal module below)
caster_pedestal_h        = 2;      // mm

// Custom PCB mounting holes (183 x 131 mm) hole pattern
pcb_hole_positions = [
    [-46.5, -47  ],
    [ 47.5, -32.5],
    [-46.5,  46],
    [ 47.5,  47.5]
];
pcb_hole_d = m3_clear;
// PCB raised mounting bosses (PCB rests on these)
pcb_standoff_h     = 5;    // mm, how far the PCB sits above deck top
pcb_standoff_d     = 7;    // mm, outer diameter of the boss

// Battery pocket 
battery_x          = 0;   // mm
battery_y          = 72;
battery_length     = 105;   // mm, along X (fore-aft)

battery_width      = 33;    // mm, along Y (lateral)
battery_height     = 22;    // mm, along Z (height of the battery)
battery_slot_depth = 1.0;   // mm, shallow indent so the pack seats 



// Wheel notches at deck edge (top and bottom of deck circle)
// Wheels are at X = +/- 103 mm, wheel disc extends Y = +/- 22 mm.
// Notch is a rectangular cutout in the deck edge.
notch_width_y      = 50;    // mm, wheel diameter + 6 mm clearance
notch_depth_x      = 22;    // mm, from X = +/-103 to X = +/-125 (edge), wheel width + 5 mm clearance

// Motor bracket bolt holes 
motor_bracket_hole_positions = [
    [-95, -9 ],
    [ 95, -9 ],
    [-95,  9 ],
    [ 95,  9 ]
];





// DECK 2 (middle) component placements


// Raspberry Pi 5 mounting holes
pi_x               = 0;     // mm, centred
pi_y               = 0;
pi_hole_dx         = 58 / 2;   // = 29 mm from Pi centre
pi_hole_dy         = 49 / 2;   // = 24.5 mm from Pi centre
pi_hole_d          = m2_5_clear;





plate_height          = 20;    // mm
plate_width           = 90;    // mm
plate_thickness       = 3;     // mm

// Camera support base mounting (front edge of deck 2)
// The vertical camera plate bolts down to deck 2 through these holes.
camera_support_x       = 0;   // mm, at front edge (adjust to match your plan)
camera_support_y       = 115;
camera_support_hole_spacing_y = 75;   // mm, between the two base tabs --> miss the z position = 9mm
camera_support_hole_d  = m4_clear; 



// DECK 3 (top) component placements

esp32_usb_cutout_x = 0;
esp32_usb_cutout_y = 50;    // mm, at deck 2 front-ish
esp32_usb_cutout_w = 12;    // mm, USB-C connector width + clearance  
esp32_usb_cutout_h = 8;     // mm

// RPLiDAR C1
lidar_hole_offset = 27.8;   // mm, half of 55.6
lidar_hole_d      = m2_5_clear;

// LiDAR USB cable pass-through (behind LiDAR, cable drops to deck 2)
lidar_usb_cutout_x = 0;
lidar_usb_cutout_y = -50;   // mm, behind LiDAR body
lidar_usb_cutout_w = 15;    // mm, USB-A connector width + clearance
lidar_usb_cutout_h = 8;     // mm, USB-A connector height + clearance

// UTILITY MODULES (used Use inside difference() by every deck)

// Punches a through-hole down through the deck.
module drill(d, depth = deck_thickness + 2) {
    translate([0, 0, -1]) cylinder(h = depth, d = d);
}

// Rectangular through-cut. Use inside difference().
module slot(w, h, depth = deck_thickness + 2) {
    translate([-w/2, -h/2, -1]) cube([w, h, depth]);
}

// Punches the 3 standoff column clearance holes.
module standoff_holes() {
    for (a = standoff_angles) {
        translate([standoff_r * cos(a), standoff_r * sin(a), 0])
            drill(standoff_hole_d);
    }
}

module standoff_holes_lidar() {
    for (a = standoff_angles_lidar) {
        translate([standoff_r_lidar * cos(a), standoff_r_lidar * sin(a), 0])
            drill(standoff_hole_d);
    }
}


// One wheel notch at +X (x_sign = 1) or -X (x_sign = -1).
module wheel_notch(x_sign) {
    translate([x_sign * (deck_radius - notch_depth_x/2),
               0,
               deck_thickness/2])
        cube([notch_depth_x, notch_width_y, deck_thickness + 2], center = true);
}

// Two caster mount holes at the specified caster_x, caster_y.
module caster_holes() {
        translate([caster_x + caster_hole_spacing/2, caster_y , 0])
            drill(caster_hole_d);
        translate([caster_x - caster_hole_spacing/2, caster_y, 0])
            drill(caster_hole_d);

}


module caster_pedestal() {
    difference() {
        // Rectangular pedestal, slightly wider than the caster flange
        translate([0, 0, -caster_pedestal_h/2])
        cube([caster_hole_spacing + 10, 20,caster_pedestal_h], center=true);

        // Two caster mounting holes, passing all the way through the deck
            for (dx = [-caster_hole_spacing/2, caster_hole_spacing/2])
                translate([dx, 0, -caster_pedestal_h - 1])
                    cylinder(h = caster_pedestal_h + deck_thickness + 2, d = caster_hole_d);
        
    }
}


module camera_support() {
    // Vertical plate (grows up from Z=0)
    difference() {
        translate([camera_support_x, camera_support_y , plate_height/2])
        cube([plate_width, plate_thickness, plate_height], center = true);

        // First M4 mounting hole
        translate([-camera_support_hole_spacing_y/2, camera_support_y, plate_height/2])
            rotate([90, 0, 0])
                cylinder(h = plate_thickness + 2, d = camera_support_hole_d, center = true);
        
        // Second M4 mounting hole
        translate([camera_support_hole_spacing_y/2, camera_support_y, plate_height/2])
            rotate([90, 0, 0])
                cylinder(h = plate_thickness + 2, d = camera_support_hole_d, center = true);
    }
}

module pi_hole_positions_loop() {
    for (dx = [-pi_hole_dx, pi_hole_dx])
        for (dy = [-pi_hole_dy, pi_hole_dy])
            translate([pi_x + dx, pi_y + dy, 0]) children();
}
