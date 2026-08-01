include <common.scad>


module deck_1_base() {
    difference() {
        
        union() {
            // Base circular disc
            cylinder(h = deck_thickness, d = deck_diameter);
                
            //Caster pedestal
            translate([caster_x, caster_y, 0])
        caster_pedestal();
                
            //four raised PCB standoffs on top
            for (pos = pcb_hole_positions) {
                translate([pos[0], pos[1], deck_thickness])
                cylinder(h = pcb_standoff_h, d = pcb_standoff_d);
            }
        }
            
            
        // Wheel notches at +X and -X edges of the deck
        wheel_notch(+1);   // notch for left wheel
        wheel_notch(-1);   // notch for right wheel

        // Standoff clearance holes (3 columns)
        standoff_holes();

        // Caster mounting holes
        caster_holes();

        // Custom PCB mounting holes
        for (pos = pcb_hole_positions) {
        translate([pos[0], pos[1], 0]) 
            drill(pcb_hole_d, deck_thickness + pcb_standoff_h + 2);
        }

        // Battery seating indent (shallow)
        translate([battery_x, battery_y, deck_thickness - battery_slot_depth])
            cube([battery_length + 2, battery_width + 2, battery_slot_depth + 2],
                 center = true);

        // Motor bracket bolt holes
        for (pos = motor_bracket_hole_positions) {
            translate([pos[0], pos[1], 0]) drill(m3_clear);
        }
        
        
    }
}


deck_1_base();



