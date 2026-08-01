include <common.scad>


module deck_2_base() {
    difference() {
        union(){
            // Base circular disc
            cylinder(h = deck_thickness, d = deck_diameter);
            
            camera_support();
            
            //four raised Pi standoffs on top
            pi_hole_positions_loop() cylinder(h = pcb_standoff_h, d = pi_hole_d + 3);
            
        }    
        

        // Standoff clearance holes (same columns as deck 1)
        standoff_holes();
        standoff_holes_lidar();
        
        // Pi 5 mounting holes (58 x 49 mm rectangular pattern)
        pi_hole_positions_loop() drill(pi_hole_d, deck_thickness + pcb_standoff_h + 2);

        // ESP32 USB-C access
        translate([esp32_usb_cutout_x, esp32_usb_cutout_y, 0])
        slot(esp32_usb_cutout_w, esp32_usb_cutout_h);
    }
}



deck_2_base();

