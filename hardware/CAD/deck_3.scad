include <common.scad>


module deck_3_base() {
    difference() {
        // Base circular disc
        cylinder(h = deck_thickness, d = third_deck_diameter);

        // Standoff clearance holes
        standoff_holes_lidar();

        // RPLIDAR C1 mounting: 4 M2.5 holes on a 55.6 x 55.6 mm square
        for (dx = [-lidar_hole_offset, lidar_hole_offset])
            for (dy = [-lidar_hole_offset, lidar_hole_offset])
                translate([dx, dy, 0]) drill(lidar_hole_d);

        // USB cable pass-through behind LiDAR
        translate([lidar_usb_cutout_x, lidar_usb_cutout_y, 0])
            slot(lidar_usb_cutout_w, lidar_usb_cutout_h);
    }
}


deck_3_base();


