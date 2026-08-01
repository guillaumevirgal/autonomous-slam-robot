include <common.scad>



module standoff() {
    
    union(){
        for (a = standoff_angles) {
            translate([standoff_r * cos(a), standoff_r * sin(a), 0])
            cylinder(h = floor_gap + 2 * deck_thickness, d = standoff_hole_d);
            
            //Lower base
            translate([standoff_r * cos(a), standoff_r * sin(a), deck_thickness])
            cylinder(h = standoff_base, d = standoff_hole_d * 1.5);
            
            //Higher base
            translate([standoff_r * cos(a), standoff_r * sin(a), floor_gap])
            cylinder(h = standoff_base, d = standoff_hole_d * 1.5);
            
        }
        
        
        
        for (a = standoff_angles_lidar) {
            translate([standoff_r_lidar * cos(a), standoff_r_lidar * sin(a), floor_gap])
            cylinder(h = floor_gap + 2 * deck_thickness, d = standoff_hole_d);
            
            //Lower base
            translate([standoff_r_lidar * cos(a), standoff_r_lidar * sin(a), floor_gap + deck_thickness])
            cylinder(h = standoff_base, d = standoff_hole_d * 1.5);
            
            //Higher base
            translate([standoff_r_lidar * cos(a), standoff_r_lidar * sin(a), 2 * floor_gap])
            cylinder(h = standoff_base, d = standoff_hole_d * 1.5);
        }
    }
}

standoff();