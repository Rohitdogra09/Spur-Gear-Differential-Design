--Function defining angle between corresponding radiuses
function involute_angle(r_1,r_2)
    return math.sqrt(((r_2 * r_2) -(r_1 *r_1))/(r_1 *r_1))
end

--Function for finding slope (y2 -y1/x2-x1)
function slope(coords)
    return ((coords[2].y - coords[1].y)/(coords[2].x - coords[1].x))
end

--Function for calculation points of involute curve
function tooth_involute(r_b, angle)                 
    return v(r_b*(math.sin(angle) - angle*math.cos(angle)), 
			r_b*(math.cos(angle) + angle* math.sin(angle)))
end

--Function for mirroring involute points w.r.t y axies
function mirror_(points)
    return v(-points.x,points.y)
end

--Function to rotate points using the rotational matrix
function rotate_points(angle, coord)                                                                                    
    return v(math.cos(angle) * coord.x + math.sin(angle) * coord.y, math.cos(angle) * coord.y - math.sin(angle) * coord.x)
end

--Function for creating cirle
function circle(centre_point,r,th)
    return v(centre_point.x+ r*math.cos(th),centre_point.y+ r*math.sin(th))
end


function gear(z_l,m_t,alpha_t,h_a_coef,h_f_coef,f_r)
    local xy_points = {} --initialization of xy_points table to store xy coordinates of gear
    z=z_l;
    alpha_t_rad=alpha_t*math.pi/180 --pressure angle converted from deg to rad.

    h_a = m_t * h_a_coef; -- Addendum
    h_f = m_t * h_f_coef; -- Dedendum

    d_p = m_t * z; -- Pitch diameter
    r_p = d_p/2; --pitch radius

    d_b = d_p * math.cos(alpha_t_rad); --Base diameter
    r_b = d_b / 2; --base radius
    
    d_a = d_p + 2*h_a; -- Tip diameter
    r_a = d_a / 2; --tip radius

    d_f = d_p - 2*h_f; -- Root diameter
    r_f = d_f / 2; --root radius

    s_0 = m_t * (math.pi/2); -- tooth thickness at pitch circle considering profile shift = 0

    psi_pitch = s_0/r_p -- Tooth thickness half angle 

    inv_alpha_t = math.tan(alpha_t_rad) - alpha_t_rad; -- involute function at pressure angle

    d_TIF = math.sqrt(math.pow(d_p*math.sin(alpha_t_rad)- 2*(h_a-h_f*(1-math.sin(alpha_t_rad))),2) + d_b * d_b)
    -- True involute diameter
    r_TIF = d_TIF/2; -- true involute radius

    alpha_TIF = math.acos((d_p*math.cos(alpha_t_rad))/d_TIF); --pressure angle at True involute diameter

    inv_alpha_TIF = math.tan(alpha_TIF) - alpha_TIF; -- Involute function at True involute diameter

    s_ty = d_TIF * ((s_0/d_p)+inv_alpha_t - inv_alpha_TIF); --tooth thickness at TIF circle (considering profile shift = 0 )


    

    invo_angle = (s_0/r_p) + 2*inv_alpha_t -- To draw the involute between two circles w.r.t angle

    start_angle = involute_angle(r_b,r_b) -- Eventually = 0
    stop_angle = involute_angle(r_b,r_a)
    -- To start and stop involute between r_a and r_b (tip to base radius)
    
    --Fillet formed at the root area of gear with slope of the line and with f_r(fillet radius) and r_b.(end point from the tooth profile above the base circle has the same common tangent with the start point from the tooth profile below the base circle)
    --next step is to create parts under the base circle by finding the boundary of the same common tangent on the profile curve, that is, the position where the base circle curve intersects the profile curve.

    n_points = 30; --for loop iteration
    local points = {} 
    -- finding point of involute to find fillet
    for i = 1,n_points do
        points[i] = tooth_involute(r_b,(start_angle + (stop_angle -start_angle) * i / n_points))
    end

    m_s = slope(points) 
    --With the value of slope, the next step is to find the slope angle.
    slope_angle = math.atan(m_s)

    local parellel_line = {}
    parellel_line[1] = v(points[1].x + f_r * math.cos(slope_angle + math.pi / 2),
                        points[1].y + f_r * math.sin(slope_angle + math.pi / 2)) 
    --A parallel line to the slope of line is formed so as to find the point to form the circle

    -- distence from parellel line to fillet radius centre
    d = (parellel_line[1].y - m_s * parellel_line[1].x) / math.sqrt(m_s*m_s + 1)
    th1 = math.asin(d/(f_r + r_b)) + slope_angle
    
    fillet_center = v(0,0) --initialization of fillet centre variable
    fillet_center = v((f_r + r_f) * math.cos(th1), (f_r + r_f) * math.sin(th1))
    filler_start_angle = 2 * math.pi + math.atan(fillet_center.y / fillet_center.x)
    fillet_stop_angle = 3 * math.pi / 2 + slope_angle
    

    --nested for loop for creating full gear profile including fillet
    for i=1, z do
        for j=1,n_points do -- for fillet
            xy_points[#xy_points+1] = rotate_points(2*math.pi*i/z,circle(fillet_center,f_r,(filler_start_angle +(fillet_stop_angle-filler_start_angle) * j / n_points)))
        end

        -- To start involute from form radius (r_TIF) and end at tip radius (r_a)
        start_angle = involute_angle(r_b,r_TIF)
        stop_angle = involute_angle(r_b,r_a)

        for j=1,n_points do -- for one side of the involute
            xy_points[#xy_points+1] = rotate_points(2*math.pi*i/z,tooth_involute(r_b, (start_angle +(stop_angle-start_angle) *j / n_points)))
        end

        -- now for the other side just mirroring involute and fillet
        for j=n_points,1,-1 do 
            xy_points[#xy_points+1] = rotate_points(2*math.pi*i/z,rotate_points(invo_angle,mirror_(tooth_involute(r_b,(start_angle +(stop_angle-start_angle) *j / n_points)))))    
        end
        for j=n_points,1,-1 do
            xy_points[#xy_points+1] = rotate_points(2*math.pi*i/z,rotate_points(invo_angle,mirror_(circle(fillet_center,f_r,(filler_start_angle +(fillet_stop_angle-filler_start_angle) * j / n_points)))))
        end
    end
    -- Adding first point to table for creating closed loop
    xy_points[#xy_points+1] = xy_points[1]
    return xy_points
end

--function to find center distance between two gears
function center_distance(z1,z2,m_t)
    return (m_t*(z1 + z2)/2) + j_n
end

--function to calculate angle of rotation for the second planetary set
function otherside_rotation_angle(z_s,z_p,m_t)
    --print("hallo")
    angle = 2*asin(center_distance(z_p,z_p,m_t)/(2*center_distance(z_s,z_p,m_t)))
    return angle
end

-- Function for properlly extruding gear
function extrude_gear(z,m_t,alpha_t,h_a_coef,h_f_coef,f_r,b,bore_diameter,is_sun,is_left_side,_shaft_length)
    gear_points = gear(z,m_t,alpha_t,h_a_coef,h_f_coef,f_r)
    gear_extrude = linear_extrude(v(0,0,b),gear_points)
    roof_cylinder = cylinder(r_f,b)
    gear_extruded = rotate(-90,Z)*rotate(90/z+(inv_alpha_t*180/math.pi),Z)*union(gear_extrude , roof_cylinder)
    
    if (is_sun) then
        local shaft = cylinder(bore_diameter/2,_shaft_length)
        local shaft = difference(shaft,translate(0,0,_shaft_length-10)*cylinder(screw_diameter/2,20))
        if (is_left_side) then
            shaft = rotate(180,X)*shaft
        end
        final_gear_extruded = union(gear_extruded,shaft)
        --print("if")
    else
        bore = cylinder(bore_diameter/2,b)
        final_gear_extruded = difference(gear_extruded,bore)
        --print('else')
    end
    return final_gear_extruded
end

-- function for generating and extruding carrier gears
function extrude_carrier(z_sun,z_planet,m_t,alpha_t,h_a_coef,h_f_coef,f_r,b,bore_diameter,is_left_carrier)
    local clearance = 5;
    min_diameter_of_carrier = m_t * (z_sun + 2*z_planet + 6* h_a_coef) + clearance;

    z_carrier = math.floor(min_diameter_of_carrier/m_t)
    
    carrier = extrude_gear(z_carrier,m_t,alpha_t,h_a_coef,h_f_coef,f_r,b,bore_diameter)
    shaft_hole_position = center_distance(z_sun,z_planet,m_t)
    shaft_hole_half = translate(shaft_hole_position,0,0)*cylinder(screw_head_diameter/2,b/2)
    shaft_hole = translate(shaft_hole_position,0,0)*cylinder((screw_diameter)/2,b)
    if (is_left_carrier) then
        for i=1,3 do
            carrier = difference(difference(carrier,rotate(120*(i-1),Z)*shaft_hole),translate(0,0,b/2)*rotate(120*(i-1),Z)*shaft_hole_half);
            carrier = difference(difference(carrier,rotate((otherside_rotation_angle(z_sun,z_planet,m_t)),Z)*rotate(120*(i-1),Z)*shaft_hole),translate(0,0,b/2)*rotate((otherside_rotation_angle(z_sun,z_planet,m_t)),Z)*rotate(120*(i-1),Z)*shaft_hole_half);
        end
        carrier = translate(0,0,spacing_bt_sun+b_carrier+b_sun)*carrier
        name_initials_extruded = translate(0,-(spacing_bt_sun+2*b_carrier+b_sun),0)*translate(-18,0,z_carrier*m_t/4)*rotate(180,Z)*rotate(90,0,180)*name_initials
        --emit(name_initials_extruded)
    else
        for i=1,3 do
            carrier = difference(difference(carrier,rotate(120*(i-1),Z)*shaft_hole),rotate(120*(i-1),Z)*shaft_hole_half);
            carrier = difference(difference(carrier,rotate((otherside_rotation_angle(z_sun,z_planet,m_t)),Z)*rotate(120*(i-1),Z)*shaft_hole),rotate((otherside_rotation_angle(z_sun,z_planet,m_t)),Z)*rotate(120*(i-1),Z)*shaft_hole_half);
        end
        name_initials_extruded = translate(22,0,z_carrier*m_t/4)*rotate(90,0,180)*name_initials
    end
    carrier_with_holes = rotate(90,X)*carrier

    
    carrier_with_names = difference(carrier_with_holes,name_initials_extruded)
    return carrier_with_names
end


--function for generating and returning rotated planet gears (three)
function planet_gears_oneside(planet)
    local p_gear1 = rotate(0,Z)*translate(center_distance(z_sun,z_planet,m_t),0,0)*rotate(180,Z)*planet
    local p_gear2 = rotate(120,Z)*translate(center_distance(z_sun,z_planet,m_t),0,0)*rotate(180,Z)*planet
    local p_gear3 = rotate(240,Z)*translate(center_distance(z_sun,z_planet,m_t),0,0)*rotate(180,Z)*planet
    return p_gear1,p_gear2,p_gear3
end

---function for creating and extruding shaft for planet gear
function extrude_planet_shaft(spacing_bt_sun,b_sun,b_planet,tolerance,bore_diameter_planet,screw_diameter)
    shaft_length = spacing_bt_sun + b_sun + tolerance;
    local spacer_length = b_planet*0.10 + b_sun;
    local shaft_radius = (bore_diameter_planet-bore_diameter_planet*0.02)/2;
    local spacer = cylinder(shaft_radius+1,spacer_length)
    local hole = cylinder(screw_diameter/2,shaft_length)
    local shaft = difference(union(cylinder(shaft_radius,shaft_length),spacer),hole)    
    return shaft
end

--function for calculating angular backlash for given gear in degrees
function angular_backlash(z)
    return 360/math.pi * j_n/(m_t*z)*cos(alpha_t)
end


--------setting up brushes for coloring parts-------
set_brush_color(100,1,0,0)
set_brush_color(101,1,1,0)
set_brush_color(102,1,0,145/255)
set_brush_color(103,0,1,0)

--------------Tweeks-------------


m_t = ui_numberBox("Module",1)
alpha_t = ui_numberBox("Pressure Angle(deg)",20)
z_sun = ui_numberBox("Number of teeth for Sun gear",30)
z_planet = ui_numberBox("Number of teeth for Planet gear",12)
--z_input = ui_numberBox("Number of teeth for input gear",10)
h_a_coef = ui_scalar("Addedum Coefficient",1,1,2)
h_f_coef = ui_scalar("Dedendum coefficient",1.25,1,2) 
f_r = ui_scalar("fillet radius(mm)",0.2,0,m_t/2)
b_sun = ui_numberBox("Width of the sun gear(mm)",5)
b_planet_min = 2*b_sun;
b_planet = ui_number("Width of the Planet gear(mm)",b_planet_min,b_planet_min,4*b_sun)
j_n = ui_scalar("Backlash(mm)",0.2,0.1,1)
--screw_diameter = ui_numberBox("Diameter of the screw for assemblly(mm)",4)
--screw_head_diameter = ui_numberBox("Diameter of the screw head(mm)",6)
--i = ui_numberBox("animation angle",0)

---- Adding Names of team members ----

f = font(Path .. "LiberationMono-Bold.ttf")

name_initials = f:str('J.A.O.R', 10)
name_initials=scale(0.5,0.5,-0.5)*name_initials
--name initials are added in carrier function
--emit(translate(0,50,0)*scale(1,1,10) * name_initials)

---Some important perameters----
screw_diameter = 4;
screw_head_diameter=6;
shaft_diameter_sun = z_sun*m_t*0.25;
if (z_planet*m_t*0.20 > screw_diameter) then 
    bore_diameter_planet = z_planet*m_t*0.20;
else
    bore_diameter_planet = screw_diameter+2;
end
bore_diameter_carrier = shaft_diameter_sun*1.03;
b_carrier = 4;
spacing_bt_sun = b_planet*1.10; -- spacing between sun gears
sun_shaft_length = 20;
gound_to_carrier_distance = 5;



--------------  one side -----------------
is_sun = true;
is_left_side = true;
sun = extrude_gear(z_sun,m_t,alpha_t,h_a_coef,h_f_coef,f_r,b_sun,shaft_diameter_sun,is_sun,is_left_side,sun_shaft_length)
sun_gear = rotate(angular_backlash(z_sun),Z)*translate(0,0,b_carrier)*rotate(180/z_sun,Z)*sun

--emit(rotate(90,X)*sun_gear,100)
planet = extrude_gear(z_planet,m_t,alpha_t,h_a_coef,h_f_coef,f_r,b_planet,bore_diameter_planet)
planet_gear1,planet_gear2,planet_gear3 = planet_gears_oneside(rotate(-angular_backlash(z_planet),Z)*translate(0,0,b_carrier)*planet)
--emit(rotate(90,X)*planet_gear1,101)
--emit(rotate(90,X)*planet_gear2,101)
--emit(rotate(90,X)*planet_gear3,101)


-------------- other side ---------------
is_sun = true;
is_left_side = false;
sun = extrude_gear(z_sun,m_t,alpha_t,h_a_coef,h_f_coef,f_r,b_sun,shaft_diameter_sun,is_sun,is_left_side,sun_shaft_length)
sun_gear = rotate(-angular_backlash(z_sun),Z)*translate(0,0,b_carrier)*rotate(180/z_sun,Z)*sun

--emit(rotate(90,X)*rotate(otherside_rotation_angle(z_sun,z_planet,m_t),Z)*rotate((-z_planet/z_sun)*(180/z_planet),Z)*translate(0,0,spacing_bt_sun)*sun_gear,100)
planet_gear = rotate(180/z_planet,Z)*planet
planet_gear1,planet_gear2,planet_gear3 = planet_gears_oneside(rotate(angular_backlash(z_planet),Z)*translate(0,0,b_carrier)*planet_gear)
--emit(rotate(90,X)*rotate(otherside_rotation_angle(z_sun,z_planet,m_t),Z)*translate(0,0,(spacing_bt_sun+b_sun)-b_planet)*planet_gear1,101)
--emit(rotate(90,X)*rotate(otherside_rotation_angle(z_sun,z_planet,m_t),Z)*translate(0,0,(spacing_bt_sun+b_sun)-b_planet)*planet_gear2,101)
--emit(rotate(90,X)*rotate(otherside_rotation_angle(z_sun,z_planet,m_t),Z)*translate(0,0,(spacing_bt_sun+b_sun)-b_planet)*planet_gear3,101)

--------------carrier----------------
--adding name initials 
is_left_carrier = false;
carrier = extrude_carrier(z_sun,z_planet,m_t,alpha_t,h_a_coef,h_f_coef,f_r,b_carrier,bore_diameter_carrier,is_left_carrier)

--emit(carrier,102)
space_bt_carrier = spacing_bt_sun+2*b_carrier+b_sun;
is_left_carrier = true;
carrier = extrude_carrier(z_sun,z_planet,m_t,alpha_t,h_a_coef,h_f_coef,f_r,b_carrier,bore_diameter_carrier,is_left_carrier)
--emit(rotate(90,X)*translate(0,0,spacing_bt_sun+2*b_carrier+b_sun)*rotate(180,X)*rotate(-otherside_rotation_angle(z_sun,z_planet,m_t),Z)*carrier,102)
--emit(carrier,102)


------ shaft for planet gears---
tolerance = 1;

shaft = extrude_planet_shaft(spacing_bt_sun,b_sun,b_planet,tolerance,bore_diameter_planet,screw_diameter)
shaft_oneside = rotate(otherside_rotation_angle(z_sun,z_planet,m_t),Z)*translate(center_distance(z_sun,z_planet,m_t),0,b_carrier)*shaft;
shaft_otherside = translate(0,0,shaft_length+b_carrier)*rotate(180,X)*translate(center_distance(z_sun,z_planet,m_t),0,0)*shaft;
all_planet_shafts = union(shaft_oneside,shaft_otherside);
for i=1,2 do
    shaft_oneside = rotate(120*i,Z)*rotate(otherside_rotation_angle(z_sun,z_planet,m_t),Z)*translate(center_distance(z_sun,z_planet,m_t),0,b_carrier)*shaft;
    shaft_otherside = rotate(120*i,Z)*translate(0,0,shaft_length+b_carrier)*rotate(180,X)*translate(center_distance(z_sun,z_planet,m_t),0,0)*shaft;
    all_planet_shafts = union(all_planet_shafts,union(shaft_oneside,shaft_otherside))
end 



--emit(rotate(90,X)*all_planet_shafts,103)


-----stand design-----

stand_width = 5;
stand_height = (z_carrier*m_t/2)+bore_diameter_carrier+gound_to_carrier_distance;
stand_length = 3*bore_diameter_carrier;


function create_stand()
    local stand = translate(0,b_carrier+10,-stand_height+bore_diameter_carrier)*cube(stand_length,stand_width,stand_height)
    local hole = translate(0,b_carrier,0)*rotate(-90,X)*cylinder(bore_diameter_carrier/2,stand_height)
    local base = translate(0,b_carrier+10,-stand_height+bore_diameter_carrier+stand_width/4)*rotate(-90,X)*cube(stand_length,stand_width/2,2*stand_width)
    local base_hole1 = translate(stand_length/4,stand_width*1.25,-stand_width)*translate(0,b_carrier+10,-stand_height+bore_diameter_carrier+stand_width/4)*rotate(0,X)*cylinder(screw_diameter/2,2*stand_width)
    local base_hole2 = translate(-stand_length/4,stand_width*1.25,-stand_width)*translate(0,b_carrier+10,-stand_height+bore_diameter_carrier+stand_width/4)*rotate(0,X)*cylinder(screw_diameter/2,2*stand_width)
    local base_holes = union(base_hole1,base_hole2)
    local base = difference(base,base_holes)
    return union(difference(stand,hole),base)
end

stand = translate(0,-8,0)*create_stand()
--emit(stand,1)
mirror_stand = create_stand()
--emit(translate(0,-spacing_bt_sun-b_carrier,0)*rotate(180,Z)*mirror_stand,1)



------ wheels -------
function create_wheel()
    local wheel = rotate(90,X)*cylinder(1.25*bore_diameter_carrier,5)
    local hole = rotate(90,X)*cylinder(screw_diameter/2,5)
    local wheel = difference(wheel,hole)
    local n = 10
    local design = rotate(0,Y)*translate(1.25*bore_diameter_carrier-screw_diameter,0,0)*scale(0.5,1,0.5)*hole;
    for i=1,n-1 do 
        design = union(design,rotate(360*i/n,Y)*translate(1.25*bore_diameter_carrier-screw_diameter,0,0)*scale(0.5,1,0.5)*hole)        
    end
    return difference(wheel,design)   
end
wheel = translate(0,sun_shaft_length,0)*create_wheel()
wheel2 = translate(0,-spacing_bt_sun-b_carrier-sun_shaft_length-b_sun,0)*rotate(180,X)*create_wheel()
--emit(wheel,5)
--emit(wheel2,5)



---- input gear ---

--function for creating input gear handle and shaft
--function create_shaft_with_handle()
--    local shaft = translate(center_distance(z_input,z_carrier,m_t),0,0)*rotate(90,X)*cylinder(z_input*m_t*0.25/2,b_input+20)
--    local handle1 = translate(center_distance(z_input,z_carrier,m_t)+z_input*m_t/2-5,-b_input-20,0)*rotate(-90,X)*cube(15,3,3)
--    local handle2 = translate(center_distance(z_input,z_carrier,m_t)+z_input*m_t/2,-b_input-22,0)*rotate(90,Z)*rotate(90,X)*cube(10,3,3)
--    local shaft_and_handle = union(union(shaft,handle1),handle2)
--    return shaft_and_handle
--end
--b_input = shaft_length+2*b_carrier;
--input_gear = translate(center_distance(z_input,z_carrier,m_t),0,0)*rotate(180/z_input,Y)*rotate(90,X)*extrude_gear(z_input,m_t,alpha_t,h_a_coef,h_f_coef,f_r,b_input,0)
--shaft_and_hadle = create_shaft_with_handle()
--input_gear_with_handle = union(input_gear,shaft_and_hadle)
--emit(input_gear_with_handle,12)





-----for printing ---------

--sun gear x2
sun = extrude_gear(z_sun,m_t,alpha_t,h_a_coef,h_f_coef,f_r,b_sun,shaft_diameter_sun,is_sun,is_left_side,sun_shaft_length)
--emit(sun)


--planet gear x6
planet = extrude_gear(z_planet,m_t,alpha_t,h_a_coef,h_f_coef,f_r,b_planet,bore_diameter_planet)
--emit(planet)

--planet shaft x6
--emit(shaft)

--wheel x2
wheel = rotate(90,X)*create_wheel()
--emit(wheel)

--stand x2
stand_stl = rotate(90,X)*create_stand()
emit(stand_stl)

--left side carrier
carrier_left = rotate(-90,X)*extrude_carrier(z_sun,z_planet,m_t,alpha_t,h_a_coef,h_f_coef,f_r,b_carrier,bore_diameter_carrier,is_left_carrier)
--emit(carrier_left)

--right side carrier
carrier_right = rotate(-90,X)*extrude_carrier(z_sun,z_planet,m_t,alpha_t,h_a_coef,h_f_coef,f_r,b_carrier,bore_diameter_carrier,is_left_carrier)
--emit(carrier_right)