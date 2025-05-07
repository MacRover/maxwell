# Initializing Params

import time

def velocity_function(current_pos, steps_to_move, cw_enable, ccw_enable, acceleration, v_i, v_max, start_time):

    set_point = current_pos + steps_to_move

    # dont want to move at all, so save the resources

    if (set_point == current_pos):
        print("\n\t\tset point == curernt")
        return 0
        # dont want to move at all
    else:

        # calculation of the projected time

        projected_time = time_calc(v_i, v_max, acceleration, steps_to_move)

        print("\n\t================================= VELOCITY PRINTING ====================================")

        # taking of the current time

        current_time = time.time()

        time_elapsed = current_time - start_time
        print("time elapsed", time_elapsed)

        # if we are going clockwise

        if (cw_enable):
            print("cw enable selected")

            # checks if we are below the halfway point, if so, make acceleration positive
            if (time_elapsed <= projected_time/2):
                acceleration = abs(acceleration)

            # checks if we are beyond the halfway point, if so, make acceleration negative
            else:
                acceleration = -abs(acceleration)

            # this makes sense because positive acceleration leads to faster motion.  We need to slow down motion at halfway

        # if counterclockwise
        
        if (ccw_enable):
            print("ccw enable selected")

            # If less than half of the time has passed and we are going CCW, NOW we use negative!
            # the reason for this is because we are moving in the opposite direction
            # if not we use positive
            if (time_elapsed <= projected_time/2):
                acceleration = -abs(acceleration)
            else:
                acceleration = abs(acceleration)

        print("accel", acceleration)

        # calculation of the current velocity

        print("time elapsed ", time_elapsed) # prints out the current time elapsed
        print("proj time", projected_time) # reprints the current projected time
        print("left", projected_time - time_elapsed) # prints out the current left time

        # if the elapsed time is greater than the projected time divided by 2

        if (time_elapsed >= (projected_time / 2)):

            # ACCELERATION MUST DECREASE HERE!

            if (cw_enable):
                # Here, acceleration is positive
                v_peak = v_i + abs(acceleration*(projected_time/2)) # this should still be positive because it needs to decrease accordingly

            elif (ccw_enable):
                # here, acceleration will be negative
                v_peak = v_i + -abs(acceleration*(projected_time/2)) # this should still be positive because it needs to decrease accordingly
            
            v_fixed = acceleration*(time_elapsed-(projected_time/2))
            v_f = v_peak + v_fixed # need to decrease acceleration
            
            # vi becomes v_peak
            print("vf vpeak")
            print(v_f)
            print(v_fixed)
            print(v_peak)


        else:
            v_f = v_i + acceleration*(time_elapsed)

        # returning the velocity

        # final velocity shown is less than v_max, return that velocity
        # final velocity shown is greater than v_max, return v_max

        print("-v_max", -v_max)
        print("v_max", v_max)

        # logic issue here that needs to be fixed

        # something going very wrong with the program that needs to be fixed in here -----

        # if clockwise is enabled
        
        if (cw_enable):

            # absolute value of v_f is greater than v_max
            
            if (abs(v_f) > v_max):
                print("CASE 1")
                print("v-final: MEASUREMENT TO RETURN:", v_max)
                print(v_f)
                return v_max
            
            elif (v_f < v_max):
                print("CASE 2")
                print("v-final: MEASUREMENT TO RETURN:", v_f)
                print("hi?")
                return v_f
            
        
        if (ccw_enable):
            if (v_f > -v_max):
                print("CASE 3")
                print("v-final: MEASUREMENT TO RETURN:", v_f)
                #print("hi?")
                return v_f
            
            elif (abs(v_f) > v_max):
                print("CASE 4")
                print("v-final: MEASUREMENT TO RETURN:", -v_max)
                return -v_max



def time_calc(v_i, v_max, acceleration, steps_to_move): # verified with 1 test case
    # calculating the maximum time of the movement


    # Time to reach the maximum point

    print("\n\t ===================== TIME PRINTING ==========================")

    t_max = abs((v_max - v_i) / acceleration) # 5
    print("tmax", t_max)

    # Time to reach the minimum point (vf = 0, vi = v_max)

    t_min = abs(-v_max / acceleration) # 5
    print("tmin", t_min)


    # finding the steps of increase and decrease

    steps_increase = v_max*t_max - 0.5*acceleration*(t_max**2)
    print("steps inc", steps_increase)

    steps_decrease = v_max*t_min + 0.5*-acceleration*(t_min**2)
    print("steps dec", steps_increase)

    # finding the total number of standard steps

    standard_steps = steps_to_move - steps_increase - steps_decrease
    print("standard steps", standard_steps)

    # Finding the time at the level value

    t_level = standard_steps / v_max
    print("t_level", t_level)

    # add all the time values to get the total projected time

    t_total = t_max + t_min + t_level
    print("total time", t_total)

    return t_total


current_pos = 0
steps_to_move = 90
cw_enable = 0
ccw_enable = 1
acceleration = 2 # arbitrary, to be changed later
time_wait = 14 #change this to test different times

# issue: boundary change at the halfway

# this is assuming a current velocity can be passed in, which, if we are switching to velocity-based PID, could be feesible
v_i = 0 # this will be the initial velocity
v_max = 10 #arbitarry maximum velocity value

start_time = time.time()


#def velocity_function(current_pos, steps_to_move, cw_enable, ccw_enable, acceleration, v_i, v_max, start_time):

print("hi?")

t = time_calc(v_i, v_max, acceleration, steps_to_move)

time.sleep(time_wait)


v = velocity_function(current_pos, steps_to_move, cw_enable, ccw_enable, acceleration, v_i, v_max, start_time)

# current areas to look into

# how it will work if we are moving one direction, then want to start moving in the other drection.  Need to stop first

# this concern might be irrelevant










