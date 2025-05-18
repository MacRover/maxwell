import time

# return an instantaneous velocity - currently anticipating to do that

def time_calc(v_i, v_max, acceleration, steps_to_move):
    # steps_to_move is the number of steps in the full motion

    # time based calculations

    t_max = abs((v_max - v_i) / acceleration) # 5
    t_min = abs(-v_max / acceleration) # 5


    # finding the steps of increase and decrease (needed later for the total time)

    steps_increase = v_max*t_max - 0.5*acceleration*(t_max**2)
    steps_decrease = v_max*t_min + 0.5*-acceleration*(t_min**2)
    standard_steps = steps_to_move - steps_increase - steps_decrease

    # Finding the time at the level value
    t_level = standard_steps / v_max
    t_total = t_max + t_min + t_level

    # return the total time for the motion (need to return velocity at a random instant and comapre against this time?)

    return t_total

def velocity_function(current_pos, steps_to_move, acceleration, v_i, v_max, start_time):

    set_point = current_pos + steps_to_move

    # dont want to move at all, so save the resources

    if (set_point == current_pos):
        return 0

    else:

        # calculation of the projected time

        projected_time = time_calc(v_i, v_max, acceleration, steps_to_move)

        # taking of the current time

        current_time = time.time()

        time_elapsed = current_time - start_time

        # Checking where we are in the motion (based on current time, that will set if acceleration is positive or negative)
        # I think we still will need this

        if (time_elapsed <= projected_time/2):
            acceleration = abs(acceleration)

        else:
            acceleration = -abs(acceleration)

            # this makes sense because positive acceleration leads to faster motion.  We need to slow down motion at halfway

        # if counterclockwise

        # if the elapsed time is greater than the projected time divided by 2

        if (time_elapsed >= (projected_time / 2)):

            # ACCELERATION MUST DECREASE HERE!

            v_peak = v_i + abs(acceleration*(projected_time/2)) # this should still be positive because it needs to decrease accordingly
            
            v_fixed = acceleration*(time_elapsed-(projected_time/2))
            v_f = v_peak + v_fixed # need to decrease acceleration
            

        else:
            v_f = v_i + acceleration*(time_elapsed)

        # returning the velocity
        
        if (abs(v_f) > v_max):
            return v_max
        
        elif (v_f < v_max):
            return v_f