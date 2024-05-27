# Author: Nithin Vasishta
# Date Created: 3rd September 2023
# Last Edited: 3rd September 2023

def set_args(original_args, new_args):

    vx = original_args.vx_desired
    vy = original_args.vy_desired
    dyw = original_args.yaw_rate_desired
    
    raise("Controller not tuned for this setting")

    if vx != 0 and vy == 0 and dyw == 0:
        # Move in x direction
        new_args.lost_contact_only_z = True

    else:
        raise("Controller not tuned for this setting")