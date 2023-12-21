#!/usr/bin/env python
import numpy as np

def cut_smokedet_hekatron_tab(center_T,
                              rectangle_dimension,
                              center_to_square_offset,
                              cnc_table_base_frame = "table_cnc",
                              cnc_machine_home_frame = 'cnc_machine_home',
                              cnc_table_base_to_cnc_machine_home = None,
                              sendTf = None,
                              tf2x = None,
                             feed_rate = 200,
                             initial_raise_above = 0.02,
                             initial_move_feed = 1000):
    """ Function to cut the smoke detector Hekatron. It's parametric 
    Args:
    -------------------------------------
    center_T: (4,4) Pose matrix of the center of the smoke detector/chuck in the cnc_table_base_frame.
    
    rectangle_dimension: (a,b) dimension of the battery cover rectangle of the hekatron smoke detector.
    
    center_to_square_offset: (a,b,c) dimension of offset from center_T to center of battery box cover
    (used to construct bboxes).
    
    cnc_table_base_frame: Name of CNC table (usually table_cnc)
    
    cnc_machine_home_frame: Name of CNC machine home(homing) frame
    
    sendTf: sendTf object from disassembly cycle manager
    
    tf2x: tf2x object from disassembly
    
    feed_rate : Feed rate for the cutting moves
    
    initial_raise_above: First point will be raised above the cutting points by this distance.
    
    initial_move_feed: Feed rate for the first move which moves above the first cutting point.
    
    
    
    """    
    assert center_T.shape == (4,4), "center_T shape must be (4,4)."
    assert len(rectangle_dimension) == 2, "rectangle_dimension must be of shape (2)."
    assert feed_rate >0, "Feed rate must be positive."
    assert feed_rate < 2000, "For cutting, feed rate should be less than 500."
    
    # If TF is not give, we must look it up
    if cnc_table_base_to_cnc_machine_home is None:
        cnc_table_base_to_cnc_machine_home = tf2x(parent_frame = cnc_table_base_frame,
                                                  child_frame = cnc_machine_home_frame)
        cnc_table_base_to_cnc_machine_home = x2t(cnc_table_base_to_cnc_machine_home)
    
    # Construct relative TF from smokedetector center to battery box center
    bat_bbox_center_dT = np.eye(4)
    bat_bbox_center_dT[0:3, -1] = center_to_square_offset
    # Construct absolute TF of battery box center
    bat_bbox_center_T = center_T@bat_bbox_center_dT
    
    # Relative TF from battery box center to bbox edges
    bat_bbox_a_dT = np.eye(4)
    bat_bbox_b_dT = np.eye(4)
    bat_bbox_c_dT = np.eye(4)
    bat_bbox_d_dT = np.eye(4)
    
    bat_bbox_a_name = 'hek_batbox_edge_a'
    bat_bbox_b_name = 'hek_batbox_edge_b'
    bat_bbox_c_name = 'hek_batbox_edge_c'
    bat_bbox_d_name = 'hek_batbox_edge_d'


    bat_bbox_a_dT[0:3, -1] = -rectangle_dimension[0], +rectangle_dimension[1], 0
    bat_bbox_b_dT[0:3, -1] = -rectangle_dimension[0], -rectangle_dimension[1], 0
    bat_bbox_c_dT[0:3, -1] = +rectangle_dimension[0], -rectangle_dimension[1], 0
    bat_bbox_d_dT[0:3, -1] = +rectangle_dimension[0], +rectangle_dimension[1], 0

    bat_bbox_a = bat_bbox_center_T@bat_bbox_a_dT
    bat_bbox_b = bat_bbox_center_T@bat_bbox_b_dT
    bat_bbox_c = bat_bbox_center_T@bat_bbox_c_dT
    bat_bbox_d = bat_bbox_center_T@bat_bbox_d_dT

        
    if sendTf is not None:
        # Draw center TF
        #sendTf(p = center_T[0:3, -1], q = r2q(center_T[0:3, 0:3]), parent_frame = cnc_table_base_frame, \
        #       child_frame = "center_hek")
        
        # Draw battery box center tf
        #sendTf(p = bat_bbox_center_T[0:3, -1], q = r2q(bat_bbox_center_T[0:3, 0:3]), parent_frame = cnc_table_base_frame, \
        #       child_frame = "center_hek_batbox")

        # Draw square bbox TFs
        sendTf(p = bat_bbox_a[0:3, -1], q = r2q(bat_bbox_a[0:3, 0:3]), parent_frame = cnc_table_base_frame, \
               child_frame = bat_bbox_a_name)
        
        sendTf(p = bat_bbox_b[0:3, -1], q = r2q(bat_bbox_b[0:3, 0:3]), parent_frame = cnc_table_base_frame, \
               child_frame = bat_bbox_b_name)
        
        sendTf(p = bat_bbox_c[0:3, -1], q = r2q(bat_bbox_c[0:3, 0:3]), parent_frame = cnc_table_base_frame, \
               child_frame = bat_bbox_c_name)
        
        sendTf(p = bat_bbox_d[0:3, -1], q = r2q(bat_bbox_d[0:3, 0:3]), parent_frame = cnc_table_base_frame, \
               child_frame = bat_bbox_d_name)

    
        #sendTf(p=cnc_table_base_to_cnc_machine_home[0:3, -1], q = r2q(cnc_table_base_to_cnc_machine_home[0:3, 0:3]), \
        #       parent_frame = cnc_table_base_frame, child_frame = cnc_machine_home_frame)
    
    # Get all points in relation to cnc machine zero frame.
    a_in_cnc = np.linalg.solve(cnc_table_base_to_cnc_machine_home, bat_bbox_a)[0:3,-1]
    
    above_a = copy.deepcopy(a_in_cnc)
    above_a[2] +=initial_raise_above
    
    b_in_cnc = np.linalg.solve(cnc_table_base_to_cnc_machine_home, bat_bbox_b)[0:3,-1]
    c_in_cnc = np.linalg.solve(cnc_table_base_to_cnc_machine_home, bat_bbox_c)[0:3,-1]
    d_in_cnc = np.linalg.solve(cnc_table_base_to_cnc_machine_home, bat_bbox_d)[0:3,-1]
        
    #print(a_in_cnc)
    #print(b_in_cnc)
    #print(c_in_cnc)
    #print(d_in_cnc)
    
    out_gcode = []
    all_commands = [above_a, a_in_cnc, b_in_cnc, c_in_cnc, d_in_cnc, a_in_cnc, above_a]
    # Append initial
    i=0
    for element in all_commands:
        feed_rate_tmp = int(feed_rate)
        
        if (i==0) or (i==len(all_commands)-1):
            # For first point above the smokedet, and for last point where we raise above the 
            # smoke detector, increase the feed rate since these are not cutting moves.
            feed_rate_tmp = int(initial_move_feed)
            
        gcode_line = "G01X{}Y{}Z{}F{}\n".format(int(1000*element[0]), int(1000*element[1]), int(1000*element[2]), \
                                                feed_rate_tmp)
        out_gcode.append(gcode_line)
        i+=1
    
    return out_gcode

if __name__ == '__main__':
    cnc_table_base_frame = 'table_cnc'
    cnc_machine_home_frame = 'cnc_machine_home'

    center_to_square_offset = [0.1, 0, 0.1]

    rectangle_dimension = [0.057, 0.035]

    smokedet_center_in_cnc_table_frame = [- 0.13, 0.1,0]
    smokedet_rotation_deg = 90

    center_T = np.eye(4)
    center_T[0:3, -1] = smokedet_center_in_cnc_table_frame
    center_T[0:3, 0:3] = center_T[0:3, 0:3]@rot_z(smokedet_rotation_deg, unit='deg')

    out_gcode_list = cut_smokedet_hekatron_tab(center_T = center_T,
                            rectangle_dimension = rectangle_dimension,
                            center_to_square_offset = center_to_square_offset,
                            sendTf = cyc_manager.sendTf,
                            cnc_table_base_frame = "table_cnc",
                            cnc_machine_home_frame = 'cnc_machine_home',
                            cnc_table_base_to_cnc_machine_home = None,
                            tf2x = cyc_manager.tf2x, feed_rate = 200)
