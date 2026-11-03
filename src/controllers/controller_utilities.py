#!/usr/bin/env python3


class WorkspaceMapper:
    def __init__(self):
        '''
        Helper class to map the robot workspace and prevent the robot from entering
        invalid states (motor limits or dead zones).        
        ### ML ###
        
        we define different states for the ML plane of the robot:
        
        state -1000 ----> corresponds to the initial state where ML tendon is assumed in the Lateral limit 
                     of the workspace and we search for the first valid Medial actuation
        
        state 0 ----> a dead zone or a limit is found inside the robot workspace and no actuation is valid there
        
        state 1 ----> all good we check for the limits
        
        ### AP ###
        
        Since the ML has two tendons we map differently the states: in fact, the deadzone represents a point where nor the Antero nor the Posterior 
        tendon is pulled and both are released. this occurs in the middle of the robot's workspace so the states are defined differently  
        
        state 0 ----> corresponds to the inital states or deadzone state where both A and P tendons are released we want to actuate in Posterior 
                      to go to a valid Posterior state in a non deadzone space 
                 
        state 1 ----> P actuation space, all good, we check for the limits or the deadzone
        
        state 2 ----> P actuation space UPPER limit: no further P actuation is possible. Only A is allowed
        
        state -1 ----> A actuation space, all good, we check for the limits or the deadzone
        
        state -2 ----> A actuation space LOWER limit: no further A actuation is possible. Only P is allowed
        
        state "deadzone" -----> during actuations we arrive at the AP central deadzone. We would like to actuate to go back to a valid point
        
        '''
        
        
        self.ml_state = 1
        self.ap_state = 1
        
        self.Max_INS = 16000
        self.Min_INS = 0
        
        self.max_M = 0
        self.max_L = 830 # 830 is the maximum to go from lateral deadzone to the medial maximum of the motors
        
        
        self.starting_AP = 900
        self.starting_ML = 1000
        self.max_P = 0 
        self.start_A_dead = 1300 
        self.start_P_dead = 800
        self.max_A = 2200
        
        self.actual_ML_actuation = 830
        self.actual_AP_actuation = 800
        self.actual_insertion = 7000

        
        
        
        
    
    def check_for_ap_limits(self, AP_actuation):
        new_val = self.actual_AP_actuation + AP_actuation
        return 1 if new_val <= self.max_A or new_val >= self.max_P else 0

    def check_for_ml_limits(self, ML_actuation):
        new_val = self.actual_ML_actuation + ML_actuation
        return 0 if self.max_L <= new_val <= self.max_M else 1

    # ---------------------------
    # STATE UPDATING
    # ---------------------------
    def update_ap_state(self):
        """Automatically update AP state based on actuator position."""
        if self.actual_AP_actuation <= self.max_P:
            self.ap_state = 2  # Posterior upper limit
        elif self.actual_AP_actuation >= self.max_A:
            self.ap_state = -2  # Anterior lower limit
        elif self.start_P_dead <= self.actual_AP_actuation <= self.start_A_dead :  
            self.ap_state = "deadzone"  # Dead zone
        elif self.actual_AP_actuation < self.start_P_dead:
            self.ap_state = 1  # Posterior space
        else:
            self.ap_state = -1  # Anterior space

    def update_ml_state(self):
        """Automatically update ML state based on actuator position."""
        if self.actual_ML_actuation >= self.max_L or self.actual_ML_actuation <= self.max_M:
            self.ml_state = 0  # medial limit reached (dead zone)
        else:
            self.ml_state = 1  # valid movement area

    # ---------------------------
    # ACTUATION FUNCTIONS
    # ---------------------------
    def add_step_AP(self, AP_actuation):
        """Apply AP actuation step and update AP state."""
        if self.check_for_ap_limits(AP_actuation):
            self.actual_AP_actuation += AP_actuation
            self.update_ap_state()
            return 1
        else:
            self.ap_state = 2 if AP_actuation > 0 else -2
            return 0

    def add_step_ML(self, ML_actuation):
        """Apply ML actuation step and update ML state."""
        if self.check_for_ml_limits(ML_actuation):
            self.actual_ML_actuation += ML_actuation
            self.update_ml_state()
            return 1
        else:
            self.ml_state = 0  # dead zone or out-of-bounds
            return 0
        
    def add_insertion(self, Ins_actuation):
        self.actual_insertion += Ins_actuation
        
        
    def get_catheter_state(self):
        print("Insertion actauted: ", self.actual_insertion)
        print("AP actuated: ", self.actual_AP_actuation)
        print("ML actuated: ", self.actual_ML_actuation)
        
        print("                 ")
        print("AP state: ", self.ap_state)
        print("ML state: ", self.ml_state)
        
        
        
    def prune_actuations(self, actuation_vector):
        """
        Takes an actuation vector [ML, AP, Insertion] and prunes it to ensure
        only valid movements are executed based on the current workspace state.

        Returns:
            pruned_vector (list): A safe actuation vector.
        """
        INS_act, ML_act, AP_act = actuation_vector

        # --- ML pruning --- #
        if self.ml_state == 0 and self.actual_ML_actuation <= self.max_M:
            # M limit, no negative motion allowed
            if ML_act < 0: 
                ML_act = 0
                
        elif self.ml_state == 0 and self.actual_ML_actuation >= self.max_L:
            # L limit, no positive motion allowed
            if ML_act > 0:
                ML_act = 0
                
        elif self.ml_state == 1:
            if ML_act < 0 and self.actual_ML_actuation + ML_act < self.max_M:
                ML_act = self.max_M - self.actual_ML_actuation
                
            if ML_act > 0 and self.actual_ML_actuation + ML_act > self.max_L:
                ML_act = self.max_L - self.actual_ML_actuation
        
        
            
            

        
        # --- AP pruning ---
        
        if self.ap_state == 2:
            # posterior upper limit: no more posterior motion allowed
            if AP_act < 0:
                AP_act = 0
                
        #elif self.ap_state == 1:
            # anterior WS
            #if AP_act < 0 and self.actual_AP_actuation + AP_act < self.max_P:
                    #AP_act = self.max_P - self.actual_AP_actuation        
            #if AP_act > 0 and self.actual_AP_actuation + AP_act > self.start_P_dead:
                    #AP_act = self.start_A_dead - self.actual_AP_actuation 
        
        elif self.ap_state == -2:
            # anterior lower limit: no more anterior motion allowed
            if AP_act > 0:
                AP_act = 0
        #elif self.ap_state == -1:
            # anterior WS
            #if AP_act > 0 and self.actual_AP_actuation + AP_act > self.max_A:
                    #AP_act = self.max_A - self.actual_AP_actuation
                    
            #if AP_act < 0 and self.actual_AP_actuation + AP_act < self.start_A_dead:
                    #AP_act = self.start_P_dead - self.actual_AP_actuation
            
        # INS pruning
        
        if self.actual_insertion + INS_act < self.Min_INS:
            INS_act = self.Min_INS - self.actual_insertion
            
        if self.actual_insertion + INS_act > self.Max_INS:
            INS_act = self.Max_INS - self.actual_insertion
                
            
            
            
        # ap_state == 1 or -1 → both directions OK until limit check

        # insertion usually always allowed unless physical constraints exist
        # (You can add insertion limits here later if needed)

        return [INS_act, ML_act, AP_act]
    
    
    
    
    
    def prune_actuations_no_dead(self, actuation_vector):
        """
        Takes an actuation vector [ML, AP, Insertion] and prunes it to ensure
        only valid movements are executed based on the current workspace state.

        Returns:
            pruned_vector (list): A safe actuation vector.
        """
        INS_act, ML_act, AP_act = actuation_vector

        # --- ML pruning --- #
        if self.ml_state == 0 and self.actual_ML_actuation <= self.max_M:
            # M limit, no negative motion allowed
            if ML_act < 0: 
                ML_act = 0
                
        elif self.ml_state == 0 and self.actual_ML_actuation >= self.max_L:
            # L limit, no positive motion allowed
            if ML_act > 0:
                ML_act = 0
                
        elif self.ml_state == 1:
            if ML_act < 0 and self.actual_ML_actuation + ML_act < self.max_M:
                ML_act = self.max_M - self.actual_ML_actuation
                
            if ML_act > 0 and self.actual_ML_actuation + ML_act > self.max_L:
                ML_act = self.max_L - self.actual_ML_actuation
        
        
            
            

        
        # --- AP pruning ---
        
        if self.ap_state == 2:
            # posterior upper limit: no more posterior motion allowed
            if AP_act < 0:
                AP_act = 0
                
        elif self.ap_state == 1:
            # anterior WS
            if AP_act < 0 and self.actual_AP_actuation + AP_act < self.max_P:
                    AP_act = self.max_P - self.actual_AP_actuation        
            if AP_act > 0 and self.actual_AP_actuation + AP_act > self.start_P_dead:
                    AP_act = self.start_A_dead - self.actual_AP_actuation 
                    return 1
        
        elif self.ap_state == -2:
            # anterior lower limit: no more anterior motion allowed
            if AP_act > 0:
                AP_act = 0
        elif self.ap_state == -1:
            # anterior WS
            if AP_act > 0 and self.actual_AP_actuation + AP_act > self.max_A:
                    AP_act = self.max_A - self.actual_AP_actuation
                    
            if AP_act < 0 and self.actual_AP_actuation + AP_act < self.start_A_dead:
                    AP_act = self.start_P_dead - self.actual_AP_actuation
                    return -1
            
        # INS pruning
        
        if self.actual_insertion + INS_act < self.Min_INS:
            INS_act = self.Min_INS - self.actual_insertion
            
        if self.actual_insertion + INS_act > self.Max_INS:
            INS_act = self.Max_INS - self.actual_insertion
                
            
            
            
        # ap_state == 1 or -1 → both directions OK until limit check

        # insertion usually always allowed unless physical constraints exist
        # (You can add insertion limits here later if needed)

        return [INS_act, ML_act, AP_act]

