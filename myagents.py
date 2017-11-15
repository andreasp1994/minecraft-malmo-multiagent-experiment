""" 
 Artificial Inteligence (H)
 Assessed Exercise 2017/2018

 Tested with Python 2.7
 
 Solution Template (revision a)
"""
import logging

#----------------------------------------------------------------------------------------------------------------#                  
class SolutionReport(object):     
    # Please do not modify this custom class !
    def __init__(self):
        """ Constructor for the solution info 
        class which is used to store a summary of 
        the mission and agent performance """

        self.start_datetime_wallclock = None;
        self.end_datetime_wallclock = None;          
        self.action_count = 0;     
        self.reward_cumulative = 0;       
        self.mission_xml = None;          
        self.mission_type = None;       
        self.mission_seed = None;       
        self.student_guid = None;       
        self.mission_xml_as_expected = None;       
        self.is_goal = None;
        self.is_timeout = None;

    def start(self):
        self.start_datetime_wallclock = datetime.datetime.now()
    
    def stop(self):
        self.checkGoal()
        self.end_datetime_wallclock = datetime.datetime.now()

    def setMissionXML(self, mission_xml):
        self.mission_xml = mission_xml

    def setMissionType(self, mission_type):
        self.mission_type = mission_type
    
    def setMissionSeed(self, mission_seed):
        self.mission_seed = mission_seed
    
    def setStudentGuid(self, student_guid):
        self.student_guid = student_guid
                  
    def addAction(self):
        self.action_count += 1

    def addReward(self, reward, datetime_):
        self.reward_cumulative += reward        

    def checkMissionXML(self):
          """ This function is part of the final check"""
          #-- It is not included for simplifity --#
          self.mission_xml_as_expected = 'Unknown'
  
    def checkGoal(self):
          """ This function checks if the goal has been reached based on the expected reward structure (will not work if you change the mission xml!)"""
          #-- It is not included for simplifity --#
          if self.reward_cumulative!=None:
            x = round((abs(self.reward_cumulative)-abs(round(self.reward_cumulative)))*100);
            rem_goal = x%7
            rem_timeout = x%20
            if rem_goal==0 and x!=0:
                self.is_goal = True
            else:
                self.is_goal = False    
            
            if rem_timeout==0 and x!=0:
                self.is_timeout = True
            else:
                self.is_timeout = False    

#----------------------------------------------------------------------------------------------------------------#                  
class StateSpace(object):     
    """ This is a datatype used to collect a number of important aspects of the environment
    It can be constructed online or be created offline using the Helper Agent
    
    You are welcome to modify or change it as you see fit
    
    """
    
    def __init__(self):
        """ Constructor for the local state-space representation derived from the Orcale"""
        self.state_locations = None;
        self.state_actions = None;
        self.start_id = None;  # The id assigned to the start state
        self.goal_id = None;  # The id assigned to the goal state
        self.start_loc = None; # The real word coordinates of the start state
        self.goal_loc = None; # The real word coordinates of the goal state        
        self.reward_states_n = None
        self.reward_states = None       
        self.reward_sendcommand = None
        self.reward_timeout = None               
        self.timeout = None


#----------------------------------------------------------------------------------------------------------------#                  
def GetMissionInstance( mission_type, mission_seed, agent_type):
    """ Creates a specific instance of a given mission type """

    #Size of the problem
    msize = {
        'small': 10,
        'medium': 20,
        'large': 40,
    }   

    # Timelimit
    mtimeout = {        
        'small':   60000,
        'medium': 240000,
        'large':  960000,
        'helper': 200000,
    }

    # Number of intermediate rewards
    nir = {   
        'small': 3,
        'medium': 9,
        'large': 27,
    }    
        
    mission_type_tmp = mission_type
    if agent_type.lower()=='helper':
        mission_type_tmp = agent_type.lower()

    #-- Define various parameters used in the generation of the mission --#
    #-- HINT: It is crucial that you understand the details of the mission, this will require some knowledge of uncertainty/probability and random variables --# 
    random.seed(mission_seed)
    reward_goal = abs(round(random.gauss(1000, 400)))+0.0700
    reward_waypoint = round(abs(random.gauss(3, 15)))
    reward_timeout = -round(abs(random.gauss(1000, 400)))-0.2000
    reward_sendcommand = round(-random.randrange(2,10))
   
    n_intermediate_rewards = random.randrange(1,5) * nir.get(mission_type, 10) # How many intermediate rewards...?

    xml_str = '''<?xml version="1.0" encoding="UTF-8" ?>
        <Mission xmlns="http://ProjectMalmo.microsoft.com" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
          <About>
            <Summary>Mission for assessed exercise 2016/2017 University of Glasgow</Summary>
          </About>
          <ServerSection>
            <ServerInitialConditions>
              <Time>
                <StartTime>6000</StartTime>
                <AllowPassageOfTime>false</AllowPassageOfTime>
              </Time>
              <Weather>clear</Weather>
              <AllowSpawning>false</AllowSpawning>
            </ServerInitialConditions>
            <ServerHandlers>
              <FlatWorldGenerator generatorString="3;7,220*1,5*3,2;3;,biome_1" />      
              <MazeDecorator>
                <Seed>'''+str(mission_seed)+'''</Seed>
                <SizeAndPosition length="'''+str(msize.get(mission_type,100))+'''" width="'''+str(msize.get(mission_type,100))+'''" yOrigin="215" zOrigin="0" xOrigin="0" height="180"/>        
                <GapProbability variance="0.3">0.2</GapProbability>        
                <MaterialSeed>1</MaterialSeed>
                <AllowDiagonalMovement>false</AllowDiagonalMovement>
                <StartBlock fixedToEdge="true" type="emerald_block" height="1"/>
                <EndBlock fixedToEdge="false" type="redstone_block" height="12"/>
                <PathBlock type="glowstone" colour="WHITE ORANGE MAGENTA LIGHT_BLUE YELLOW LIME PINK GRAY SILVER CYAN PURPLE BLUE BROWN GREEN RED BLACK" height="1"/>
                <FloorBlock type="air"/>
                <SubgoalBlock type="glowstone"/>        
                <GapBlock type="stained_hardened_clay" colour="WHITE ORANGE MAGENTA LIGHT_BLUE YELLOW LIME PINK GRAY SILVER CYAN PURPLE BLUE BROWN GREEN RED BLACK" height="3"/>
                <Waypoints quantity="'''+str(n_intermediate_rewards)+'''">
                  <WaypointItem type="diamond_block"/>          
                </Waypoints>      
              </MazeDecorator>      
              <ServerQuitFromTimeUp timeLimitMs="'''+str(mtimeout.get(mission_type_tmp,0))+'''" description="out_of_time"/>
              <ServerQuitWhenAnyAgentFinishes />
            </ServerHandlers>
          </ServerSection>
          <AgentSection>
            <Name>My Agent</Name>
            <AgentStart>
              <Placement x="0" y="216" z="90"/> <!-- will be overwritten by MazeDecorator -->
            </AgentStart>
            <AgentHandlers>
              <ObservationFromFullStats/>
              <ObservationFromRecentCommands/>
              <ObservationFromFullInventory/>
              <RewardForCollectingItem>                
                <Item reward="'''+str(reward_waypoint)+'''" type="diamond_block"/>        
              </RewardForCollectingItem>
              <RewardForSendingCommand reward="'''+str(reward_sendcommand)+'''"/>              
              <RewardForMissionEnd rewardForDeath="-1000000">
                <Reward description="found_goal" reward="'''+str(reward_goal)+'''" />                
                <Reward description="out_of_time" reward="'''+str(reward_timeout)+'''" />
              </RewardForMissionEnd>
              <AgentQuitFromTouchingBlockType>
                <Block type="redstone_block" description="found_goal" />
              </AgentQuitFromTouchingBlockType>                           
            </AgentHandlers>    
          </AgentSection>
        </Mission>'''
    return xml_str, msize.get(mission_type,100),reward_goal,reward_waypoint,n_intermediate_rewards,reward_timeout,reward_sendcommand,mtimeout.get(mission_type_tmp,0)    

#----------------------------------------------------------------------------------------------------------------#                  
# This function initialized the mission based on the input arguments
def init_mission(agent_host, port=0, agent_type='Unknown',mission_type='Unknown', mission_seed=0, movement_type='Continuous'):    
    """ Generate, and load the mission and return the agent host """
   
    #-- Set up the mission via XML definition --#         
    mission_xml, msize, reward_goal,reward_intermediate,n_intermediate_rewards,reward_timeout,reward_sendcommand, timeout = GetMissionInstance(mission_type,mission_seed,agent_type)
    my_mission = MalmoPython.MissionSpec(mission_xml, True)
    my_mission.forceWorldReset()
    
    #-- Enforce the specific restriction for the assessed exercise --#
    #-- If you want a super agent, define one for you self  --#
    my_mission.setModeToCreative()    
    if agent_type.lower()=='random':
        n = msize
        my_mission.observeGrid(-n, -1, -n, n, -1, n, 'grid')
        my_mission.requestVideoWithDepth(320,240)                             
    elif agent_type.lower()=='simple':    
        n = msize    
        my_mission.observeGrid(-n, -1, -n, n, -1, n, 'grid');
        my_mission.requestVideo(320,240)        
    elif agent_type.lower()=='realistic':
        n = 1 # n=1 means local info only !
        my_mission.observeGrid(-n,-1,-n, n, -1, n, 'grid');
        my_mission.requestVideoWithDepth(320,240)   
    elif agent_type.lower()=='helper':
        n = 100 
        my_mission.observeGrid(-n,-1,-n, n, -1, n, 'grid');
        my_mission.requestVideoWithDepth(320,240)            
    else:        
        #-- Define a custom agent and add the sensors you need --#
        n = 100    
        my_mission.observeGrid(-n, -1, -n, n, 1, n, 'grid');
        my_mission.requestVideoWithDepth(320,240)    
    
    #-- Add support for the specific movement type requested (and given the constraints of the assignment) --#  
    #-- See e.g. http://microsoft.github.io/malmo/0.17.0/Schemas/MissionHandlers.html   --#
    if movement_type.lower()=='absolute':           
        my_mission.allowAllAbsoluteMovementCommands()     
    elif movement_type.lower()=='continuous':    
        my_mission.allowContinuousMovementCommand('move')
        my_mission.allowContinuousMovementCommand('strafe') 
        my_mission.allowContinuousMovementCommand('pitch') 
        my_mission.allowContinuousMovementCommand('turn') 
        my_mission.allowContinuousMovementCommand('crouch') 
    elif movement_type.lower()=='discrete':         
        my_mission.allowDiscreteMovementCommand('turn')
        my_mission.allowDiscreteMovementCommand('move')
        my_mission.allowDiscreteMovementCommand('movenorth')
        my_mission.allowDiscreteMovementCommand('moveeast')
        my_mission.allowDiscreteMovementCommand('movesouth')
        my_mission.allowDiscreteMovementCommand('movewest')
        my_mission.allowDiscreteMovementCommand('look')
                       
    #-- Get the resulting xml (and return in order to check that conditions match the report) --#
    final_xml = my_mission.getAsXML(True)

    # Set up a recording for later inspection
    my_mission_record = MalmoPython.MissionRecordSpec('tmp' + ".tgz")
    my_mission_record.recordRewards()
    my_mission_record.recordMP4(24,400000)


    #-- Attempt to start a mission --#
    max_retries = 5
    for retry in range(max_retries):
        try:
            agent_host.startMission(my_mission, my_mission_record )
            break
        except RuntimeError as e:
            if retry == max_retries - 1:
                print("Error starting mission:",e)
                exit(1)
            else:
                time.sleep(2)

    #-- Loop until mission starts: --#
    print("Waiting for the mission to start ")
    state_t = agent_host.getWorldState()
    while not state_t.has_mission_begun:
        sys.stdout.write(".")
        time.sleep(0.1)
        state_t = agent_host.getWorldState()
        for error in state_t.errors:
            print("Error:",error.text)

    print
    print( "Mission started (xml returned)... ")
    return final_xml,reward_goal,reward_intermediate,n_intermediate_rewards,reward_timeout,reward_sendcommand,timeout


def show_map(node_colors, G, locations, node_labels, node_label_pos, edge_labels):
    # set the size of the plot
    plt.figure(figsize=(16, 13))
    # draw the graph (both nodes and edges) with locations
    nx.draw(G, pos=locations, node_color=[node_colors[node] for node in G.nodes()])

    # draw labels for nodes
    node_label_handles = nx.draw_networkx_labels(G, pos=node_label_pos, labels=node_labels, font_size=9)
    # add a white bounding box behind the node labels
    [label.set_bbox(dict(facecolor='white', edgecolor='none')) for label in node_label_handles.values()]

    # add edge lables to the graph
    nx.draw_networkx_edge_labels(G, pos=locations, edge_labels=edge_labels, font_size=8)

    # add a legend
    white_circle = lines.Line2D([], [], color="white", marker='o', markersize=15, markerfacecolor="white")
    orange_circle = lines.Line2D([], [], color="white", marker='o', markersize=15, markerfacecolor="orange")
    red_circle = lines.Line2D([], [], color="white", marker='o', markersize=15, markerfacecolor="red")
    gray_circle = lines.Line2D([], [], color="white", marker='o', markersize=15, markerfacecolor="gray")
    green_circle = lines.Line2D([], [], color="white", marker='o', markersize=15, markerfacecolor="green")
    plt.legend((white_circle, orange_circle, red_circle, gray_circle, green_circle),
               ('Un-explored', 'Frontier', 'Currently exploring', 'Explored', 'Solution path'),
               numpoints=1, prop={'size': 16}, loc=(.8, 1.0))
    plt.show()


def final_path_colors(problem, solution, initial_node_colors):
    "returns a node_colors dict of the final path provided the problem and solution"

    # get initial node colors
    final_colors = dict(initial_node_colors)
    # color all the nodes in solution and starting node to green
    final_colors[problem.initial] = "green"
    for node in solution:
        final_colors[node] = "green"
    return final_colors


def display_visual(user_input, algorithm=None, problem=None):
    if user_input == False:
        def slider_callback(iteration):
            # don't show graph for the first time running the cell calling this function
            try:
                show_map(all_node_colors[iteration])
            except:
                pass

        def visualize_callback(Visualize):
            if Visualize is True:
                button.value = False

                global all_node_colors

                iterations, all_node_colors, node = algorithm(problem)
                solution = node.solution()
                all_node_colors.append(final_path_colors(problem, solution))

                slider.max = len(all_node_colors) - 1

                for i in range(slider.max + 1):
                    slider.value = i
                    # time.sleep(3.)

        slider = widgets.IntSlider(min=0, max=1, step=1, value=0)
        slider_visual = widgets.interactive(slider_callback, iteration=slider)
        display(slider_visual)

        button = widgets.ToggleButton(value=False)
        button_visual = widgets.interactive(visualize_callback, Visualize=button)
        display(button_visual)

#--------------------------------------------------------------------------------------
#-- This class implements the Realistic Agent --#
class AgentRealistic:
    
    def __init__(self,agent_host,agent_port, mission_type, mission_seed, solution_report, state_space_graph):
        """ Constructor for the realistic agent """
        self.AGENT_MOVEMENT_TYPE = 'Discrete' # HINT: You can change this if you want {Absolute, Discrete, Continuous}
        self.AGENT_NAME = 'Realistic'
        self.AGENT_ALLOWED_ACTIONS = ["movenorth 1", "movesouth 1", "movewest 1", "moveeast 1"]
         
        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type        
        self.state_space = None # NOTE: The Realistic can not know anything about the state_space a prior i !
        self.solution_report = solution_report  # Python is call by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)     

        self.training = True
        self.q_table = {}
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)

        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        self.logger.addHandler(ch)

        self.alpha = 0.1;
        self.epsillon = 0.01;
        self.gamma = 1.0;
        self.scale = 40
        self.world_x = 0
        self.world_y = 0
        if (mission_type == "small"):
            self.world_x = 10
            self.world_y = 10
        elif (mission_type == "medium"):
            self.world_x = 20
            self.world_y = 20
        else:
            self.world_x = 40
            self.world_y = 40
        self.root = tk.Tk()
        self.root.wm_title("Q-table")
        self.canvas = tk.Canvas(self.root, width=self.world_x*self.scale, height=self.world_y*self.scale, borderwidth=0, highlightthickness=0, bg="black")
        self.canvas.grid()
        self.root.update()

    #----------------------------------------------------------------------------------------------------------------#       
    def __ExecuteActionForRealisticAgentWithNoisyTransitionModel__(idx_requested_action, noise_level):     
        """ Creates a well-defined transition model with a certain noise level """                  
        n = len(self.AGENT_ALLOWED_ACTIONS)     
        pp = noise_level/(n-1) * np.ones((n,1))
        pp[idx_request_action] = 1.0 - noise_level
        idx_actual = np.random.choice(n, 1, p=pp.flatten()) # sample from the distribution of actions 
        actual_action = self.AGENT_ALLOWED_ACTIONS[int(idx_actual)]         
        self.agent_host.sendCommand(actual_action) 
        return actual_action   
  
    #----------------------------------------------------------------------------------------------------------------#        

    def stop_training(self):
        self.training = False

    def run_agent(self):
        """ Run the Realistic agent and log the performance and resource use """       
       
        #-- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')            
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)            
        self.solution_report.setMissionXML(mission_xml)        
        time.sleep(1)
        self.solution_report.start()

        # INSERT YOUR SOLUTION HERE (REWARDS MUST BE UPDATED IN THE solution_report)
        #
        # NOTICE: YOUR FINAL AGENT MUST MAKE USE OF THE FOLLOWING NOISY TRANSISION MODEL  
        #       ExecuteActionForRealisticAgentWithNoisyTransitionModel(idx_requested_action, 0.05)
        #   FOR DEVELOPMENT IT IS RECOMMENDED TO FIST USE A NOISE FREE VERSION, i.e.  
        #       ExecuteActionForRealisticAgentWithNoisyTransitionModel(idx_requested_action, 0.0)

        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)
        self.agent_host.setRewardsPolicy(MalmoPython.RewardsPolicy.KEEP_ALL_REWARDS)

        state_t = self.agent_host.getWorldState()
        reward_cumulative = 0.0

        self.prev_state = None
        self.prev_action = None

        # Check if anything went wrong along the way
        for error in state_t.errors:
            print("Error:", error.text)

        obs = json.loads(state_t.observations[-1].text)
        prev_x = obs[u'XPos']
        prev_z = obs[u'ZPos']
        print 'Initial position:', prev_x, ',', prev_z

        # take first action
        reward_cumulative += self.take_action(state_t, agent_host, 0)

        while state_t.is_mission_running:
            # Wait 0.5 sec
            time.sleep(0.25)

            # raw_input('Press enter to continue....')
            # Get the world state
            state_t = self.agent_host.getWorldState()

            # Collect the number of rewards and add to reward_cumulative
            # Note: Since we only observe the sensors and environment every a number of rewards may have accumulated in the buffer
            current_reward = 0
            for reward_t in state_t.rewards:
                print "Reward_t:", reward_t.getValue()
                current_reward += reward_t.getValue()
                reward_cumulative += reward_t.getValue()
                self.solution_report.addReward(reward_t.getValue(), datetime.datetime.now())
                print("Cummulative reward so far:", reward_cumulative)

            # Check if anything went wrong along the way
            for error in state_t.errors:
                print("Error:", error.text)

            # Handle the sensor input
            xpos = None
            ypos = None
            zpos = None
            yaw = None
            pitch = None
            if state_t.number_of_observations_since_last_state > 0:  # Has any Oracle-like and/or internal sensor observations come in?
                msg = state_t.observations[-1].text  # Get the detailed for the last observed state
                oracle = json.loads(msg)  # Parse the Oracle JSON

                # Orcale
                grid = oracle.get(u'grid', 0)  #

                # GPS-like sensor
                xpos = oracle.get(u'XPos', 0)  # Position in 2D plane, 1st axis
                zpos = oracle.get(u'ZPos', 0)  # Position in 2D plane, 2nd axis (yes Z!)
                ypos = oracle.get(u'YPos', 0)  # Height as measured from surface! (yes Y!)

                self.logger.debug('Agent new position: %f %f' % (xpos, zpos))

                # Standard "internal" sensory inputs
                yaw = oracle.get(u'Yaw', 0)  # Yaw
                pitch = oracle.get(u'Pitch', 0)  # Pitch

                prev_x = xpos
                prev_z = zpos

                reward_cumulative += self.take_action(state_t, agent_host, current_reward)

            # -- Print some of the state information --#
            print("Percept: video,observations,rewards received:", state_t.number_of_video_frames_since_last_state,
                  state_t.number_of_observations_since_last_state, state_t.number_of_rewards_since_last_state)
            print("\tcoordinates (x,y,z,yaw,pitch):" + str(xpos) + " " + str(ypos) + " " + str(zpos) + " " + str(
                yaw) + " " + str(pitch))

            self.drawQ()            
        return

    def take_action(self, state, agent_host, current_reward):

        obs_text = state.observations[-1].text
        obs = json.loads(obs_text)  # most recent observation
        current_state = "%d:%d" % (int(obs[u'XPos']), int(obs[u'ZPos']))
        self.logger.debug("State: %s (x = %.2f, z = %.2f)" % (current_state, float(obs[u'XPos']), float(obs[u'ZPos'])))

        # Initialize table if not initialized
        if not self.q_table.has_key(current_state):
            self.q_table[current_state] = ([0.0] * len(self.AGENT_ALLOWED_ACTIONS))

        # update Q values - Note frequency function is not included.
        if self.training and self.prev_state is not None and self.prev_action is not None:
            self.logger.debug('Updating Q Values... ( r = %f, a = %f, g = %f)' % (current_reward, self.alpha, self.gamma))
            old_q = self.q_table[self.prev_state][self.prev_action]
            self.q_table[self.prev_state][self.prev_action] = old_q + self.alpha * (current_reward
                                                    + self.gamma * max(self.q_table[current_state]) - old_q)
            self.logger.debug('Updated value %f' % self.q_table[self.prev_state][self.prev_action])

        self.drawQ( curr_x = int(obs[u'XPos']), curr_y = int(obs[u'ZPos']) )

        random.seed()
        rnd = random.random()
        if rnd < self.epsillon:
            next_action = random.randint(0, len(self.AGENT_ALLOWED_ACTIONS) - 1)
            self.logger.debug('Random Action: %s' % self.AGENT_ALLOWED_ACTIONS[next_action])
        else:
            m = max(self.q_table[current_state])
            self.logger.debug("Current values: %s" % ",".join(str(x) for x in self.q_table[current_state]))

            #if multiple actions have same utility
            l = list()
            for x in range(0, len(self.AGENT_ALLOWED_ACTIONS)):
                if self.q_table[current_state][x] == m:
                    l.append(x)
            y = random.randint(0, len(l) - 1)
            next_action = l[y]
            self.logger.debug("Taking q action: %s" % self.AGENT_ALLOWED_ACTIONS[next_action])

        # send the selected action
        agent_host.sendCommand(self.AGENT_ALLOWED_ACTIONS[next_action])
        self.prev_state = current_state
        self.prev_action = next_action

        return current_reward
    
    def drawQ( self, curr_x=None, curr_y=None):
        if self.canvas is None or self.root is None:
            return
        self.canvas.delete("all")
        action_inset = 0.1
        action_radius = 0.1
        curr_radius = 0.2
        action_positions = [ ( 0.5, 1-action_inset ), ( 0.5, action_inset ), ( 1-action_inset, 0.5 ), ( action_inset, 0.5 ) ]
        # (NSWE to match action order)
        min_value = -20
        max_value = 20
        for x in range(self.world_x):
            for y in range(self.world_y):
                s = "%d:%d" % (x,y)
                self.canvas.create_rectangle( (self.world_x-1-x)*self.scale, (self.world_y-1-y)*self.scale, (self.world_x-1-x+1)*self.scale, (self.world_y-1-y+1)*self.scale, outline="#fff", fill="#000")
                for action in range(4):
                    if not s in self.q_table:
                        continue
                    value = self.q_table[s][action]
                    color = 255 * ( value - min_value ) / ( max_value - min_value ) # map value to 0-255
                    color = max( min( color, 255 ), 0 ) # ensure within [0,255]
                    color_string = '#%02x%02x%02x' % (255-color, color, 0)
                    self.canvas.create_oval( (self.world_x - 1 - x + action_positions[action][0] - action_radius ) *self.scale,
                                             (self.world_y - 1 - y + action_positions[action][1] - action_radius ) *self.scale,
                                             (self.world_x - 1 - x + action_positions[action][0] + action_radius ) *self.scale,
                                             (self.world_y - 1 - y + action_positions[action][1] + action_radius ) *self.scale, 
                                             outline=color_string, fill=color_string )
        if curr_x is not None and curr_y is not None:
            self.canvas.create_oval( (self.world_x - 1 - curr_x + 0.5 - curr_radius ) * self.scale, 
                                     (self.world_y - 1 - curr_y + 0.5 - curr_radius ) * self.scale, 
                                     (self.world_x - 1 - curr_x + 0.5 + curr_radius ) * self.scale, 
                                     (self.world_y - 1 - curr_y + 0.5 + curr_radius ) * self.scale, 
                                     outline="#fff", fill="#fff" )
        self.root.update()

#--------------------------------------------------------------------------------------
#-- This class implements the Simple Agent --#
class AgentSimple:
      
    def __init__(self,agent_host,agent_port, mission_type, mission_seed, solution_report, state_space):
        """ Constructor for the simple agent """
        self.AGENT_MOVEMENT_TYPE = 'Discrete' # HINT: You can change this if you want {Absolute, Discrete, Continuous}
        self.AGENT_NAME = 'Simple'

        self.agent_host = agent_host
        self.agent_port = agent_port      
        self.mission_seed = mission_seed
        self.mission_type = mission_type        
        self.state_space = state_space; 
        self.solution_report = solution_report;  # Python calls by reference !     
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)     

    def run_agent(self):   
        """ Run the Simple agent and log the performance and resource use """                
        
        #-- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')            
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)            
        self.solution_report.setMissionXML(mission_xml)        
        time.sleep(1)
        self.solution_report.start()

        # -- Define local capabilities of the agent (sensors)--#
        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)
        self.agent_host.setRewardsPolicy(MalmoPython.RewardsPolicy.KEEP_ALL_REWARDS)

        # INSERT: YOUR SOLUTION HERE (REMEMBER TO MANUALLY UPDATE THE solution_report DEPENDING ON YOU SOLUTION)


        # initialise a graph
        maze_map = UndirectedGraph(state_space.state_actions)

        G = nx.Graph()

        node_labels = dict()
        node_colors = dict()

        for n, p in state_space.state_locations.items():
            G.add_node(n)  # add nodes from locations
            node_labels[n] = n  # add nodes to node_labels
            node_colors[n] = "white"  # node_colors to color nodes while exploring the map

        # we'll save the initial node colors to a dict for later use
        self.initial_node_colors = dict(node_colors)

        # positions for node labels
        node_label_pos = {k: [v[0], v[1] - 0.25] for k, v in
                          state_space.state_locations.items()}  # spec the position of the labels relative to the nodes

        # use this while labeling edges
        edge_labels = dict()

        # add edges between nodes in the map - UndirectedGraph defined in search.py
        for node in maze_map.nodes():
            connections = maze_map.get(node)
            for connection in connections.keys():
                distance = connections[connection]
                G.add_edge(node, connection)  # add edges to the graph
                edge_labels[(node, connection)] = distance  # add distances to edge_labels

        print("Done creating the graph object")
        # show_map(node_colors, G, state_space.state_locations, node_labels, node_label_pos, edge_labels)

        initial_state = 'S_' + str(state_space.start_loc[0]) + '_' + str(state_space.start_loc[1])
        goal_state = 'S_' + str(state_space.goal_loc[0]) + '_' + str(state_space.goal_loc[1])


        maze_problem = GraphProblem(initial_state, goal_state, maze_map)
        print("Initial state:" + maze_problem.initial)
        print("Goal state:" + maze_problem.goal)

        all_node_colors = []
        iterations, all_node_colors, node = self.astar_search(problem=maze_problem, h=None)

        # -- Trace the solution --#
        solution_path = [node]
        cnode = node.parent
        solution_path.append(cnode)
        while cnode.state != initial_state:
            cnode = cnode.parent
            solution_path.append(cnode)

        print("----------------------------------------")
        print("Identified goal state:" + str(solution_path[0]))
        print("----------------------------------------")
        print("Solution trace:" + str(solution_path))
        print("----------------------------------------")
        print("Final solution path:")

        # show_map(final_path_colors(maze_problem, node.solution(), self.initial_node_colors), G, state_space.state_locations, node_labels, node_label_pos, edge_labels)

        solution_path_local = deepcopy(solution_path)
        reward_cumulative = 0.0
        state_t = self.agent_host.getWorldState()

        x_old = state_space.start_loc[0]
        z_old = state_space.start_loc[1]
        while state_t.is_mission_running:

            target_node = solution_path_local.pop()
            try:
                print("Action_t: Goto state " + target_node.state)
                xz_new = state_space.state_locations.get(target_node.state)
                x_new = xz_new[0]
                z_new = xz_new[1]
                if x_old == x_new and z_old == z_new:
                    pass
                else:
                    if x_new > x_old:
                        command = "moveeast"
                    elif x_new < x_old:
                        command = "movewest"
                    elif z_new > z_old:
                        command = "movesouth"
                    else:
                        command = "movenorth"

                    self.agent_host.sendCommand(command + " 1")
                    x_old = x_new
                    z_old = z_new


            except RuntimeError as e:
                print "Failed to send command:", e
                pass

            # Wait 0.5 sec
            time.sleep(0.5)

            # Get the world state
            state_t = self.agent_host.getWorldState()

            # Collect the number of rewards and add to reward_cumulative
            # Note: Since we only observe the sensors and environment every a number of rewards may have accumulated in the buffer
            for reward_t in state_t.rewards:
                print "Reward_t:", reward_t.getValue()
                reward_cumulative += reward_t.getValue()
                self.solution_report.addReward(reward_t.getValue(), datetime.datetime.now())
                print("Cummulative reward so far:", reward_cumulative)

            # Check if anything went wrong along the way
            for error in state_t.errors:
                print("Error:", error.text)

            # Handle the sensor input
            xpos = None
            ypos = None
            zpos = None
            yaw = None
            pitch = None
            if state_t.number_of_observations_since_last_state > 0:  # Has any Oracle-like and/or internal sensor observations come in?
                msg = state_t.observations[-1].text  # Get the detailed for the last observed state
                oracle = json.loads(msg)  # Parse the Oracle JSON

                # Orcale
                grid = oracle.get(u'grid', 0)  #

                # GPS-like sensor
                xpos = oracle.get(u'XPos', 0)  # Position in 2D plane, 1st axis
                zpos = oracle.get(u'ZPos', 0)  # Position in 2D plane, 2nd axis (yes Z!)
                ypos = oracle.get(u'YPos', 0)  # Height as measured from surface! (yes Y!)

                # Standard "internal" sensory inputs
                yaw = oracle.get(u'Yaw', 0)  # Yaw
                pitch = oracle.get(u'Pitch', 0)  # Pitch

            # Vision
            if state_t.number_of_video_frames_since_last_state > 0:  # Have any Vision percepts been registred ?
                frame = state_t.video_frames[0]

            # -- Print some of the state information --#
            print("Percept: video,observations,rewards received:", state_t.number_of_video_frames_since_last_state,
                  state_t.number_of_observations_since_last_state, state_t.number_of_rewards_since_last_state)
            print("\tcoordinates (x,y,z,yaw,pitch):" + str(xpos) + " " + str(ypos) + " " + str(zpos) + " " + str(
                yaw) + " " + str(pitch))

        return

    def astar_search(self, problem, h=None):
        """A* search is best-first graph search with f(n) = g(n)+h(n).
        You need to specify the h function when you call astar_search, or
        else in your Problem subclass."""
        h = memoize(h or problem.h, 'h')
        iterations, all_node_colors, node = self.best_first_graph_search(problem, lambda n: n.path_cost + h(n))
        return (iterations, all_node_colors, node)

    def best_first_graph_search(self, problem, f):
        """Search the nodes with the lowest f scores first.
        You specify the function f(node) that you want to minimize; for example,
        if f is a heuristic estimate to the goal, then we have greedy best
        first search; if f is node.depth then we have breadth-first search.
        There is a subtlety: the line "f = memoize(f, 'f')" means that the f
        values will be cached on the nodes as they are computed. So after doing
        a best first search you can examine the f values of the path returned."""

        # we use these two variables at the time of visualisations
        iterations = 0
        all_node_colors = []
        node_colors = dict(self.initial_node_colors)

        f = memoize(f, 'f')
        node = Node(problem.initial)

        node_colors[node.state] = "red"
        iterations += 1
        all_node_colors.append(dict(node_colors))

        if problem.goal_test(node.state):
            node_colors[node.state] = "green"
            iterations += 1
            all_node_colors.append(dict(node_colors))
            return (iterations, all_node_colors, node)

        frontier = PriorityQueue(min, f)
        frontier.append(node)

        node_colors[node.state] = "orange"
        iterations += 1
        all_node_colors.append(dict(node_colors))

        explored = set()
        while frontier:
            node = frontier.pop()

            node_colors[node.state] = "red"
            iterations += 1
            all_node_colors.append(dict(node_colors))

            if problem.goal_test(node.state):
                node_colors[node.state] = "green"
                iterations += 1
                all_node_colors.append(dict(node_colors))
                return (iterations, all_node_colors, node)

            explored.add(node.state)
            for child in node.expand(problem):
                if child.state not in explored and child not in frontier:
                    frontier.append(child)
                    node_colors[child.state] = "orange"
                    iterations += 1
                    all_node_colors.append(dict(node_colors))
                elif child in frontier:
                    incumbent = frontier[child]
                    if f(child) < f(incumbent):
                        del frontier[incumbent]
                        frontier.append(child)
                        node_colors[child.state] = "orange"
                        iterations += 1
                        all_node_colors.append(dict(node_colors))

            node_colors[node.state] = "gray"
            iterations += 1
            all_node_colors.append(dict(node_colors))
        return None


#--------------------------------------------------------------------------------------
#-- This class implements a basic, suboptimal Random Agent. The purpurpose is to provide a baseline for other agent to beat. --#
class AgentRandom:
    
    def __init__(self,agent_host,agent_port, mission_type, mission_seed, solution_report, state_space_graph):
        """ Constructor for the Random agent """
        self.AGENT_MOVEMENT_TYPE = 'Discrete'
        self.AGENT_NAME = 'Random' 
        self.AGENT_ALLOWED_ACTIONS = ["movenorth 1", "movesouth 1", "movewest 1", "moveeast 1"]

        self.agent_host = agent_host
        self.agent_port = agent_port       
        self.mission_seed = mission_seed
        self.mission_type = mission_type        
        self.state_space = state_space;
        self.solution_report = solution_report;   # Python makes call by reference !     
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed) 

    def __ExecuteActionForRandomAgentWithNoisyTransitionModel__(self, idx_request_action, noise_level):     
        """ Creates a well-defined transition model with a certain noise level """                  
        n = len(self.AGENT_ALLOWED_ACTIONS)     
        pp = noise_level/(n-1) * np.ones((n,1))
        pp[idx_request_action] = 1.0 - noise_level
        idx_actual = np.random.choice(n, 1, p=pp.flatten()) # sample from the distrbution of actions 
        actual_action = self.AGENT_ALLOWED_ACTIONS[int(idx_actual)]         
        self.agent_host.sendCommand(actual_action) 
        return actual_action

    def run_agent(self):   
        """ Run the Random agent and log the performance and resource use """               
        
        #-- Load and init mission --#        
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')            
        mission_xml,reward_goal,reward_intermediate,n_intermediate_rewards,reward_timeout,reward_sendcommand, timeout = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)            
        self.solution_report.setMissionXML(mission_xml)                

        #-- Define local capabilities of the agent (sensors)--#
        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)
        self.agent_host.setRewardsPolicy(MalmoPython.RewardsPolicy.KEEP_ALL_REWARDS)
        
        # Fix the randomness of the agent by seeding the random number generator                
        reward_cumulative = 0.0
        
        # Main loop:
        state_t = self.agent_host.getWorldState()               

        while state_t.is_mission_running:              
            # Wait 0.5 sec 
            time.sleep(0.5)
        
            # Get the world state
            state_t = self.agent_host.getWorldState()
                
            if state_t.is_mission_running:                
                actionIdx = random.randint(0, 3) 
                print("Requested Action:",self.AGENT_ALLOWED_ACTIONS[actionIdx])
                
                # Now try to execute the action givne a noisy transition model
                actual_action = self.__ExecuteActionForRandomAgentWithNoisyTransitionModel__(actionIdx, 0.00);
                print("Actual Action:",actual_action)
                                       
            # Collect the number of rewards and add to reward_cumulative
            # Note: Since we only observe the sensors and environment every a number of rewards may have accumulated in the buffer
            for reward_t in state_t.rewards:              
                reward_cumulative += reward_t.getValue()
                self.solution_report.addReward(reward_t.getValue(), datetime.datetime.now())
                print("Reward_t:",reward_t.getValue())
                print("Cummulative reward so far:",reward_cumulative)

            # Check if anything went wrong along the way
            for error in state_t.errors:
                print("Error:",error.text)

            # Handle the sensor input     
            xpos = None
            ypos = None
            zpos = None
            yaw  = None
            pitch = None
            if state_t.number_of_observations_since_last_state > 0: # Has any Oracle-like and/or internal sensor observations come in?
                msg = state_t.observations[-1].text      # Get the detailed for the last observed state
                oracle = json.loads(msg)                 # Parse the Oracle JSON

                # Orcale
                grid = oracle.get(u'grid', 0)            # 
        
                # GPS-like sensor
                xpos = oracle.get(u'XPos', 0)            # Position in 2D plane, 1st axis
                zpos = oracle.get(u'ZPos', 0)            # Position in 2D plane, 2nd axis (yes Z!)
                ypos = oracle.get(u'YPos', 0)            # Height as measured from surface! (yes Y!)
                
                # Standard "internal" sensory inputs
                yaw  = oracle.get(u'Yaw', 0)             # Yaw
                pitch = oracle.get(u'Pitch', 0)          # Pitch        
       
            # Vision
            if state_t.number_of_video_frames_since_last_state > 0: # Have any Vision percepts been registred ?
                frame = state_t.video_frames[0]
        
            #-- Print some of the state information --#
            print("Percept: video,observations,rewards received:",state_t.number_of_video_frames_since_last_state,state_t.number_of_observations_since_last_state,state_t.number_of_rewards_since_last_state)        
            print("\tcoordinates (x,y,z,yaw,pitch):" + str(xpos) + " " + str(ypos) + " " + str(zpos)+ " " + str(yaw) + " " + str(pitch))

        # --------------------------------------------------------------------------------------------   
        # Summary
        print("Summary:")        
        print("Cumulative reward = " + str(reward_cumulative) )

        return


#--------------------------------------------------------------------------------------
#-- This class implements a helper Agent for deriving the state-space representation ---#
class AgentHelper:
    """ This agent determines the state space for use by the actual problem solving agent. Enabeling do_plot will allow you to visualize the results """
        
    def __init__(self,agent_host,agent_port, mission_type, mission_seed, solution_report, state_space_graph):
        """ Constructor for the helper agent """
        self.AGENT_NAME = 'Helper'
        self.AGENT_MOVEMENT_TYPE = 'Absolute' # Note the helper needs absolute movements
        self.DO_PLOT = False

        self.agent_host = agent_host
        self.agent_port = agent_port
        self.mission_seed = mission_seed
        self.mission_type = mission_type        
        self.state_space = StateSpace()
        self.solution_report = solution_report;   # Python is call by reference !     
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)     
                                     
    def run_agent(self):   
        """ Run the Helper agent to get the state-space """               
                
        #-- Load and init the Helper mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(self.mission_seed) + ' allowing ' +  self.AGENT_MOVEMENT_TYPE + ' movements')            
        mission_xml,reward_goal,reward_intermediate,n_intermediate_rewards,reward_timeout,reward_sendcommand, timeout = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type, self.mission_seed, self.AGENT_MOVEMENT_TYPE)            
        self.solution_report.setMissionXML(mission_xml)
                
        #-- Define local capabilities of the agent (sensors)--#        
        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)
        self.agent_host.setRewardsPolicy(MalmoPython.RewardsPolicy.KEEP_ALL_REWARDS)
     
        time.sleep(1)

        #-- Get the state of the world along with internal agent state...--#
        state_t = self.agent_host.getWorldState()    
        
        #-- Get a state-space model by observing the Orcale/GridObserver--#
        if state_t.is_mission_running:       
            #-- Make sure we look in the right direction when observing the surrounding (otherwise the coordinate system will rotated by the Yaw !) --#
            # Look East (towards +x (east) and +z (south) on the right, i.e. a std x,y coordinate system) yaw=-90
            self.agent_host.sendCommand("setPitch 20")     
            time.sleep(1)   
            self.agent_host.sendCommand("setYaw -90")                    
            time.sleep(1)      
              
            #-- Basic map --#
            state_t = self.agent_host.getWorldState() 
                                           
            if state_t.number_of_observations_since_last_state > 0:
                msg = state_t.observations[-1].text                 # Get the details for the last observed state
                oracle_and_internal = json.loads(msg)               # Parse the Oracle JSON        
                grid = oracle_and_internal.get(u'grid', 0)          
                xpos = oracle_and_internal.get(u'XPos', 0)          
                zpos = oracle_and_internal.get(u'ZPos', 0)          
                ypos = oracle_and_internal.get(u'YPos', 0)          
                yaw  = oracle_and_internal.get(u'Yaw', 0)            
                pitch = oracle_and_internal.get(u'Pitch', 0)    
               
                #-- Parste the JOSN string, Note there are better ways of doing this! --#               
                full_state_map_raw = str(grid)   
                full_state_map_raw=full_state_map_raw.replace("[","")
                full_state_map_raw=full_state_map_raw.replace("]","")
                full_state_map_raw=full_state_map_raw.replace("u'","")
                full_state_map_raw=full_state_map_raw.replace("'","")
                full_state_map_raw=full_state_map_raw.replace(" ","")
                aa=full_state_map_raw.split(",")
                vocs = list(set(aa))                
                for word in vocs:                   
                    for i in range(0,len(aa)):
                        if aa[i]==word : 
                            aa[i] = vocs.index(word)
                
                X = np.asarray(aa);                
                nn = int(math.sqrt(X.size))
                X = np.reshape(X, [nn,nn]) # Note: this matrix/table is index as z,x

                #-- Visualize the discrete state-space --#
                if self.DO_PLOT:
                    print(yaw)
                    plt.figure(1)
                    imgplot = plt.imshow(X.astype('float'),interpolation='none')                    
                    plt.show()
       
                #-- Define the unique states available --#
                state_wall = vocs.index("stained_hardened_clay")
                state_impossible = vocs.index("stone")
                state_initial = vocs.index("emerald_block")
                state_goal = vocs.index("redstone_block")

                #-- Extract state-space --#
                offset_x = 100-math.floor(xpos);
                offset_z = 100-math.floor(zpos);
                
                state_space_locations = {}; # create a dict
                
                for i_z in range(0,len(X)):
                    for j_x in range(0,len(X)):                        
                        if X[i_z,j_x] != state_impossible and X[i_z,j_x] != state_wall:
                            state_id = "S_"+str(int(j_x - offset_x))+"_"+str(int(i_z - offset_z) )
                            state_space_locations[state_id] = (int(j_x- offset_x),int(i_z - offset_z) )
                            if X[i_z,j_x] == state_initial:
                                state_initial_id = state_id                             
                                loc_start = state_space_locations[state_id]
                            elif X[i_z,j_x] == state_goal:
                                state_goal_id = state_id  
                                loc_goal = state_space_locations[state_id]               
                                                                                                                              
                #-- Generate state / action list --#
                # First define the set of actions in the defined coordinate system             
                actions = {"west": [-1,0],"east": [+1,0],"north": [0,-1], "south": [0,+1]}
                state_space_actions = {}
                for state_id in state_space_locations:                                       
                    possible_states = {}
                    for action in actions:
                        #-- Check if a specific action is possible --#
                        delta = actions.get(action)
                        state_loc = state_space_locations.get(state_id)
                        state_loc_post_action = [state_loc[0]+delta[0],state_loc[1]+delta[1]]

                        #-- Check if the new possible state is in the state_space, i.e., is accessible --#
                        state_id_post_action = "S_"+str(state_loc_post_action[0])+"_"+str(state_loc_post_action[1])                        
                        if state_space_locations.get(state_id_post_action) != None:
                            possible_states[state_id_post_action] = 1 
                        
                    #-- Add the possible actions for this state to the global dict --#                              
                    state_space_actions[state_id] = possible_states
                
                #-- Kill the agent/mission --#                                                  
                agent_host.sendCommand("tp " + str(0 ) + " " + str(0) + " " + str(0))            
                time.sleep(2)

                #-- Save the info an instance of the StateSpace class --
                self.state_space.state_actions = state_space_actions
                self.state_space.state_locations = state_space_locations
                self.state_space.start_id = state_initial_id
                self.state_space.start_loc = loc_start
                self.state_space.goal_id  = state_goal_id 
                self.state_space.goal_loc = loc_goal
                
            #-- Reward location and values --#
            # OPTIONAL: If you want to account for the intermediate rewards 
            # in the Random/Simple agent (or in your analysis) you can 
            # obtain ground-truth by teleporting with the tp command 
            # to all states and detect whether you recieve recieve a 
            # diamond or not using the inventory field in the oracle variable                        
            #
            # As default the state_space_rewards is just set to contain 
            # the goal state which is found above.
            #                               
            state_space_rewards = {}
            state_space_rewards[state_goal_id] = reward_goal   
            
            # HINT: You can insert your own code for getting 
            # the location of the intermediate rewards
            # and populate the state_space_rewards dict 
            # with more information (optional). 
            # WARNING: This is a bit tricky, please consult tutors before starting
                                                               
            #-- Set the values in the state_space container --#
            self.state_space.reward_states = state_space_rewards                                
            self.state_space.reward_states_n = n_intermediate_rewards + 1
            self.state_space.reward_timeout = reward_timeout
            self.state_space.timeout = timeout
            self.state_space.reward_sendcommand = reward_sendcommand                
        else:
            self.state_space = None     
            #-- End if observations --#

        return

# --------------------------------------------------------------------------------------------   
#-- The main entry point if you run the module as a script--#
if __name__ == "__main__":     

    #-- Define default arguments, in case you run the module as a script --#
    DEFAULT_STUDENT_GUID = '2126280p'
    DEFAULT_AGENT_NAME   = 'Realistic' #HINT: Currently choose between {Random,Simple, Realistic}
    DEFAULT_MALMO_PATH   = '/home/sanilo/Desktop/Level4/AI/tools/Malmo/' # HINT: Change this to your own path
    DEFAULT_AIMA_PATH    = '/home/sanilo/Desktop/Level4/AI/tools/aima-python/'  # HINT: Change this to your own path, forward slash only, should be the 2.7 version from https://www.dropbox.com/s/vulnv2pkbv8q92u/aima-python_python_v27_r001.zip?dl=0) or for Python 3.x get it from https://github.com/aimacode/aima-python
    DEFAULT_MISSION_TYPE = 'small'  #HINT: Choose between {small,medium,large}
    DEFAULT_MISSION_SEED_MAX = 1    #HINT: How many different instances of the given mission (i.e. maze layout)    
    DEFAULT_REPEATS      = 1        #HINT: How many repetitions of the same maze layout
    DEFAULT_PORT         = 0
    DEFAULT_SAVE_PATH    = './results/'

    #-- Import required modules --#
    import os
    import sys
    import time
    import random
    from random import shuffle
    import json
    import argparse
    import pickle
    import datetime
    import math 
    import numpy as np
    from copy import deepcopy 
    import hashlib
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg
    import networkx as nx    
    from matplotlib import lines
    import Tkinter as tk

            
    #-- Define the commandline arguments required to run the agents from command line --#
    parser = argparse.ArgumentParser()
    parser.add_argument("-a" , "--agentname"        , type=str, help="path for the malmo pyhton examples"   , default=DEFAULT_AGENT_NAME)
    parser.add_argument("-t" , "--missiontype"      , type=str, help="mission type (small,medium,large)"    , default=DEFAULT_MISSION_TYPE)
    parser.add_argument("-s" , "--missionseedmax"   , type=int, help="maximum mission seed value (integer)"           , default=DEFAULT_MISSION_SEED_MAX)
    parser.add_argument("-n" , "--nrepeats"         , type=int, help="repeat of a specific agent (if stochastic behavior)"  , default=DEFAULT_REPEATS)
    parser.add_argument("-g" , "--studentguid"      , type=str, help="student guid"                         , default=DEFAULT_STUDENT_GUID)
    parser.add_argument("-p" , "--malmopath"        , type=str, help="path for the malmo pyhton examples"   , default=DEFAULT_MALMO_PATH)
    parser.add_argument("-x" , "--malmoport"        , type=int, help="special port for the Minecraft client", default=DEFAULT_PORT)
    parser.add_argument("-o" , "--aimapath"         , type=str, help="path for the aima toolbox (optional)"   , default=DEFAULT_AIMA_PATH)
    parser.add_argument("-r" , "--resultpath"       , type=str, help="the path where the results are saved" , default=DEFAULT_SAVE_PATH)
    args = parser.parse_args()        
    print args     

    #-- Display infor about the system --#     
    print("Working dir:"+os.getcwd())    
    print("Python version:"+sys.version)
    print("malmopath:"+args.malmopath)
    print("JAVA_HOME:'"+os.environ["JAVA_HOME"]+"'")
    print("MALMO_XSD_PATH:'"+os.environ["MALMO_XSD_PATH"]+"'")

    sys.path.append(DEFAULT_AIMA_PATH)
    from search import *

    #-- Add the Malmo path  --#
    print('Add Malmo Python API/lib to the Python environment ['+args.malmopath+'/Python_Examples'+']')    
    sys.path.append(args.malmopath+'Python_Examples/') 
    
    #-- Import the Malmo Python wrapper/module --#
    print('Import the Malmo module...')
    import MalmoPython

    #-- OPTIONAL: Import the AIMA tools (for representing the state-space)--#
    #print('Add AIMA lib to the Python environment ['+args.aimapath+']')
    #sys.path.append(args.aimapath+'/')    
    #from search import *
    
    #-- Create the command line string for convenience --#
    cmd = 'python myagents.py -a ' + args.agentname + ' -s ' + str(args.missionseedmax) + ' -n ' + str(args.nrepeats) + ' -t ' + args.missiontype + ' -g ' + args.studentguid + ' -p ' + args.malmopath + ' -x ' + str(args.malmoport)
    print(cmd)    
       
    #-- Run the agent a number of times (it only makes a difference if you agent has some random elemnt to it, initalizaiton, behavior, etc.) --#
    #-- HINT: It is quite important that you understand the need for the loops  --#    
    #-- HINT: Depending on how you implement your realistic agent in terms of restarts and repeats, you may want to change the way the loops operate --#
    
    print('Instantiate an agent interface/api to Malmo')
    agent_host = MalmoPython.AgentHost()

    #-- Itereate a few different layout of the same mission stype --#
    for i_training_seed in range(0,args.missionseedmax):
        
        #-- Observe the full state space a prior i (only allowed for the simple agent!) ? --#
        if args.agentname.lower()=='simple':  
            print('Get state-space representation using a AgentHelper...[note in v0.30 there is now an faster way of getting the state-space ]')            
            helper_solution_report = SolutionReport()
            helper_agent = AgentHelper(agent_host,args.malmoport,args.missiontype,i_training_seed, helper_solution_report, None)
            helper_agent.run_agent()
        else:
            helper_agent = None            
       
        solution_report = SolutionReport()
        print('Get an instance of the specific ' + args.agentname + ' agent with the agent_host and load the ' + args.missiontype + ' mission with seed ' + str(i_training_seed))
        agent_name = 'Agent' + args.agentname        
        state_space = None;
        if not helper_agent==None:
            state_space = deepcopy(helper_agent.state_space)                            
        
        agent_to_be_evaluated = eval(agent_name+'(agent_host,args.malmoport,args.missiontype,i_training_seed,solution_report,state_space)') 

        #-- Repeat the same instance (size and seed) multiple times --#
        for i_rep in range(0,args.nrepeats):                                   
            print('Setup the performance log...')
            
            solution_report.setStudentGuid(args.studentguid)
            
            print('Run the agent, time it and log the performance...')
            solution_report.start() # start the timer (may be overwritten in the agent to provide a fair comparison)            
            agent_to_be_evaluated.run_agent()                  
            solution_report.stop() # stop the timer
            
            print("\n---------------------------------------------")
            print("| Solution Report Summary: ")
            print("|\tCumulative reward = " + str(solution_report.reward_cumulative))    
            print("|\tDuration (wallclock) = " + str((solution_report.end_datetime_wallclock-solution_report.start_datetime_wallclock).total_seconds()))    
            print("|\tNumber of reported actions = " + str(solution_report.action_count))    
            print("|\tFinal goal reached = " + str(solution_report.is_goal))    
            print("|\tTimeout = " + str(solution_report.is_timeout))    
            print("---------------------------------------------\n")

            print('Save the solution report to a specific file for later analysis and reporting...')
            fn_result = args.resultpath + 'solution_' + args.studentguid + '_' + agent_name + '_' +args.missiontype + '_' + str(i_training_seed) + '_' + str(i_rep) 
            foutput = open(fn_result+'.pkl', 'wb')
            pickle.dump(agent_to_be_evaluated.solution_report,foutput) # Save the solution information in a specific file, HiNT:  It can be loaded with pickle.load(output) with read permissions to the file
            foutput.close()

            # You can reload the results for this instance using...
            #finput = open(fn_result+'.pkl', 'rb')
            #res =  pickle.load(finput)
            #finput.close()
            
            print('Sleep a sec to make sure the client is ready for next mission/agent variation...')            
            time.sleep(1)
            print("------------------------------------------------------------------------------\n")

    print("Done")