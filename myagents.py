""" 
 Artificial Inteligence (H)
 Assessed Exercise 2017/2018

 Tested with Python 2.7
 
 Solution Template (revision a)
"""

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
        self.intermediate_reward_states = None


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
        self.solution_report = solution_report;   # Python is call by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)

        self.training = True
        self.init_logger()
        self.visualize = False

        self.gamma = 0.9
        self.state_space = state_space_graph    # used for running offline
        self.q_learning_agent = QLearningAgent(self.AGENT_ALLOWED_ACTIONS, self.gamma, 2, 1500, 9)
        self.q_learning_visualization = None

    def __ExecuteActionForRealisticAgentWithNoisyTransitionModel__(self, idx_requested_action, noise_level):
        """ Creates a well-defined transition model with a certain noise level """                  
        n = len(self.AGENT_ALLOWED_ACTIONS)     
        pp = noise_level/(n-1) * np.ones((n,1))
        pp[idx_requested_action] = 1.0 - noise_level
        idx_actual = np.random.choice(n, 1, p=pp.flatten()) # sample from the distribution of actions 
        actual_action = self.AGENT_ALLOWED_ACTIONS[int(idx_actual)]         
        if self.state_space == None:    # That means we are using online training
            self.agent_host.sendCommand(actual_action)
        return actual_action

    def set_visualize(self, visualize):
        self.visualize = visualize
        if self.visualize:
            self.q_learning_visualization = QLearningVisualization(self.mission_type, self.q_learning_agent.Q,
                                                                   self.AGENT_ALLOWED_ACTIONS)

    def stop_training(self):
        self.training = False

    def run_agent(self):           
        """ Run the Realistic agent and log the performance and resource use """

        self.agent_host.setObservationsPolicy(MalmoPython.ObservationsPolicy.LATEST_OBSERVATION_ONLY)
        self.agent_host.setVideoPolicy(MalmoPython.VideoPolicy.LATEST_FRAME_ONLY)
        self.agent_host.setRewardsPolicy(MalmoPython.RewardsPolicy.KEEP_ALL_REWARDS)

        # state_t = self.agent_host.getWorldState()
        reward_cumulative = 0.0
        # initial_x, initial_z = self.get_agent_position(state_t)
        # self.logger.debug('Initial position: %s, %s' % (initial_x, initial_z))

        if self.state_space is not None:
            initial_x, initial_z = self.state_space.start_loc[0], self.state_space.start_loc[1]
            self.run_agent_offline(initial_x, initial_z)
        else:
            self.load_and_init_mission()
            state_t = self.agent_host.getWorldState()
            self.run_agent_online(state_t)
        return

    def get_agent_position(self, state_t):
        # Check if anything went wrong along the way
        for error in state_t.errors:
            self.logger.debug("Error:", error.text)

        obs = json.loads(state_t.observations[-1].text)
        x = obs[u'XPos']
        z = obs[u'ZPos']
        return int(x), int(z)

    def run_agent_offline(self, initial_x, initial_z):
        self.logger.debug('Running agent offline...')
        reward_sendcommand = self.state_space.reward_sendcommand
        reward_goal = self.state_space.reward_states[self.state_space.goal_id]
        state = (int(initial_x), int(initial_z))
        state_str = "S_%d_%d" % (int(initial_x), int(initial_z))
        goal_state = [self.state_space.goal_loc[0], self.state_space.goal_loc[1]]

        iterations = self.get_iterations_by_mission_type()    #Simulate timeout
        reward = 0
        i = 0
        while(i < iterations):
            # Simulate perception
            percept = {}
            percept['state'] = state_str
            percept['reward'] = reward
            percept['state_actions'] = []
            for action in state_space.state_actions[state_str].keys():
                action_slpit = action.split('_')
                action_tuple = (int(action_slpit[1]), int(action_slpit[2]))
                if action_tuple[0] < state[0]:
                    percept['state_actions'].append("movewest 1")
                elif action_tuple[0] > state[0]:
                    percept['state_actions'].append("moveeast 1")
                elif action_tuple[1] < state[1]:
                    percept['state_actions'].append("movenorth 1")
                else:
                    percept['state_actions'].append("movesouth 1")

            if (state == goal_state):
                reward += reward_goal
                percept['reward'] = reward
                self.logger.debug('Got to goal! Reward: %s' % reward)
                self.solution_report.addReward(reward, datetime.datetime.now())
                self.q_learning_agent.finished(reward)
                self.q_learning_agent(percept, self.training)
                self.q_learning_agent.reset()
                if self.q_learning_visualization != None:
                    self.q_learning_visualization.drawQ(curr_x=state[0], curr_y=state[1])
                break

            self.solution_report.addReward(reward, datetime.datetime.now())
            self.logger.debug('Updating Q table...')
            a = self.q_learning_agent(percept, self.training)
            self.logger.debug('Utilities at state: (%s %s)' % (state[0], state[1]))
            self.logger.debug(self.q_learning_agent.all_utilities(state_str))
            if self.q_learning_visualization != None:
                self.q_learning_visualization.drawQ(curr_x=state[0], curr_y=state[1])
            self.logger.debug('Requested action: %s' % a)
            idx_requested_action = self.get_action_index(a)
            action = self.__ExecuteActionForRealisticAgentWithNoisyTransitionModel__(idx_requested_action, 0.05)
            self.logger.debug('Taking action: %s' % action)
            state, state_str = self.update_state_offline(state, action)
            self.solution_report.addAction()
            reward = reward_sendcommand
            self.logger.debug('New agent state: %s, %s', state[0], state[1])
            if self.q_learning_visualization != None:
                self.q_learning_visualization.drawQ()
            i += 1

    def run_agent_online(self, state_t):
        self.logger.debug('Running agent online...')
        state_t = self.agent_host.getWorldState()

        while state_t.is_mission_running:
            # Wait 0.5 sec
            time.sleep(0.5)
            # Get the world state
            state_t = self.agent_host.getWorldState()
            print state_t

            reward = 0
            for reward_t in state_t.rewards:
                print "Reward_t:", reward_t.getValue()
                reward += reward_t.getValue()
                self.solution_report.addReward(reward_t.getValue(), datetime.datetime.now())

                # Check if anything went wrong along the way
            for error in state_t.errors:
                print("Error:", error.text)


            if state_t.number_of_observations_since_last_state > 0:  # Has any Oracle-like and/or internal sensor observations come in?
                msg = state_t.observations[-1].text  # Get the detailed for the last observed state
                oracle = json.loads(msg)  # Parse the Oracle JSON
                grid = oracle.get(u'grid', 0)
                state = self.get_agent_position(state_t)
                state_str = "S_%d_%d" % (int(state[0]), int(state[1]))

                percept = {}
                percept['state'] = state_str
                if reward < (-50):
                    self.logger.debug('Agent timed out!')
                    reward = 0  # eliminate timeout reward
                percept['reward'] = reward
                percept['state_actions'] = []

                if grid[3] != 'stained_hardened_clay' and grid[3] != 'stone':
                    percept['state_actions'].append("movewest 1")
                if grid[5] != 'stained_hardened_clay' and grid[5] != 'stone':
                    percept['state_actions'].append("moveeast 1")
                if grid[1] != 'stained_hardened_clay' and grid[1] != 'stone':
                    percept['state_actions'].append("movenorth 1")
                if grid[7] != 'stained_hardened_clay' and grid[7] != 'stone':
                    percept['state_actions'].append("movesouth 1")

                if not state_t.is_mission_running and reward > 50:
                    self.logger.debug('Found goal!')
                    action_simulations = {"movenorth 1": (0, -1), "movesouth 1": (0, 1), "movewest 1": (-1, 0),
                                          "moveeast 1": (1, 0)}
                    state = map(sum, zip(state, action_simulations[action]))
                    self.logger.debug('Current state: %s, %s' % (state[0], state[1]))

                    self.q_learning_agent.finished(reward)
                    self.q_learning_agent(percept, self.training)
                    self.q_learning_agent.reset()
                    if self.q_learning_visualization != None:
                        self.q_learning_visualization.drawQ(curr_x=state[0], curr_y=state[1])
                    break

                self.logger.debug('Current state: %s, %s' % (state[0], state[1]))

                self.logger.debug('Utilities at state: (%s %s)' % (state[0], state[1]))
                self.logger.debug(list(self.q_learning_agent.all_utilities(state_str)))
                self.logger.debug('Updating Q table...')
                a = self.q_learning_agent(percept, self.training)
                if self.q_learning_visualization != None:
                    self.q_learning_visualization.drawQ(curr_x=state[0], curr_y=state[1])
                self.logger.debug('Requested action: %s' % a)
                idx_requested_action = self.get_action_index(a)
                action = self.__ExecuteActionForRealisticAgentWithNoisyTransitionModel__(idx_requested_action, 0.05)
                self.logger.debug('Taking action: %s' % action)
                self.solution_report.addAction()

            # -- Print some of the state information --#
            print("Percept: video,observations,rewards received:", state_t.number_of_video_frames_since_last_state,
                  state_t.number_of_observations_since_last_state, state_t.number_of_rewards_since_last_state)

            if self.q_learning_visualization != None:
                self.q_learning_visualization.drawQ()

        self.logger.debug('Mission finished!')

    def update_state_offline(self, state, action):
        state_str = "S_%d_%d" % (int(state[0]), int(state[1]))
        action_simulations = {"movenorth 1": (0,-1), "movesouth 1": (0, 1), "movewest 1": (-1, 0), "moveeast 1": (1, 0)}
        new_state = map(sum, zip(state, action_simulations[action]))
        new_state_str = "S_%d_%d" % (int(new_state[0]), int(new_state[1]))
        if not (self.state_space.state_actions[state_str].has_key(new_state_str)):
            return state, state_str
        return new_state, new_state_str

    def get_action_index(self, action):
        for i in range(0, len(self.AGENT_ALLOWED_ACTIONS)):
            if self.AGENT_ALLOWED_ACTIONS[i] == action:
                return i

    def get_iterations_by_mission_type(self):
        if self.mission_type == 'small':
            return 120
        elif self.mission_type == 'medium':
            return 400
        elif self.mission_type == 'large':
            return 1800

    def load_and_init_mission(self):
        # -- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(
            self.mission_seed) + ' allowing ' + self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type,
                                   self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)
        time.sleep(1)
        self.solution_report.start()

    def init_logger(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        if not len(self.logger.handlers):
            ch = logging.StreamHandler(sys.stdout)
            ch.setLevel(logging.DEBUG)
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            ch.setFormatter(formatter)
            self.logger.addHandler(ch)


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
        self.solution_report = solution_report  # Python calls by reference !
        self.solution_report.setMissionType(self.mission_type)
        self.solution_report.setMissionSeed(self.mission_seed)
        self.visualize = False
        self.init_logger()

    def run_agent(self):   
        """ Run the Simple agent and log the performance and resource use """                
        self.logger.debug('Running Simple agent...')
        self.load_and_init_mission()
        maze_map = self.create_graph()
        initial_state = 'S_' + str(state_space.start_loc[0]) + '_' + str(state_space.start_loc[1])
        goal_state = 'S_' + str(state_space.goal_loc[0]) + '_' + str(state_space.goal_loc[1])
        solution = self.solve_graph_problem(maze_map, initial_state, goal_state)
        self.execute_solution_on_agent(solution)
        return

    def set_visualize(self, visualize):
        self.visualize = visualize

    def execute_solution_on_agent(self, solution_path_local):
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
                agent_command = self.get_command_from_coords(x_old, z_old, x_new, z_new)
                if (agent_command != None):
                    self.agent_host.sendCommand(agent_command)
                    self.solution_report.addAction()
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

    def get_command_from_coords(self, x_old, z_old, x_new, z_new):
        if x_old == x_new and z_old == z_new:
            return None
        else:
            if x_new > x_old:
                command = "moveeast 1"
            elif x_new < x_old:
                command = "movewest 1"
            elif z_new > z_old:
                command = "movesouth 1"
            else:
                command = "movenorth 1"
        return  command

    def solve_graph_problem(self, maze_map, initial_state, goal_state):
        maze_problem = GraphProblem(initial_state, goal_state, maze_map)

        self.logger.debug('Solving graph problem...')
        self.logger.debug("Initial state:" + maze_problem.initial)
        self.logger.debug("Goal state:" + maze_problem.goal)

        all_node_colors = []
        iterations, all_node_colors, node = self.astar_search(problem=maze_problem, h=None)
        # -- Trace the solution --#
        solution_path = [node]
        cnode = node.parent
        solution_path.append(cnode)
        while cnode.state != initial_state:
            cnode = cnode.parent
            solution_path.append(cnode)

        self.logger.debug("----------------------------------------")
        self.logger.debug("Identified goal state:" + str(solution_path[0]))
        self.logger.debug("----------------------------------------")
        self.logger.debug("Solution trace:" + str(solution_path))
        self.logger.debug("----------------------------------------")
        self.logger.debug("Final solution path:")

        node_colors = self.final_path_colors(maze_problem, node.solution(), self.initial_node_colors)
        if self.visualize:
            self.logger.debug("Visualizing graph solution...")
            self.show_map_graph(self.G, state_space.state_locations, node_colors, self.node_labels,
                                self.node_label_pos, self.edge_labels)
        return deepcopy(solution_path)

    def astar_search(self, problem, h=None):
        """A* search is best-first graph search with f(n) = g(n)+h(n).
        You need to specify the h function when you call astar_search, or
        else in your Problem subclass."""
        h = h or (lambda n: 0)
        iterations, all_node_colors, node = self.best_first_graph_search(problem, lambda n: n.depth + h(n))
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

    def create_graph(self):
        maze_map = UndirectedGraph(state_space.state_actions)
        self.G = nx.Graph()
        self.node_labels = dict()
        node_colors = dict()

        for n, p in state_space.state_locations.items():
            self.G.add_node(n)  # add nodes from locations
            self.node_labels[n] = n  # add nodes to node_labels
            node_colors[n] = "white"  # node_colors to color nodes while exploring the map

        self.initial_node_colors = dict(node_colors)
        self.node_label_pos = {k: [v[0], v[1] - 0.25] for k, v in state_space.state_locations.items()}  # spec the position of the labels relative to the nodes
        self.edge_labels = dict()
        # add edges between nodes in the map - UndirectedGraph defined in search.py
        for node in maze_map.nodes():
            connections = maze_map.get(node)
            for connection in connections.keys():
                distance = connections[connection]
                self.G.add_edge(node, connection)  # add edges to the graph
                self.edge_labels[(node, connection)] = distance  # add distances to edge_labels

        self.logger.debug("Done creating the graph object")

        if self.visualize:
            self.logger.debug("Showing graph visualization")
            self.show_map_graph(self.G, state_space.state_locations, node_colors, self.node_labels, self.node_label_pos, self.edge_labels)

        return maze_map

    def load_and_init_mission(self):
        # -- Load and init mission --#
        print('Generate and load the ' + self.mission_type + ' mission with seed ' + str(
            self.mission_seed) + ' allowing ' + self.AGENT_MOVEMENT_TYPE + ' movements')
        mission_xml = init_mission(self.agent_host, self.agent_port, self.AGENT_NAME, self.mission_type,
                                   self.mission_seed, self.AGENT_MOVEMENT_TYPE)
        self.solution_report.setMissionXML(mission_xml)
        time.sleep(1)
        self.solution_report.start()

    def show_map_graph(self, G, locations, node_colors, node_labels, node_label_pos, edge_labels):
        plt.figure(figsize=(16, 13))
        nx.draw(G, pos=locations, node_color=[node_colors[node] for node in G.nodes()])
        node_label_handles = nx.draw_networkx_labels(G, pos=node_label_pos, labels=node_labels, font_size=9)
        [label.set_bbox(dict(facecolor='white', edgecolor='none')) for label in node_label_handles.values()]
        nx.draw_networkx_edge_labels(G, pos=locations, edge_labels=edge_labels, font_size=8)
        white_circle = lines.Line2D([], [], color="white", marker='o', markersize=15, markerfacecolor="white")
        orange_circle = lines.Line2D([], [], color="white", marker='o', markersize=15, markerfacecolor="orange")
        red_circle = lines.Line2D([], [], color="white", marker='o', markersize=15, markerfacecolor="red")
        gray_circle = lines.Line2D([], [], color="white", marker='o', markersize=15, markerfacecolor="gray")
        green_circle = lines.Line2D([], [], color="white", marker='o', markersize=15, markerfacecolor="green")
        plt.legend((white_circle, orange_circle, red_circle, gray_circle, green_circle),
                   ('Un-explored', 'Frontier', 'Currently exploring', 'Explored', 'Solution path'),
                   numpoints=1, prop={'size': 16}, loc=(.8, 1.0))
        plt.show()

    def final_path_colors(self, problem, solution, initial_node_colors):
        "returns a node_colors dict of the final path provided the problem and solution"

        # get initial node colors
        final_colors = dict(initial_node_colors)
        # color all the nodes in solution and starting node to green
        final_colors[problem.initial] = "green"
        for node in solution:
            final_colors[node] = "green"
        return final_colors

    def init_logger(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        if not len(self.logger.handlers):
            ch = logging.StreamHandler(sys.stdout)
            ch.setLevel(logging.DEBUG)
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            ch.setFormatter(formatter)
            self.logger.addHandler(ch)


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
        self.state_space = state_space
        self.solution_report = solution_report   # Python makes call by reference !
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
        random.seed()
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
                print("Taking Action:",self.AGENT_ALLOWED_ACTIONS[actionIdx])
                # Now try to execute the action
                self.agent_host.sendCommand(self.AGENT_ALLOWED_ACTIONS[actionIdx])
                self.solution_report.addAction()

            for reward_t in state_t.rewards:
                reward_cumulative += reward_t.getValue()
                self.solution_report.addReward(reward_t.getValue(), datetime.datetime.now())
                print("Reward_t:",reward_t.getValue())
                print("Cummulative reward so far:",reward_cumulative)

            for error in state_t.errors:
                print("Error:",error.text)

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
                intermediate_rewards_states = {}
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

                    #-- Explore intermediate rewards
                    try:
                        if state_loc[0] != 7 and state_loc[1] != 5:
                            self.agent_host.sendCommand("tp " + str(state_loc[0]) + ".5" + " 217.0 " + str(state_loc[1]) + ".5")
                            time.sleep(0.3)
                            state_t = self.agent_host.getWorldState()
                            for reward_t in state_t.rewards:
                                if reward_t.getValue() > 0:
                                    intermediate_rewards_states['S_' + str(state_loc[0]) + '_' + str(state_loc[1])] = reward_t.getValue()
                    except e:
                        print e
                
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
                self.state_space.intermediate_reward_states = intermediate_rewards_states
                
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
    DEFAULT_STUDENT_GUID = '2140845P'
    DEFAULT_AGENT_NAME   = 'Simple'
    DEFAULT_MALMO_PATH   = '/Users/Antreas/Desktop/University_Of_Glasgow/Year_4/AI/Malmo-0.30.0-Mac-64bit_withBoost/' # HINT: Change this to your own path
    DEFAULT_AIMA_PATH    = '/Users/Antreas/Desktop/University_Of_Glasgow/Year_4/AI/aima-python/'  # HINT: Change this to your own path, forward slash only, should be the 2.7 version from https://www.dropbox.com/s/vulnv2pkbv8q92u/aima-python_python_v27_r001.zip?dl=0) or for Python 3.x get it from https://github.com/aimacode/aima-python
    DEFAULT_MISSION_TYPE = 'small'  #HINT: Choose between {small,medium,large}
    DEFAULT_MISSION_SEED_MAX = 3    #HINT: How many different instances of the given mission (i.e. maze layout)
    DEFAULT_REPEATS      = 1        #HINT: How many repetitions of the same maze layout
    DEFAULT_PORT         = 0
    DEFAULT_SAVE_PATH    = './results/'
    DEFAULT_RUN_OFFLINE = False
    DEFAULT_VISUALIZE_PROCESS = False
    DEFAULT_LOAD_PATH = None
    DEFAULT_SAVE_TRAIN_PATH = None


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
    import logging
    import hashlib
    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.image as mpimg
    import networkx as nx    
    from matplotlib import lines
    import matplotlib.pyplot as plt
    from QLearningVisualization import QLearningVisualization
    from QLearningAgent import QLearningAgent

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
    parser.add_argument("-f" , "--offline"       , type=bool, help="if this is set to true the realistic agent runs offline." , default=DEFAULT_RUN_OFFLINE)
    parser.add_argument("-v" , "--visualize"       , type=bool, help="if this is set to true the solutions for realistic and simple agents are visualized." , default=DEFAULT_VISUALIZE_PROCESS)
    parser.add_argument("-l" , "--load"       , type=str, help="path to load pickled q-table" , default=DEFAULT_LOAD_PATH)
    parser.add_argument("-e" , "--save"       , type=str, help="path to save pickled q-table" , default=DEFAULT_SAVE_TRAIN_PATH)
    args = parser.parse_args()
    print args     

    #-- Display infor about the system --#     
    print("Working dir:"+os.getcwd())    
    print("Python version:"+sys.version)
    print("malmopath:"+args.malmopath)
    print("JAVA_HOME:'"+os.environ["JAVA_HOME"]+"'")
    print("MALMO_XSD_PATH:'"+os.environ["MALMO_XSD_PATH"]+"'")
        
    #-- Add the Malmo path  --#
    print('Add Malmo Python API/lib to the Python environment ['+args.malmopath+'Python_Examples'+']')
    sys.path.append(args.malmopath+'Python_Examples/') 
    
    #-- Import the Malmo Python wrapper/module --#
    print('Import the Malmo module...')
    import MalmoPython

    #-- Import the AIMA tools (for representing the state-space)--#
    print('Add AIMA lib to the Python environment ['+args.aimapath+']')
    sys.path.append(args.aimapath+'/')
    from search import *
    
    #-- Create the command line string for convenience --#
    cmd = 'python myagents.py -a ' + args.agentname + ' -s ' + str(args.missionseedmax) + ' -n ' + str(args.nrepeats) + ' -t ' + args.missiontype + ' -g ' + args.studentguid + ' -p ' + args.malmopath + ' -x ' + str(args.malmoport)
    print(cmd)

    print('Instantiate an agent interface/api to Malmo')
    agent_host = MalmoPython.AgentHost()

    #-- Itereate a few different layout of the same mission stype --#
    for i_training_seed in range(0,args.missionseedmax):
        #-- Observe the full state space a prior i (only allowed for the simple agent!) ? --#
        if args.agentname.lower()=='simple' or args.offline:
            print('Get state-space representation using a AgentHelper...[note in v0.30 there is now an faster way of getting the state-space ]')            
            helper_solution_report = SolutionReport()
            helper_agent = AgentHelper(agent_host,args.malmoport,args.missiontype,i_training_seed, helper_solution_report, None)
            helper_agent.run_agent()
        else:
            helper_agent = None            
        
        #-- Repeat the same instance (size and seed) multiple times --#
        agent_to_be_evaluated = None
        actions_per_iteration = []
        reward_per_iteration = []
        iterations = range(0,args.nrepeats)
        for i_rep in iterations:

            print('Setup the performance log...')
            solution_report = SolutionReport()
            solution_report.setStudentGuid(args.studentguid)
            
            print('Get an instance of the specific ' + args.agentname + ' agent with the agent_host and load the ' + args.missiontype + ' mission with seed ' + str(i_training_seed))
            agent_name = 'Agent' + args.agentname        
            state_space = None
            if not helper_agent==None:
                state_space = deepcopy(helper_agent.state_space)                            

            if agent_to_be_evaluated == None:   #keep states between iterations
                agent_to_be_evaluated = eval(agent_name+'(agent_host,args.malmoport,args.missiontype,i_training_seed,solution_report,state_space)')
                print 'Load existing training data...'
                if args.agentname.lower() == 'realistic' and args.load != None:
                    fn_load = args.load
                    try:
                        finput = open(fn_load , 'rb')
                        agent_to_be_evaluated.q_learning_agent.Q = pickle.load(finput)
                        finput.close()
                    except IOError as e:
                        print 'Trained data file not found...'

            else:
                agent_to_be_evaluated.solution_report = solution_report

            if args.agentname.lower()=='simple' or args.agentname.lower()=='realistic':
                if agent_to_be_evaluated.visualize != args.visualize:
                    agent_to_be_evaluated.set_visualize(args.visualize)

            if i_rep == (args.nrepeats-1) and state_space != None and args.agentname.lower() == 'realistic':
                agent_to_be_evaluated.state_space = None
                agent_to_be_evaluated.stop_training()

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

            # Save performance measures to use in evaluator
            actions_per_iteration.append(solution_report.action_count)
            reward_per_iteration.append(solution_report.reward_cumulative)

            print('Save the solution report to a specific file for later analysis and reporting...')
            fn_result = args.resultpath + 'solution_' + args.studentguid + '_' + agent_name + '_' +args.missiontype + '_' + str(i_training_seed) + '_' + str(i_rep) 
            foutput = open(fn_result+'.pkl', 'wb')
            pickle.dump(agent_to_be_evaluated.solution_report,foutput) # Save the solution information in a specific file, HiNT:  It can be loaded with pickle.load(output) with read permissions to the file
            foutput.close()

            print('Sleep a sec to make sure the client is ready for next mission/agent variation...')            
            time.sleep(1)
            print("------------------------------------------------------------------------------\n")

        if args.agentname.lower() == 'realistic' and args.save != None:
            print('Saving q_table for later use...')
            save_path = args.save
            foutput = open(save_path, 'wb')
            pickle.dump(agent_to_be_evaluated.q_learning_agent.Q,
                        foutput)  # Save the solution information in a specific file, HiNT:  It can be loaded with pickle.load(output) with read permissions to the file
            foutput.close()

        # print 'Saving eval data...'
        # fn_eval= args.resultpath + 'eval'
        # try:
        #     finput = open(fn_eval + '.pkl', 'rb')
        #     evaluator = pickle.load(finput)
        #     time.sleep(4)
        #     finput.close()
        # except IOError as e:
        #     evaluator = Evaluator()
        # finally:
        #     # evaluator.eval_data['Nf'][str(agent_to_be_evaluated.q_learning_agent.Nf)] = {}
        #     # evaluator.eval_data['Nf'][str(agent_to_be_evaluated.q_learning_agent.Nf)]['actions_per_iter'] = actions_per_iteration
        #     # evaluator.eval_data['Nf'][str(agent_to_be_evaluated.q_learning_agent.Nf)]['reward_per_iter'] = reward_per_iteration
        #     evaluator.eval_data['small'][str(i_training_seed)] = {}
        #     evaluator.eval_data['small'][str(i_training_seed)]['actions_per_iter'] = actions_per_iteration
        #     evaluator.eval_data['small'][str(i_training_seed)]['reward_per_iter'] = reward_per_iteration
        #     evaluator.eval_data['small'][str(i_training_seed)]['finished'] = solution_report.is_goal
        #
        #     foutput = open(fn_eval+'.pkl', 'wb')
        #     print evaluator.__dict__
        #     pickle.dump(evaluator, foutput)
        #     time.sleep(4)
        #     foutput.close()
        #
        # evaluator.plot_evaluation(iterations, actions_per_iteration, reward_per_iteration)


    print("Done")