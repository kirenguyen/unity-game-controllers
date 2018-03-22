"""
This Module handles aspects of the agent architecture's decision-making and gameplay.
"""
# pylint: disable=import-error
import random
from ..RobotBehaviorList.RobotBehaviorList import RobotRoles
from ..RobotBehaviorList.RobotBehaviorList import RobotRolesBehaviorsMap

# reinforcement learning modules
from .RLAgent import *
from .RLiSpyEnvironment import *

class EnvBuilder():
    def __init__(self,child_id):
        self.A_map = {'EXPERT':0, 'NOVICE':1}
        self.S_features = ['s1_total_turns','s2_prev_expert_roles','s3_child_correct_answers']
        self.reward_label = 'rl_reward_scores'

        self.nA = len(self.A_map.keys())
        self.nS = len(self.S_features)
        

        self.child_id = child_id
        
        self.S_dict = pickle.load( open( "saved_rl/s_dict.p", "rb" ) )
        self.scaler = joblib.load('saved_rl/scaler.pkl') 

        
    def get_rl_state(self,features):
        i = '-'.join([str(i) for i in list(features)])
        print("rl state: {}".format(i))
        if not i in self.S_dict.keys():
            print("cannot find {} in s dict".format(i))
            self.S_dict.update({i:len(self.S_dict)})
        return self.S_dict[i]
        
    def get_state_features(self,state):
        def quadratic(arr):
            arr = arr.tolist()
            qual =list(map(lambda x: math.pow(x,2),arr))
            ilist =arr + qual + [arr[0]*arr[1]*arr[2], qual[0]*qual[1]*qual[2]]
            for each in itertools.permutations(arr,2):
                ilist += [math.pow(each[0],2)*each[1], math.pow(each[1],2)*each[0]]
            for each in itertools.combinations(arr,2):
                ilist += [each[0]*each[1], math.pow(each[0],2)*math.pow(each[1],2)]
            return ilist
            
        def generate_features(ar):
            return quadratic(ar)
        
        for i,v in self.S_dict.items():
            if state == v:
                return generate_features(self.scaler.transform([float(m) for m in i.split('-')]))
                break
        print("cannot find state features!!! {}".format(state))
        return "nan"
    
    def calculate_rl_rewards(self):
        return 0

class Estimator():
    """
    Value Function approximator. 
    """
    def __init__(self,env,reset= False):
        def retrive_regressor_models():
            if os.path.isfile(self.model_names[0]) and reset == False:
                model_0 = joblib.load(self.model_names[0])
                model_1 = joblib.load(self.model_names[1]) 
                return [model_0, model_1]
            else:
                model_0 = joblib.load('saved_rl/pretrained_regressor_model_0.pkl') 
                model_1 = joblib.load('saved_rl/pretrained_regressor_model_1.pkl') 
                return [model_0, model_1]
        self.model_names = ['saved_rl/'+env.child_id+'_regressor_model_0.pkl', 'saved_rl/'+env.child_id+'_regressor_model_1.pkl']
        self.models = retrive_regressor_models()         
            
    def save_updated_regressor_models(self):
        '''called at the end of each task mission'''
        for i in  range(len(self.model_names)):
            joblib.dump(self.models[i], self.model_names[i]) 
    
    def featurize_state(self, state):
        return env.get_state_features(state)
    
    def predict(self, s, a=None):
        """
        Makes value function predictions
        """
        features = self.featurize_state(s)
        if not a:
            return np.array([m.predict([features])[0] for m in self.models])
        else:
            return self.models[a].predict([features])[0]
    
    def update(self, s, a, y):
        """
        Updates the estimator parameters for a given state and action towards
        the target y.
        """
        features = self.featurize_state(s)
        print("features: {}. y: {}".format(features,y))
        
        self.models[a].partial_fit([features], [y])
    def get_weights(self):
        for i in self.models:
            print("model weights: {}".format(i.coef_ ))


class QModel():
    def initialize_q_learning(self, env, estimator, initial_features, reset=False, discount_factor=0.3, epsilon=0.5, epsilon_decay=1.0):
        ''''''
        self.env = env
        self.estimator = estimator
        self.discount_factor = discount_factor
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        try:
            self.i_episode =  pickle.load( open( "saved_rl/"+env.child_id+"_i_episode.pkl", "rb" ) ) if reset == False else 0
            self.action_history = pickle.load(open("saved_rl/"+env.child_id+"_action_history.pkl","rb")) if reset == False else dict()
        except:
            self.i_episode = 0
            self.action_history = dict()  ## load
        self.i_time = 0
        self.start_state = self.env.get_rl_state(initial_features)


        
    def save_rl(self):
        '''save after each timestamp'''
        pickle.dump(self.action_history, open("saved_rl/"+env.child_id+"_action_history.pkl","wb"))
        pickle.dump(self.i_episode, open("saved_rl/"+env.child_id+"_i_episode.pkl","wb"))
        self.estimator.save_updated_regressor_models()
        
    def q_new_episode(self,initial_features):
        '''called when the child finishes the first turn for a given task. then call q_get_action'''
        self.i_episode += 1

        # Reset the environment and pick the first action
        self.start_state = self.env.get_rl_state(initial_features)
        self.action_history.update({self.i_episode:{}})

        
    def q_get_action(self): 
        def make_epsilon_greedy_policy(estimator, epsilon, nA):
            print("~~cur epsilon: {}~~".format(epsilon))
            def policy_fn(observation):
                A = np.ones(nA, dtype=float) * epsilon / nA
                q_values = estimator.predict(observation)
                best_action = np.argmax(q_values)
                A[best_action] += (1.0 - epsilon)
                return A
            return policy_fn
        
        
        # The policy we're following
        policy = make_epsilon_greedy_policy(
            self.estimator, self.epsilon * self.epsilon_decay**self.i_episode, env.nA)
        next_action = None
        
        # Choose an action to take
        # If we're using SARSA we already decided in the previous step
        if next_action is None:
            action_probs = policy(self.start_state)
            self.action = np.random.choice(np.arange(len(action_probs)), p=action_probs)
        else:
            self.action = next_action
        
        return self.action
    
    def q_learning_evaluate(self,next_state):
            
        print("=======current episode: {}=========".format(self.i_episode))
        
        if isinstance(next_state, list):
            next_state = self.env.get_rl_state(next_state)

        self.i_time += 1

        # Take a step
        reward = env.calculate_rl_rewards()
        if np.isnan(reward):
            reward = 0


        # TD Update
        q_values_next = estimator.predict(next_state)
        print("q values next: {}".format(q_values_next))
        estimator.get_weights()

        # Use this code for Q-Learning
        # Q-Value TD Target
        td_target = reward + self.discount_factor * np.max(q_values_next)
        print("timeframe: {} cur state: {}, next state: {}, action: {}, reward: {}, max: {}, td target: {}".format(self.i_time, self.start_state, next_state, self.action, reward,np.max(q_values_next),td_target))
        if not self.i_episode in self.action_history.keys():
            self.action_history.update({self.i_episode:{}})
        self.action_history[self.i_episode].update({self.i_time: [self.start_state, next_state, self.action, reward,np.max(q_values_next),td_target]})
        
            
        # Use this code for SARSA TD Target for on policy-training:
            # next_action_probs = policy(next_state)
            # next_action = np.random.choice(np.arange(len(next_action_probs)), p=next_action_probs)             
            # td_target = reward + discount_factor * q_values_next[next_action]

        # Update the function approximator using our target
        estimator.update(self.start_state, self.action, td_target)

        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

        self.start_state = next_state
            

class Estimator():
    """
    Value Function approximator. 
    """
    def __init__(self,env):
        self.models = []
        for _ in range(env.nA):
            model = SGDRegressor(learning_rate="constant",penalty='l1')
            env.reset()
            model.partial_fit([self.featurize_state(env.next_episode())], [1])
            self.models.append(model)
            
            self.get_weights()
            
    def featurize_state(self, state):
        return env.get_state_features(state)
    
    def predict(self, s, a=None):
        """
        Makes value function predictions.
        
        Args:
            s: state to make a prediction for
            a: (Optional) action to make a prediction for
            
        Returns
            If an action a is given this returns a single number as the prediction.
            If no action is given this returns a vector or predictions for all actions
            in the environment where pred[i] is the prediction for action i.
            
        """
        features = self.featurize_state(s)
        if not a:
            return np.array([m.predict([features])[0] for m in self.models])
        else:
            return self.models[a].predict([features])[0]
    
    def update(self, s, a, y):
        """
        Updates the estimator parameters for a given state and action towards
        the target y.
        """
        features = self.featurize_state(s)
        print("features: {}. y: {}".format(features,y))
        
        self.models[a].partial_fit([features], [y])
    def get_weights(self):
        for i in self.models:
            print("model weights: {}".format(i.coef_ ))


class AgentModel():
    """
    This class implements a simple reinforcement learning agent that chooses what to do each round
    """

    def __init__(self):

        self.role_space= list(RobotRoles)
        
        self.role_history=[]
        #RobotRolesBehaviorsMap()


        # set up reinforcement learning here
        self.rl_env = iSpyEnvironment()


        all_states = self.rl_env.get_all_states()
        num_states = sum([len(i) for i in all_states[:,]])

        self.rl_agent = QLearningAgent(num_states=num_states,num_actions=len(list(RobotRoles)))

        self.current_action = ""

        self.estimator = Estimator()

    def get_next_robot_role(self):
        """
        Returns one of the actions from the ActionSpace
        """
        # get the next action using a rl model
        current_state = self.rl_env.observe_cur_state()
        next_action = self.rl_agent.get_action(current_state)
        self.current_action = next_action # update RL's current action here

        return RobotRoles(next_action)
       
    def onRewardsReceived(self,rewards):
        '''
        after robot performs the action, child's rewards are received from ChildStates
        let the agent learn based on RL model's new current state and received rewards.
        '''
        # perform action by updating rl's current state, getting reward and updating its q function
        prev_state, cur_state= self.rl_env.perform_action(self.current_action)
        # the agent learns and updates its q value
        if self.current_action != "":
            self.rl_agent.learn(prev_state,self.current_action,rewards,cur_state) 


    def make_epsilon_greedy_policy(estimator, epsilon, nA):
        """
        Creates an epsilon-greedy policy based on a given Q-function approximator and epsilon.

        Returns:
        A function that takes the observation as an argument and returns
        the probabilities for each action in the form of a numpy array of length nA.
    
        """
        def policy_fn(observation):
            A = np.ones(nA, dtype=float) * epsilon / nA
            q_values = estimator.predict(observation)
            best_action = np.argmax(q_values)
            A[best_action] += (1.0 - epsilon)
            return A
        return policy_fn

    def q_learning_step(self):
        if next_action is None:
            action_probs = policy(state)
            self.rl_action = np.random.choice(np.arange(len(action_probs)), p=action_probs)
        else:
            self.rl_action = next_action

         # Take a step
        next_state, reward, action = env.auto_step()

         # TD Update
        q_values_next = estimator.predict(next_state)
        print("q values next: {}".format(q_values_next))
        estimator.get_weights()
            
        # Use this code for Q-Learning
        # Q-Value TD Target
        td_target = reward + discount_factor * np.max(q_values_next)
        print("cur state: {}, next state: {}, action: {}, reward: {}, max: {}, td target: {}".format(state, next_state, action, reward,np.max(q_values_next),td_target))
            
        if np.isnan(reward):
            continue
            
        # Update the function approximator using our target
        print("state: {}".format(state))
        estimator.update(state, action, td_target)
            
        print("\rStep {} @ Episode {}/{} ({})".format(t, i_episode + 1, num_episodes, last_reward), end="")
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                
        state = next_state

    def setup_q_learning(env, estimator, num_episodes, discount_factor=0.3, epsilon=0.1, epsilon_decay=1.0):
    
            
        # The policy we're following
        self.rl_policy = make_epsilon_greedy_policy(
            estimator, epsilon * epsilon_decay**i_episode, env.nA)
        
        self.rl_action = None
        self.rl_next_action = None
        





