# ROS Message (child robot interaction)

# current turn: child or robot?
string whoseTurn

# robot's current role: expert or novice?
string robotRole

# robot's current behavior (action): question asking, feedback, hints? 
string robotBehavior


# robot's clicked object
string robotClickedObj

# clicked object (either robot's or child's) right or wrong (bool)
string clickedRightObject  

# clicked object name (either robot's or child's)
string clickedObjName

# current game task index: 
int8 gameTask 

# vocab word in the current game task
string taskVocab 

# number of retrieved object for the given task
int8 numFinishedObjects


# number of child's attempts of retrieving an object per game
int8 numChildAttemptsPerGame 

string childAnswerContent

# number of child's correct attempts (num of objs collected by the child) per game
int8 numChildCorrectAttemptsPerGame

# game state trigger (e.g., object clicked, object found, object pronounced) 
string gameStateTrigger

# how many questions robot has asked the child
int8 numRobotQuestionsAsked

# how many questions child answers the robot
int8 numRobotQuestionsAnswered 

# how many yes/no questions
int8 numRobotYNQuestion

# how many yes/no questions answered
int8 numRobotYNQuestionAnswered

# how many open ended questions
int8 numRobotOpenQuestion

# how many open ended questions answered
int8 numRobotOpenQuestionAnswered

# no tablet touch alert 
int8 numTouchAbsenceAlertPerTask

# current turn length
float32 current_turn_length

# current interaction FSM state
string currentInteractionState

# correctness of child's current attempt
bool childCurrAttemptCorrectness

# correctness of child's previous attempt
bool childPrevAttemptCorrectness

# whether the child pronoucnes the word to retrieve the object. 
bool objectWordPronounced

# number of child's attempts for the current task so far
int8 numChildAttemptsCurrTask

# number of child's correct attemps for the current task so far
int8 numChildCorrectAttemptsCurrTask

# number of robot's helping behaviors
int8 numRobotOfferHelp

# number of times child accepts robot's help
int8 numChildAcceptHelp

# number of times robot asks for help
int8 numRobotAskHelp

# number of times childs chooses to help
int8 numChildOfferHelp








