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
bool clickedRightObject  

# clicked object name (either robot's or child's)
string clickedObjName

# current game task index: 
int8 gameTask 

# vocab word in the current game task
string taskVocab 

# number of retrieved object for the given task
int8 numFinishedObjects


# number of child's attempts of retrieving an object
int8 numChildAttempts 

string childAnswerContent

# number of child's correct attempts (num of objs collected by the child)
int8 numChildCorrectAttempts 

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
int8 numTouchAbsenceAlert

# current turn length
float32 current_turn_length






