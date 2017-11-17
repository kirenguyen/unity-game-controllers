### Manual for iSpy Project (2017 Fall)
### 0. Ubuntu and ROS Installation
1. Install VMWare Fusion and Ubuntu 14.04
2. Install ROS Indigo following the instruction on [the website](http://wiki.ros.org/indigo) 
	* ROS is an important platform for us to do social robotics research here at Media Lab
3. Basic tutorials on how to use ROS
	* How to run roscore
	* How to suscribe and publish ros messages in Python and in terminal
	* How to build customized ROS messages
	* How rosbridge works
4. Create a short Python script that is able to publish a ROS message
	* Take a screenshot and show it to me
 
### 1. CITI Training

1. Complete the CITI (IRB) training
2. Instructions are below:
	* You first need to install MIT certificate if you don’t have one yet. [Click here](https://ca.mit.edu/ca/)
	* Then go to [the website](http://coeus.mit.edu/coeus/citi/CitiLogin.jsp) 
	* Update your profile, then goto MainMenu. Under “MIT Courses”, click “Add a course”
	* In the orange “IRB courses” category:
		* Social & Behavioral Research Investigators
		* If it’s your first time taking this course, select “I have not previously..”
		* No
	* In the blue “RCR courses” category:
		* RCR for Engineers
	* Click submit, then you’ll see that new courses have been added to your MainMenu.
	* After you finish the course, forward me your certificate and copy Hae Won (our post-doc) in the email so that she could add you to our study protocol

### 2. iSpy Game

1. You should have access to all iSpy-related git repos 
	* [iSpy-game-unity](https://github.com/mitmedialab/iSpy-game-unity)
	* [unity-game-controllers](https://github.com/mitmedialab/unity-game-controllers)

2. Download them and install them (using ```git clone``` command)
	* download iSpy-game-unity to your mac
	* download unit-game-controllers to your Ubuntu.
		* location: ```/home/[YOUR NAME]/catkin_ws/src```
3. Become familiar with the iSpy infrastructure. Understand the structure of the code. If you have any questions, please consult with Huili, Mike or Sam
	* iSpy game in Unity
	* iSpy game controller in Python

4. Learn how to use rosbridge and send ROS messages via ROS Bridge back and forth between Unity code and Python code
	* Download and install [rosbridge](http://wiki.ros.org/rosbridge_server) 
	
5. Try to run iSpy game on your machine
	* run ```roscore```
	* run command ```roslaunch rosbridge_server rosbridge_websocket.launch```
	* run ```unity-game-controllers``` using ```python3 -m xxx```
	* Install [Unity 5.6.1](https://unity3d.com/get-unity/download/archive)
	* Open Unity and run the game
		* the ROS ip address should match the ros ip address roscore is running on

6. Download the iSpy game to an Anroid tablet in PRG
	* Run the app on the tablet to see if everything works well

7. Demo what you have done to Huili

### 3. Git commands
1. Learn to use the following commands
	* ```git checkout```
	* ```git add```
	* ```git commit```
	* ```git push```
	* ```git pull```
	* ```git stash```
	* ```git clone```

2. How to update your code to your local branch on github? Do the following.
	* ```git add -A``` to add all files in your project to your Amazon package
		* ```git add XXX.py``` add only ```XXX.py``` to your Amazon package
	* ```git commit - m "XX updated/implemented"``` to leave a receipt for your package and seal the package box
	* ```git push origin [your branch name``` to send your Amazon package to your customer. Specify ```origin [your branch name]``` if this is the first time you udpate your branch on github.
		* Next time, just use ```git push``` because you can have an "express order" for your clients and you have their "mailing address" information already.

3. Use ```git pull``` to update your local git repo before working on your work.
	* Others may have updated the code when you are sleeping. 

4. ```git branch -a``` 
	* list out all branches in the current repo

5. ```git branch``` 
	* print the branch you are in 

### 4. How iSpy Game, Unity Controller and Robot communicate with each other via ROS?

1. They communicate with each other via ROSBridge
1. For the iSpy unity game, please check file `Assets/Scripts/Utilities/RosbridgeWebSocketClient.cs` and the function `void HandleOnMessage (object sender, MessageEventArgs e)` in it. This function receives the ROS messages that are sent from the controller

2. look at file `GameController.cs` file for `public void OnTaskMessageReceived(Dictionary<string, object> msgParams)`. this function basically recieves the game mission content from the unity controller and then display the content in the unity game. 

3. Notice that `public void OnTaskMessageReceived(Dictionary<string, object> msgParams)` is called in `Assets/Scripts/Utilities/RosbridgeWebSocketClient.cs` by a function called `public void HandleClientSocketReceivedMsgEvent(int cmd, object msgParams)`.

4. Last, when you want to virtually move the scene, zoom in zoom out, click an an object, please check the file `UITouchController.cs` that contains all UITouch functions



### 5. Deployung and Debugging Apps on a Tablet
1. connect your tablet to your laptop. 
2. Make sure `debugging mode` is on on your tablet (in settings)
3. `adb tcpip 5555` to start `adb` debugging 
4. `adb devices` to list all connected devices
5. install Android Studio if you haven't
6. Build your app apk in Unity
7. Open your apk in Android Studio
	* choose `profile or debug APK`
8. Deploy your apk to the tablet and debug in Android Studio
9. `E/Unity: UnauthorizedAccessException: Access to the path`. if this issue exists, then do the following:
	* Make sure in the Unity setting (`Edit/Project Settings/Player`): change th permission from `internal` to `external (sd card)`
	










	 