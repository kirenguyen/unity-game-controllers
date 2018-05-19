### Manual for iSpy Project (2017 Fall)
### 0. Ubuntu and ROS Installation
1. Install VMWare Fusion and Ubuntu 14.04
2. Install ROS Indigo following the instruction on [the website](http://wiki.ros.org/indigo) 
	* ROS is an important platform for us to do social robotics research here at Media Lab
	* Use Indigo version
3. Basic tutorials on how to use ROS
	* How to run roscore
	* How to suscribe and publish ros messages in Python and in terminal
	* How to build customized ROS messages
	* How rosbridge works

4. Create a short Python script that is able to publish a ROS message if you haven't done so before

5. add the following to your `./bashrc` for ROS IP
	* `export ROS_IP=$(/sbin/ip -o -4 addr list eth0 | awk '{print $4}' | cut -d/ -f1)`
	* or just use the sample bashrc script attached in this folder. copy and paste it to your own bashrc file and modify it accordingly

6. Download `git` to both Mac and Ubuntu using `sudo apt-get install git`

7. The default storage size of your ubuntu is 20GB. Use the following video to learn how to increase the size of your vmware partitions to 60GB. [video link](https://www.youtube.com/watch?v=99ImE9ZlX2s)
 
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
3. Become familiar with the iSpy infrastructure. Understand the structure of the code. If you have any questions, please consult with Huili or Sam
	* iSpy game in Unity
	* iSpy game controller in Python
	* Check the [iSpy architecture overview doc](https://github.com/mitmedialab/unity-game-controllers/blob/ispy-test/iSpyGameController/iSpyGameController%20Overview.md) to learn about its architecture

4. Learn how to use rosbridge and send ROS messages via ROS Bridge back and forth between Unity code and Python code
	* Download and install [rosbridge](http://wiki.ros.org/rosbridge_server) 

5. Install all dependencies for iSpy 
	* First install pip3 if necessary. `sudo apt install python3-pip`
	* Install all python dependencies in `unity-game-controllers/requirements.txt`
		* You may use `make init` to install python dependencies. Check this [makefile](https://github.com/mitmedialab/unity-game-controllers/blob/ispy-test/Makefile)
	* Use `sudo apt-get install python-pyaudio python3-pyaudio` to install pyaudio
	* May need to upgrade `pip` before installing the dependencies. use `pip3 install --upgrade pip`
	* Use `pip3` to isntall
	
6. Try to run iSpy game on your machine
	* run ```roscore```
	* run command ```roslaunch rosbridge_server rosbridge_websocket.launch```
	* run ```unity-game-controllers``` using ```./scripts/run_ispy.sh [participant id] [experimenter name] [session number]```
		* Example: `./scripts/run_ispy.sh p00 huili s01`
	* Install [Unity 2017.2.0f](https://unity3d.com/get-unity/download/archive)
	* Open Unity and run the game
		* the ROS ip address should match the ros ip address roscore is running on

7. Download the iSpy game to an Anroid tablet in PRG (optional)
	* Run the app on the tablet to see if everything works well


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

6. git status shown in the command prompt
	* if you didn't use the sample `bashrc` file in the folder, then do the following
	* add the following to your `./bashrc`
	* ```parse_git_branch() { git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/ (\1)/' } ```

	* ```export PS1="\n\[$(tput bold)\]\[$(tput setaf 5)\]➜ \[$(tput setaf 6)\]\w\[$(tput setaf 3)\]\$(parse_git_branch) \[$(tput sgr0)\]"```

7. `git push` fails to push a new commit due to a large file that has already been deleted
	* `git filter-branch --index-filter 'git rm -r --cached --ignore-unmatch <file/dir>' HEAD`
	* (more info can be found here)[https://stackoverflow.com/questions/19573031/cant-push-to-github-because-of-large-file-which-i-already-deleted]


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
	
### 6. Video Recording For Role-Switching Study
1. optional 
2. install `ffmpeg` 
	* for Ubuntu 14.04:  `sudo apt-add-repository ppa:mc3man/trusty-media`
	* for Ubuntu 16.04: `sudo apt-add-repository ppa:jonathonf/ffmpeg-3`
	* `sudo apt-get update`
	* `sudo apt-get install ffmpeg`

2. install fonts 
	* `sudo apt-get install ttf-dejavu`

### 7. Deploy audios to Tega
	* put all audios to tega phone's sd card.
	* create a different folder and put the audios under the following directory: `contentroot/robots/tega/01/speech`
	* the way to play those audio files is shown in `TegaBehaviors.py`
	* lipsync the audio files: `https://sites.google.com/site/personalrobotsgroupmit/r1d1/toolbox/dragonbot-lipsync`

	* download `HandShaker` in Mac's app store for file transfer on Android devices. This app is a better alternative for `AndroidFileTransfer`

### 8. JIBO 










	 