import os
import json
import pickle
import subprocess

affdex_dir = "affdex-outputs"
#os.path.exists()
remaining_rosbags=""

if os.path.exists("remaining_rosbags.p"):
	remaining_rosbags = pickle.load( open( "remaining_rosbags.p", "rb" ) )
else:
	with open('study_data_map.json') as json_data:
		remaining_rosbags = json.load(json_data)

print(remaining_rosbags.keys())

next_participant = list(remaining_rosbags.keys())[0]
s1 = remaining_rosbags[next_participant]['s1']
s2 = remaining_rosbags[next_participant]['s2']

s1_rosbags = [i[0] for i in s1]
s2_rosbags = [i[0] for i in s2]
all_bags = [i+".bag" for i in s1_rosbags + s2_rosbags]

rosbags_dict = {i[0]+".bag":i[1]+".csv" for i in s1+s2}




import os
import httplib2

# pip install --upgrade google-api-python-client
from oauth2client.file import Storage
from apiclient.discovery import build
from oauth2client.client import OAuth2WebServerFlow

# Copy your credentials from the console
# https://console.developers.google.com. click credentials. create oauth2 ID
CLIENT_ID = '380833764831-853pp7su2msc7tk57erulgbdgrl63ea1.apps.googleusercontent.com'
CLIENT_SECRET = 'nUlM-TbacZD5ZxWyexWQlLv9'


OAUTH_SCOPE = 'https://www.googleapis.com/auth/drive'
REDIRECT_URI = 'urn:ietf:wg:oauth:2.0:oob'
OUT_PATH = os.path.join(os.path.dirname(__file__), 'rosbag_inputs')
CREDS_FILE = os.path.join(os.path.dirname(__file__), 'credentials.json')

print(OUT_PATH)
if not os.path.exists(OUT_PATH):
    os.makedirs(OUT_PATH)

storage = Storage(CREDS_FILE)
credentials = storage.get()

if credentials is None:
    # Run through the OAuth flow and retrieve credentials
    flow = OAuth2WebServerFlow(CLIENT_ID, CLIENT_SECRET, OAUTH_SCOPE, REDIRECT_URI)
    authorize_url = flow.step1_get_authorize_url()
    print('Go to the following link in your browser: ' + authorize_url)
    code = raw_input('Enter verification code: ').strip()
    #code ="4/AACrsLsy4yJ1wbQSaneHl2dlSGklb8n8GDAnSwYJJHUdTizNX5UtKFw"
    credentials = flow.step2_exchange(code)
    storage.put(credentials)

print("credential part is done!")
# Create an httplib2.Http object and authorize it with our credentials
http = httplib2.Http()
http = credentials.authorize(http)
drive_service = build('drive', 'v2', http=http)

#drive_service.setRootUrl("https://drive.google.com/drive/u/2/folders/1WxYaa5rvGB-x_BgaxrVvRzCm2JPbXRxo")

folder_id = "https://drive.google.com/drive/u/2/folders/1WxYaa5rvGB-x_BgaxrVvRzCm2JPbXRxo"
def list_files(service):
    page_token = None
    while True:
        param = {}
        if page_token:
            param['pageToken'] = page_token

        param['q']="'1WxYaa5rvGB-x_BgaxrVvRzCm2JPbXRxo' in parents"  # parent folder directory
        files = service.files().list(**param).execute()
        for item in files['items']:
            yield item
        page_token = files.get('nextPageToken')
        if not page_token:
            break

print("all bags")
print(all_bags)
print("for item in list files ....")
success = True
for item in list_files(drive_service):

    if item.get('title') in all_bags:
        outfile = os.path.join(OUT_PATH, '%s' % item['title'])
        print("outfile: {}".format(outfile))
        download_url = None
        print("after download url...")
        if 'exportLinks' in item and 'application/pdf' in item['exportLinks']:
            download_url = item['exportLinks']['application/pdf']
        elif 'downloadUrl' in item:
            download_url = item['downloadUrl']
        else:
            print('ERROR getting %s' % item.get('title'))
            print(item)
            print(dir(item))
            success = False
        print("download url: {}".format(download_url))
        if download_url:
            print("downloading %s" % item.get('title'))
            resp, content = drive_service._http.request(download_url)
            if resp.status == 200:
                if os.path.isfile(outfile):
                	msg="\"ERROR, %s already exist\"" % outfile
                	print(msg)
                	os.system("echo "+msg + " >> rosbag_downloading_error_report.txt")
                	success = False
                else:
                    with open(outfile, 'wb') as f:
                        f.write(content)
                    print("OK")
            else:
            	msg = "\"ERROR downloading %s\"" % item.get('title')
            	print(msg)
            	os.system("echo "+msg + " >> rosbag_downloading_error_report.txt")
            	success = False

    else:
    	#print("else")
    	#print(item.get('title'))
    	# msg = "\"ERROR downloading %s\"" % item.get('title')
    	# print(msg)
    	# os.system("echo "+msg + " >> rosbag_downloading_error_report.txt")
    	pass

if success == True:
	# delete the current participant
	try:
		del remaining_rosbags[next_participant]
		pickle.dump( remaining_rosbags , open( "remaining_rosbags.p", "wb" ))
	except:
		print("failed to delte participant: {}".format(next_participant))
		os.system("echo \"ERROR: failed to delete "+next_participant+">> rosbag_downloading_error_report.txt")
    



    


