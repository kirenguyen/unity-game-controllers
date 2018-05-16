from apiclient.discovery import build
from httplib2 import Http
from oauth2client import file, client, tools
import io
from apiclient.http import MediaIoBaseDownload

SCOPES = ["https://www.googleapis.com/auth/drive",
          "https://www.googleapis.com/auth/drive.readonly",
          "https://www.googleapis.com/auth/drive.file"]



store = file.Storage('credentials.json')
creds = store.get()
if not creds or creds.invalid:
    flow = client.flow_from_clientsecrets('client_secret.json', SCOPES)
    creds = tools.run_flow(flow, store)
drive_service = build('drive', 'v3', http=creds.authorize(Http()))


file_id = '1vk2zOWSpgrbEplQwr-rt-D2t913AqXTl'
request = drive_service.files().get_media(fileId=file_id)
fh = io.BytesIO()
downloader = MediaIoBaseDownload(fh, request)
done = False
while done is False:
    status, done = downloader.next_chunk()
    print ("Download %d%%." % int(status.progress() * 100))
