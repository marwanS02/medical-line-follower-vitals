# Simple Cloud Data Store (Google Apps Script + Google Sheets)

## Quick Setup
1) Create a Google Sheet and copy its **Spreadsheet ID** (the long ID in the URL).
2) In https://script.google.com/ create a new project, paste `apps-script/Code.gs`.
3) Replace `SPREADSHEET_ID` in the code with your Sheet ID.
4) **Deploy → New deployment → Web app**
   - Execute as: *Me*
   - Who has access: *Anyone with the link* (or your choice)
5) Copy the Web App URL and put it into your App Inventor “Web” component.
   - Method: POST
   - Content-Type: application/json
   - Body: `{"hr":<number>,"spo2":<number>,"deviceId":"linebot-01","ts":"<phone time>"}`

**Sheet columns** will be: Timestamp (server), HR, SpO2, DeviceID, PhoneTS, Note
