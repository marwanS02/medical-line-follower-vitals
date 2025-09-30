function doPost(e) {
  try {
    const ss = SpreadsheetApp.openById('SPREADSHEET_ID'); // <-- replace
    const sheet = ss.getSheetByName('Data') || ss.insertSheet('Data');
    const now = new Date();

    let body = {};
    if (e && e.postData && e.postData.contents) {
      try { body = JSON.parse(e.postData.contents); } catch (err) {}
    }

    const hr   = Number(body.hr)   || '';
    const spo2 = Number(body.spo2) || '';
    const dev  = body.deviceId || '';
    const ts   = body.ts || '';

    // Header row if empty
    if (sheet.getLastRow() === 0) {
      sheet.appendRow(['ServerTS', 'HR_bpm', 'SpO2_pct', 'DeviceID', 'PhoneTS', 'Note']);
    }

    sheet.appendRow([now, hr, spo2, dev, ts, body.note || '']);
    return ContentService.createTextOutput('OK').setMimeType(ContentService.MimeType.TEXT);
  } catch (err) {
    return ContentService.createTextOutput('ERR: ' + err)
      .setMimeType(ContentService.MimeType.TEXT);
  }
}
