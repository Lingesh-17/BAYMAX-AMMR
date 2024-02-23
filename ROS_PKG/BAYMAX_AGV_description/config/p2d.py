import gspread

Sheet_credential = gspread.service_account("/home/lingesh/Downloads/p2d-publisher-b14b126148ae.json")
 
# Open Spreadsheet by URL
spreadsheet = Sheet_credential.open_by_url('https://docs.google.com/spreadsheets/d/1e_ajOkJsJ4x8ux_aHIie8HpfP4Bxd_FZRH__Pd-4nxM/edit#gid=0')
 
# Open Spreadsheet by key
spreadsheet = Sheet_credential.open_by_key('1e_ajOkJsJ4x8ux_aHIie8HpfP4Bxd_FZRH__Pd-4nxM')
print(spreadsheet.title)


# to print worksheet name using sheet id
worksheet = spreadsheet.get_worksheet(0)
 
# to print worksheet name using sheet name

print(worksheet)

worksheet.update('A2', [["BED1", "Patient_name","Amputee","Need Food" ]])
