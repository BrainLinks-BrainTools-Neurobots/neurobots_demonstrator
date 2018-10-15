workingdirectory = "C:\KRC\Roboter\Log\MatData"
days = 10

Set fso = CreateObject("Scripting.FileSystemObject")
Set directory = fso.GetFolder(workingdirectory)
today = Date()
DeleteInFolder(directory)

Sub DeleteInFolder(directory)
	Set files = directory.Files
	For Each file In files
		If file.DateLastModified < (today - days) Then
			file.Delete
		End If
	Next
End Sub
