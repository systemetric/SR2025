param (
	[string]$driveLetter
)

if ( $driveLetter -eq "")
{
	Write-Output "Usage: update_usb <drive letter>"
}
else 
{
	Copy-Item -Path .\*.* -Destination ${driveLetter}:\
	Write-Output "Success."
}