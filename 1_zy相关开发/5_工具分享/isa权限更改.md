$key = "D:\dtf\0_run_dkit_tool\id_rsa"

icacls.exe "$key"
takeown.exe /f "$key"
icacls.exe "$key" /inheritance:r
icacls.exe "$key" /remove:g "NT AUTHORITY\Authenticated Users"
icacls.exe "$key" /remove:g "Authenticated Users"
icacls.exe "$key" /remove:g "BUILTIN\Users"
icacls.exe "$key" /remove:g "Users"
icacls.exe "$key" /remove:g "Everyone"
icacls.exe "$key" /grant:r "$($env:USERNAME):(R)"
icacls.exe "$key"

ssh -i "$key" root@192.168.1.101