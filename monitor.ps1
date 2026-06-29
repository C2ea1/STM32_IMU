$port = New-Object System.IO.Ports.SerialPort "COM9", 115200, "None", 8, "One"
$port.DtrEnable = $false
$port.RtsEnable = $false
try {
    $port.Open()
    Write-Host "--- Serial Monitor Opened on COM4 (Press Ctrl+C to exit) ---"
    while ($port.IsOpen) {
        if ($port.BytesToRead -gt 0) {
            $data = $port.ReadExisting()
            Write-Host -NoNewline $data
        }
        Start-Sleep -Milliseconds 50
    }
}
catch {
    Write-Host "Error opening port: $_"
}
finally {
    if ($port.IsOpen) {
        $port.Close()
    }
    Write-Host "`n--- Serial Monitor Closed ---"
}
