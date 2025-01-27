$pio_env1 = "t4"
$pio_env2 = "t41"


function clean_workspace {
    $junk = @(
        ".\.pio\"
        ".\.vscode\" 
        ".\.gitignore"
    )

    foreach($item in $junk) {
        Remove-Item -Recurse -Force $item -ErrorAction SilentlyContinue
    }
}

function upload {

    Clear-Host
    $device = Read-Host -Prompt "Upload to Teensy4.0 (1) or Teensy4.1 (2) [1/2] >"
        
    if ($device -eq "1") {
        pio run -t upload -e $pio_env1
    } elseif ($device -eq "2") {
        pio run -t upload -e $pio_env2
    } else {
        Write-Host "Invalid device selection" -fore red
        init
    }

}

function build {
    Clear-Host
    Write-Host "Checking 4 bugs..." -fore green
    pio run
}
function init {
    
    $up = Read-Host -Prompt "Just Build (b) or Upload to Device (u) [b/u] >"
    if ($up -eq "u") {
        upload
    } elseif ($up -eq "b") {
        build
    } else {
        Write-Host "Invalid option" -fore red
    }
}

clean_workspace
init