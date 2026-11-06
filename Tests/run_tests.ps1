$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    New-Item -ItemType Directory -Path 'build' -Force | Out-Null
    & gcc -std=c99 -O2 -Wall -Wextra -Werror -I../Core/Inc test_base_control.c ../Core/Src/base_control.c -o build/test_base_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Controller test compilation failed.' }
    & .\build\test_base_control.exe
    if ($LASTEXITCODE -ne 0) { throw 'Controller tests failed.' }
    & gcc -std=c99 -O2 -Wall -Wextra -Werror -Istubs -I../Core/Inc test_base_app.c ../Core/Src/base_control.c -o build/test_base_app.exe
    if ($LASTEXITCODE -ne 0) { throw 'Adapter test compilation failed.' }
    & .\build\test_base_app.exe
    if ($LASTEXITCODE -ne 0) { throw 'Adapter tests failed.' }
} finally {
    Pop-Location
}
