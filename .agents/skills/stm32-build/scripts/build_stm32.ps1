[CmdletBinding()]
param(
    [string]$ProjectRoot = (Get-Location).Path,
    [string]$Preset,
    [string]$BuildDirectory,
    [string]$Configuration,
    [switch]$ProbeOnly
)

$ErrorActionPreference = "Stop"

function Stop-SkillBuild {
    param([Parameter(Mandatory = $true)][string]$Message)

    [Console]::Error.WriteLine("ERROR: " + $Message)
    exit 2
}

function Get-JsonDocument {
    param([Parameter(Mandatory = $true)][string]$Path)

    try {
        return Get-Content -Raw -LiteralPath $Path | ConvertFrom-Json
    } catch {
        Stop-SkillBuild "Invalid JSON in $Path`: $($_.Exception.Message)"
    }
}

function Add-PresetDocument {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][ValidateSet("Project", "User")][string]$Origin,
        [Parameter(Mandatory = $true)]$Documents,
        [Parameter(Mandatory = $true)]$Visited
    )

    $fullPath = [System.IO.Path]::GetFullPath($Path)
    if (-not (Test-Path -LiteralPath $fullPath -PathType Leaf)) {
        return
    }

    if (-not $Visited.Add($fullPath)) {
        return
    }

    $data = Get-JsonDocument $fullPath
    $includeProperty = $data.PSObject.Properties["include"]
    if ($includeProperty) {
        foreach ($include in @($includeProperty.Value)) {
            $includePath = [string]$include
            if (-not [System.IO.Path]::IsPathRooted($includePath)) {
                $includePath = Join-Path ([System.IO.Path]::GetDirectoryName($fullPath)) $includePath
            }
            Add-PresetDocument $includePath $Origin $Documents $Visited
        }
    }

    $Documents.Add([pscustomobject]@{
        Path = $fullPath
        Origin = $Origin
        Data = $data
    })
}

function Get-ObjectProperty {
    param(
        [Parameter(Mandatory = $true)]$Object,
        [Parameter(Mandatory = $true)][string]$Name
    )

    $property = $Object.PSObject.Properties[$Name]
    if ($property) {
        return $property.Value
    }
    return $null
}

function Get-InheritedPresetValue {
    param(
        [Parameter(Mandatory = $true)]$PresetMap,
        [Parameter(Mandatory = $true)][string]$PresetName,
        [Parameter(Mandatory = $true)][string]$PropertyName,
        $VisitedNames = $null
    )

    if (-not $PresetMap.ContainsKey($PresetName)) {
        return $null
    }

    if (-not $VisitedNames) {
        $VisitedNames = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
    }
    if (-not $VisitedNames.Add($PresetName)) {
        Stop-SkillBuild "Cyclic preset inheritance detected at '$PresetName'."
    }

    $presetObject = $PresetMap[$PresetName]
    $directValue = Get-ObjectProperty $presetObject $PropertyName
    if ($null -ne $directValue) {
        [void]$VisitedNames.Remove($PresetName)
        return $directValue
    }

    foreach ($parentName in @(Get-ObjectProperty $presetObject "inherits")) {
        if ($null -eq $parentName) {
            continue
        }
        $inheritedValue = Get-InheritedPresetValue $PresetMap ([string]$parentName) $PropertyName $VisitedNames
        if ($null -ne $inheritedValue) {
            [void]$VisitedNames.Remove($PresetName)
            return $inheritedValue
        }
    }

    [void]$VisitedNames.Remove($PresetName)
    return $null
}

function Expand-BinaryDirectory {
    param(
        [Parameter(Mandatory = $true)][string]$Value,
        [Parameter(Mandatory = $true)][string]$SourceDirectory,
        [Parameter(Mandatory = $true)][string]$PresetName
    )

    $sourceParent = [System.IO.Path]::GetDirectoryName($SourceDirectory)
    $sourceName = [System.IO.Path]::GetFileName($SourceDirectory.TrimEnd('\', '/'))
    $expanded = $Value
    $expanded = $expanded.Replace('${sourceDir}', $SourceDirectory)
    $expanded = $expanded.Replace('${sourceParentDir}', $sourceParent)
    $expanded = $expanded.Replace('${sourceDirName}', $sourceName)
    $expanded = $expanded.Replace('${presetName}', $PresetName)
    $expanded = [regex]::Replace($expanded, '\$(?:p)?env\{([^}]+)\}', {
        param($match)
        return [Environment]::GetEnvironmentVariable($match.Groups[1].Value)
    })

    if ($expanded -match '\$\{[^}]+\}|\$(?:p)?env\{[^}]+\}') {
        Stop-SkillBuild "Unsupported macro in preset binaryDir '$Value'. Pass -BuildDirectory explicitly."
    }

    if (-not [System.IO.Path]::IsPathRooted($expanded)) {
        $expanded = Join-Path $SourceDirectory $expanded
    }
    return [System.IO.Path]::GetFullPath($expanded.Replace('/', '\'))
}

function Get-RequiredMatch {
    param(
        [Parameter(Mandatory = $true)][string]$Content,
        [Parameter(Mandatory = $true)][string]$Pattern,
        [Parameter(Mandatory = $true)][string]$Description
    )

    $match = [regex]::Match($Content, $Pattern)
    if (-not $match.Success) {
        Stop-SkillBuild "Could not determine $Description from the selected CMake build cache."
    }
    return $match.Groups[1].Value.Trim()
}

function Get-NormalizedPath {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$BasePath
    )

    $normalized = $Path.Replace('/', '\')
    if (-not [System.IO.Path]::IsPathRooted($normalized)) {
        $normalized = Join-Path $BasePath $normalized
    }
    return [System.IO.Path]::GetFullPath($normalized)
}

function Get-Stm32BundleRoot {
    param([Parameter(Mandatory = $true)][string]$ToolPath)

    $normalized = [System.IO.Path]::GetFullPath($ToolPath).Replace('/', '\')
    $marker = "\stm32cube\bundles\"
    $index = $normalized.IndexOf($marker, [System.StringComparison]::OrdinalIgnoreCase)
    if ($index -lt 0) {
        Stop-SkillBuild "CMake is not from an STM32Cube VS Code bundle: $normalized"
    }
    return $normalized.Substring(0, $index + $marker.Length).TrimEnd('\')
}

function Assert-BundledTool {
    param(
        [Parameter(Mandatory = $true)][string]$ToolPath,
        [Parameter(Mandatory = $true)][string]$BundleRoot,
        [Parameter(Mandatory = $true)][string]$Label
    )

    if (-not (Test-Path -LiteralPath $ToolPath -PathType Leaf)) {
        Stop-SkillBuild "$Label does not exist: $ToolPath"
    }

    $prefix = [System.IO.Path]::GetFullPath($BundleRoot).TrimEnd('\') + '\'
    if (-not $ToolPath.StartsWith($prefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        Stop-SkillBuild "$Label is not from the selected STM32Cube bundle: $ToolPath"
    }
}

function Find-BuildCaches {
    param([Parameter(Mandatory = $true)][string]$Root)

    return @(Get-ChildItem -LiteralPath $Root -Recurse -File -Filter "CMakeCache.txt" -ErrorAction SilentlyContinue |
        Where-Object { $_.FullName -notmatch '[\\/]\.agents[\\/]|[\\/]\.git[\\/]' } |
        Sort-Object LastWriteTime -Descending)
}

$projectRootPath = [System.IO.Path]::GetFullPath($ProjectRoot)
if (-not (Test-Path -LiteralPath (Join-Path $projectRootPath "CMakeLists.txt") -PathType Leaf)) {
    Stop-SkillBuild "CMakeLists.txt was not found in project root: $projectRootPath"
}

$documents = New-Object 'System.Collections.Generic.List[object]'
$visitedFiles = New-Object 'System.Collections.Generic.HashSet[string]' ([System.StringComparer]::OrdinalIgnoreCase)
Add-PresetDocument (Join-Path $projectRootPath "CMakePresets.json") "Project" $documents $visitedFiles
Add-PresetDocument (Join-Path $projectRootPath "CMakeUserPresets.json") "User" $documents $visitedFiles

$configurePresets = @{}
$buildPresets = @{}
$userBuildNames = New-Object 'System.Collections.Generic.List[string]'
$projectBuildNames = New-Object 'System.Collections.Generic.List[string]'

foreach ($document in $documents) {
    foreach ($configurePreset in @($document.Data.configurePresets)) {
        if ($null -ne $configurePreset -and $configurePreset.name) {
            $configurePresets[[string]$configurePreset.name] = $configurePreset
        }
    }

    foreach ($buildPreset in @($document.Data.buildPresets)) {
        if ($null -eq $buildPreset -or -not $buildPreset.name) {
            continue
        }
        $name = [string]$buildPreset.name
        $buildPresets[$name] = $buildPreset
        $hidden = Get-ObjectProperty $buildPreset "hidden"
        if ($hidden -eq $true) {
            continue
        }
        if ($document.Origin -eq "User") {
            $targetList = $userBuildNames
        } else {
            $targetList = $projectBuildNames
        }
        if (-not $targetList.Contains($name)) {
            $targetList.Add($name)
        }
    }
}

$selectedPreset = $null
$selectionSource = $null
$buildDirectoryPath = $null

if ($BuildDirectory) {
    $buildDirectoryPath = Get-NormalizedPath $BuildDirectory $projectRootPath
    $selectionSource = "explicit build directory"
} else {
    if ($Preset) {
        $selectedPreset = $Preset
        $selectionSource = "explicit preset"
    } elseif ($env:STM32_BUILD_PRESET) {
        $selectedPreset = $env:STM32_BUILD_PRESET
        $selectionSource = "STM32_BUILD_PRESET"
    } else {
        $settingsPath = Join-Path $projectRootPath ".vscode\settings.json"
        if (Test-Path -LiteralPath $settingsPath -PathType Leaf) {
            $settingsRaw = Get-Content -Raw -LiteralPath $settingsPath
            try {
                $settings = $settingsRaw | ConvertFrom-Json
                $settingsPreset = Get-ObjectProperty $settings "cmake.buildPreset"
            } catch {
                $settingsMatch = [regex]::Match($settingsRaw, '"cmake\.buildPreset"\s*:\s*"([^"]+)"')
                $settingsPreset = if ($settingsMatch.Success) { $settingsMatch.Groups[1].Value } else { $null }
            }
            if ($settingsPreset -and $settingsPreset -ne "auto" -and $settingsPreset -notmatch '^\$\{') {
                $selectedPreset = [string]$settingsPreset
                $selectionSource = "VS Code cmake.buildPreset"
            }
        }
    }

    if (-not $selectedPreset -and $userBuildNames.Count -gt 0) {
        $selectedPreset = $userBuildNames[0]
        $selectionSource = "CMakeUserPresets.json"
    }
    if (-not $selectedPreset -and $projectBuildNames.Count -gt 0) {
        $selectedPreset = $projectBuildNames[0]
        $selectionSource = "CMakePresets.json"
    }

    if ($selectedPreset) {
        if (-not $buildPresets.ContainsKey($selectedPreset)) {
            $available = @($buildPresets.Keys | Sort-Object) -join ", "
            Stop-SkillBuild "Build preset '$selectedPreset' was not found. Available build presets: $available"
        }
        $configurePresetName = Get-InheritedPresetValue $buildPresets $selectedPreset "configurePreset"
        if (-not $configurePresetName) {
            Stop-SkillBuild "Build preset '$selectedPreset' does not resolve to a configurePreset."
        }
        $binaryDirectory = Get-InheritedPresetValue $configurePresets ([string]$configurePresetName) "binaryDir"
        if (-not $binaryDirectory) {
            Stop-SkillBuild "Configure preset '$configurePresetName' does not define binaryDir. Pass -BuildDirectory explicitly."
        }
        $buildDirectoryPath = Expand-BinaryDirectory ([string]$binaryDirectory) $projectRootPath ([string]$configurePresetName)
    }
}

if (-not $buildDirectoryPath) {
    $availableCaches = Find-BuildCaches $projectRootPath
    if ($availableCaches.Count -eq 0) {
        Stop-SkillBuild "No build preset or existing CMakeCache.txt was found. Configure the project with the STM32Cube VS Code extension first."
    }
    $buildDirectoryPath = $availableCaches[0].Directory.FullName
    $selectionSource = "most recent CMake cache"
}

$cachePath = Join-Path $buildDirectoryPath "CMakeCache.txt"
if (-not (Test-Path -LiteralPath $cachePath -PathType Leaf)) {
    Stop-SkillBuild "The selected configuration has no CMake cache: $cachePath. Configure that preset with the STM32Cube VS Code extension first."
}

$cache = Get-Content -Raw -LiteralPath $cachePath
$cmakeRaw = Get-RequiredMatch $cache '(?m)^# It was generated by CMake: *([^\r\n]+?cmake\.exe)[\r ]*$' "CMake executable"
$buildToolRaw = Get-RequiredMatch $cache '(?m)^CMAKE_MAKE_PROGRAM:[^=]*=([^\r\n]+)[\r ]*$' "build tool"
$cmakePath = Get-NormalizedPath $cmakeRaw $projectRootPath
$buildToolPath = Get-NormalizedPath $buildToolRaw $projectRootPath
$bundleRoot = Get-Stm32BundleRoot $cmakePath
Assert-BundledTool $cmakePath $bundleRoot "CMake"
Assert-BundledTool $buildToolPath $bundleRoot "Build tool"

$compilerInfo = Get-ChildItem -LiteralPath (Join-Path $buildDirectoryPath "CMakeFiles") -Recurse -File -Filter "CMakeCCompiler.cmake" |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1
if (-not $compilerInfo) {
    Stop-SkillBuild "CMakeCCompiler.cmake was not found under $buildDirectoryPath."
}

$compilerData = Get-Content -Raw -LiteralPath $compilerInfo.FullName
$compilerRaw = Get-RequiredMatch $compilerData '(?m)^set\(CMAKE_C_COMPILER "([^"]+)"\)' "C compiler"
$compilerPath = Get-NormalizedPath $compilerRaw $projectRootPath
Assert-BundledTool $compilerPath $bundleRoot "C compiler"

$toolchainMatch = [regex]::Match($cache, '(?m)^CMAKE_TOOLCHAIN_FILE:[^=]*=([^\r\n]+)[\r ]*$')
$toolchainPath = if ($toolchainMatch.Success) {
    Get-NormalizedPath $toolchainMatch.Groups[1].Value.Trim() $projectRootPath
} else {
    "Not recorded"
}

Write-Output "Project: $projectRootPath"
Write-Output "Selection source: $selectionSource"
Write-Output "Build preset: $(if ($selectedPreset) { $selectedPreset } else { 'None' })"
Write-Output "Build directory: $buildDirectoryPath"
Write-Output "CMake: $cmakePath"
Write-Output "Build tool: $buildToolPath"
Write-Output "C compiler: $compilerPath"
Write-Output "Toolchain file: $toolchainPath"

if ($ProbeOnly) {
    Write-Output "Toolchain probe completed; build was not run."
    exit 0
}

Push-Location $projectRootPath
try {
    if ($selectedPreset) {
        & $cmakePath --build --preset $selectedPreset
    } else {
        $arguments = @("--build", $buildDirectoryPath)
        if ($Configuration) {
            $arguments += @("--config", $Configuration)
        }
        & $cmakePath @arguments
    }
    $buildExitCode = $LASTEXITCODE
} finally {
    Pop-Location
}

if ($buildExitCode -ne 0) {
    [Console]::Error.WriteLine("ERROR: STM32 build failed with exit code $buildExitCode.")
    exit $buildExitCode
}

Write-Output "STM32 build completed successfully."
exit 0
