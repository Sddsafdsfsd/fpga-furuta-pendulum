# build_pd.ps1 -- 从本包源码综合出 .fs 比特流
#
# 用法（在包根目录或任意位置执行均可）：
#   powershell -ExecutionPolicy Bypass -File .\工具\build_pd.ps1
#   powershell -ExecutionPolicy Bypass -File .\工具\build_pd.ps1 -Stage 5 -I2cDiv 1000 -OutName f1_balance.fs
#
# 本脚本自带工程：每次运行会在包内创建 build\ 目录，把 源码\ 复制进去，
# 现场生成 f1_attitude.gprj 与 build.tcl，再用 Gowin 的 gw_sh 综合。
# 不需要事先用 Gowin IDE 建工程，也不会改动 源码\ 里的任何文件。
#
# 默认参数对应交付固件 User Code 0xE931。

param(
    [int]$Stage = 5,
    [int]$I2cDiv = 1000,
    [string]$OutName = 'f1_balance.fs',
    [string]$GowinBin = ''          # gw_sh.exe 所在目录；留空则自动探测
)

$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$fwDir = Join-Path $root '固件'

# 源文件目录：支持两种布局
#   扁平布局  源码\*.v  + 源码\*.cst        （交付包：锦辉）
#   分离布局  rtl\*.v   + constr\*.cst      （开源仓库：GitHub 版）
$flat = Join-Path $root '源码'
$rtl = Join-Path $root 'rtl'
$constr = Join-Path $root 'constr'
if (Test-Path $flat) {
    $srcDirs = @($flat)
    $layout = 'flat'
} elseif ((Test-Path $rtl) -and (Test-Path $constr)) {
    $srcDirs = @($rtl, $constr)
    $layout = 'split'
} else {
    throw "找不到源码目录。期望 '源码\' 或 'rtl\'+'constr\'，实际在: $root"
}

# Gowin 的 Tcl 控制台不处理非 ASCII 路径（中文目录会被吃成 ???），
# 因此在系统临时目录里建工程，综合完再把结果复制回包内。
$buildDir = Join-Path $env:TEMP ('f1_build_' + [guid]::NewGuid().ToString('N').Substring(0, 8))

Write-Host "布局  : $layout" -ForegroundColor DarkGray

# ---------- 1) 定位 Gowin ----------
function Find-GowinBin {
    param([string]$Hint)
    if ($Hint) {
        $p = Join-Path $Hint 'gw_sh.exe'
        if (Test-Path $p) { return $Hint }
        throw "指定的 GowinBin 下没有 gw_sh.exe: $Hint"
    }
    $roots = @(
        'C:\Gowin', 'D:\Gowin', 'E:\Gowin',
        'C:\Program Files\Gowin', 'C:\Program Files (x86)\Gowin',
        'D:\Program Files\Gowin', 'E:\Installation_package\Gowin'
    )
    $hits = @()
    foreach ($r in $roots) {
        if (-not (Test-Path $r)) { continue }
        $hits += Get-ChildItem $r -Recurse -Filter 'gw_sh.exe' -File -ErrorAction SilentlyContinue |
                 Select-Object -ExpandProperty DirectoryName
    }
    if (-not $hits) { throw "未找到 Gowin EDA。请用 -GowinBin 指定 gw_sh.exe 所在目录。" }
    return ($hits | Sort-Object -Descending | Select-Object -First 1)
}

$bin = Find-GowinBin -Hint $GowinBin
$gwsh = Join-Path $bin 'gw_sh.exe'
Write-Host "Gowin : $gwsh" -ForegroundColor Cyan

# ---------- 2) 准备工程目录（临时目录，纯 ASCII 路径）----------
New-Item -ItemType Directory -Force -Path $buildDir | Out-Null
$bSrc = Join-Path $buildDir 'src'
New-Item -ItemType Directory -Force -Path $bSrc | Out-Null
foreach ($d in $srcDirs) { Copy-Item (Join-Path $d '*') $bSrc -Recurse -Force }
Write-Host "工作在: $buildDir" -ForegroundColor DarkGray

# 注入 STAGE 与 OS_CLKDIV（只改临时副本）
$topV = Join-Path $bSrc 'top.v'
$tv = [System.IO.File]::ReadAllText($topV)

$m = [regex]::Match($tv, 'parameter\s+integer\s+STAGE\s*=\s*\d+')
if (-not $m.Success) { throw "top.v 里找不到 'parameter integer STAGE'" }
$tv = $tv.Replace($m.Value, "parameter integer STAGE          = $Stage")

$m = [regex]::Match($tv, 'parameter\s+integer\s+OS_CLKDIV\s*=\s*\d+')
if (-not $m.Success) { throw "top.v 里找不到 'parameter integer OS_CLKDIV'" }
$tv = $tv.Replace($m.Value, "parameter integer OS_CLKDIV    = $I2cDiv")

[System.IO.File]::WriteAllText($topV, $tv)
Write-Host "STAGE    -> $Stage" -ForegroundColor Cyan
Write-Host "OS_CLKDIV-> $I2cDiv" -ForegroundColor Cyan

$hz = 50000000.0 / (205.0 * $I2cDiv)
Write-Host ("           transaction {0:N0} clk = {1:N2} ms -> {2:N1} Hz ; SCL {3:N0} Hz" -f `
    (205 * $I2cDiv), (205 * $I2cDiv / 50000.0), $hz, (50000000.0 / (4.0 * $I2cDiv))) -ForegroundColor Cyan

# ---------- 3) 生成 .gprj ----------
$verilog = @('top.v', 'adc_3pa1030.v', 'dac_3pd5651.v', 'dac_square.v', 'ramp_gen.v',
             'pid_ctrl.v', 'pid_s16.v', 'dual_loop_ctrl.v', 'pd_ctrl.v',
             'quad_decoder.v', 'enc_single.v', 'enc_sim.v', 'tb6612_drv.v',
             'servo_pwm.v', 'uart_tx.v', 'uart_rx.v', 'telemetry.v', 'cmd_parser.v',
             'i2c_master.v', 'as5600_read.v', 'as5600_i2c.v')
$files = @()
foreach ($v in $verilog) {
    if (Test-Path (Join-Path $bSrc $v)) { $files += "        <File path=`"src/$v`" type=`"file.verilog`" enable=`"1`"/>" }
}
foreach ($c in @(@('f1_attitude.cst', 'file.cst'), @('f1_attitude.sdc', 'file.sdc'))) {
    if (Test-Path (Join-Path $bSrc $c[0])) { $files += "        <File path=`"src/$($c[0])`" type=`"$($c[1])`" enable=`"1`"/>" }
}

$gprj = @"
<?xml version="1" encoding="UTF-8"?>
<!DOCTYPE gowin-fpga-project>
<Project>
    <Template>FPGA</Template>
    <Version>5</Version>
    <Device name="GW1N-9" pn="GW1N-UV9EQ144C6/I5">gw1n9c-024</Device>
    <FileList>
$($files -join "`r`n")
    </FileList>
</Project>
"@
$gprjPath = Join-Path $buildDir 'f1_attitude.gprj'
[System.IO.File]::WriteAllText($gprjPath, $gprj)
Write-Host "gprj  : $gprjPath  ($($files.Count) 个文件)" -ForegroundColor Cyan

# ---------- 4) 生成 build.tcl 并综合 ----------
$tclPath = Join-Path $buildDir 'build.tcl'
$gp = $gprjPath -replace '\\', '/'
$tcl = @"
open_project {$gp}
set_option -top_module top
set_option -use_sspi_as_gpio 1
set_option -use_mspi_as_gpio 1
run all
"@
[System.IO.File]::WriteAllText($tclPath, $tcl)

$log = Join-Path $buildDir ("build_" + ($OutName -replace '\.fs$', '') + ".log")
Write-Host "开始综合（Gowin 需要几分钟）……" -ForegroundColor Yellow

# gw_sh 会把进度写到 stderr，必须合并捕获，否则 PowerShell 会当成异常
$prevEap = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
& $gwsh $tclPath 2>&1 | Out-File -FilePath $log -Encoding utf8
$ErrorActionPreference = $prevEap

$fs = Join-Path $buildDir 'impl\pnr\f1_attitude.fs'
if (-not (Test-Path $fs)) {
    Write-Host "综合失败。日志已复制到包内: build_failed.log" -ForegroundColor Red
    if (Test-Path $log) { Copy-Item $log (Join-Path $root 'build_failed.log') -Force }
    Select-String -Path $log -Pattern 'ERROR|does not exist|Error' -ErrorAction SilentlyContinue |
        Select-Object -First 15 | ForEach-Object { "  " + $_.Line.Trim() }
    exit 1
}
$errn = (Select-String -Path $log -Pattern 'ERROR' | Measure-Object).Count
if ($errn -ne 0) {
    Write-Host "日志里有 $errn 条 ERROR，请先检查。" -ForegroundColor Red
    Select-String -Path $log -Pattern 'ERROR' | Select-Object -First 10 | ForEach-Object { "  " + $_.Line.Trim() }
}

# ---------- 5) 归档到 固件\ ----------
if (-not (Test-Path $fwDir)) { New-Item -ItemType Directory -Force -Path $fwDir | Out-Null }
$dst = Join-Path $fwDir $OutName
Copy-Item $fs $dst -Force

$bb = [System.IO.File]::ReadAllBytes($dst)
$head = [System.Text.Encoding]::ASCII.GetString($bb[0..1199])
$cs = '0x????'
if ($head -match 'CheckSum\s*:\s*(0x[0-9A-Fa-f]+)') { $cs = $matches[1] }

$rpt = Join-Path $buildDir 'impl\pnr\f1_attitude.rpt.txt'
$logic = '?'; $lut = '?'; $alu = '?'; $reg = '?'; $latch = '?'; $dsp = '?'
if (Test-Path $rpt) {
    $r = Get-Content $rpt
    $q = $r | Select-String -Pattern 'Logic\s*\|\s*(\d+)/' | Select-Object -First 1
    if ($q) { $logic = $q.Matches[0].Groups[1].Value }
    $q = $r | Select-String -Pattern '--LUT,ALU,ROM16\s*\|\s*(\d+)\((\d+) LUT,\s*(\d+) ALU' | Select-Object -First 1
    if ($q) { $lut = $q.Matches[0].Groups[2].Value; $alu = $q.Matches[0].Groups[3].Value }
    $q = $r | Select-String -Pattern '^\s*Register\s*\|\s*(\d+)/' | Select-Object -First 1
    if ($q) { $reg = $q.Matches[0].Groups[1].Value }
    $q = $r | Select-String -Pattern 'Logic Register as Latch\s*\|\s*(\d+)/' | Select-Object -First 1
    if ($q) { $latch = $q.Matches[0].Groups[1].Value }
    $q = $r | Select-String -Pattern 'DSP\s*\|\s*([\d.]+)/' | Select-Object -First 1
    if ($q) { $dsp = $q.Matches[0].Groups[1].Value }
}

# 哈希取【原始源码】而不是注入后的临时副本，这样 sidecar 记录的是
# 源码目录里那份文件的真实哈希，可与 (Get-FileHash .\源码\top.v) 直接比对。
$origTop = $srcDirs | ForEach-Object { Join-Path $_ 'top.v' } | Where-Object { Test-Path $_ } | Select-Object -First 1
$origPd = $srcDirs | ForEach-Object { Join-Path $_ 'pd_ctrl.v' } | Where-Object { Test-Path $_ } | Select-Object -First 1
$sha1Top = (Get-FileHash $origTop -Algorithm SHA1).Hash
$sha1Pd = (Get-FileHash $origPd -Algorithm SHA1).Hash

$side = @(
    "file      : $OutName",
    "built     : $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')",
    "STAGE     : $Stage",
    "OS_CLKDIV : $I2cDiv",
    "trans_ms  : $([math]::Round(205*$I2cDiv/50000.0,1))",
    "trans_hz  : $([math]::Round($hz,1))",
    "checksum  : $cs",
    "bytes     : $((Get-Item $dst).Length)",
    "logic     : $logic LU ($lut LUT + $alu ALU)",
    "register  : $reg     latches: $latch",
    "dsp       : $dsp/10",
    "src_sha1  : top.v=$($sha1Top.Substring(0,12)) pd_ctrl.v=$($sha1Pd.Substring(0,12))",
    "            (源码 top.v 原样哈希：参数注入前从原文件读取，未被改动)"
)
[System.IO.File]::WriteAllLines("$dst.txt", $side, (New-Object System.Text.UTF8Encoding($false)))

Write-Host ""
Write-Host ("完成  ->  $dst   CheckSum $cs   $((Get-Item $dst).Length) bytes") -ForegroundColor Green
Write-Host ("资源  logic=$logic LU ($lut LUT + $alu ALU)  reg=$reg  latch=$latch  dsp=$dsp") -ForegroundColor Cyan
Write-Host "烧录后请核对 User Code 与上面 CheckSum 一致。" -ForegroundColor Cyan
