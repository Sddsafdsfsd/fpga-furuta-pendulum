<#
  program_f1.ps1 —— 用云源命令行把码流烧进 F1（先把线插到核心板 J1 下载口）  v2

  v2 新增：
    · 自动尝试 cable-index 0~3，找到能用的那条线（解决 "Cable failed to open"）
    · 烧写失败自动重试一次
    · 失败时打印明确的排查清单
  用法：
    powershell -NoProfile -ExecutionPolicy Bypass -File program_f1.ps1 -Fs "...\xxx.fs"
    powershell -NoProfile -ExecutionPolicy Bypass -File program_f1.ps1 -Mode scan
#>
param(
    [string]$Gowin    = 'E:\Installation_package\Gowin\Gowin_V1.9.11.03_Education_x64',
    [string]$Project  = 'E:\Installation_package\Gowin\Gowin_V1.9.11.03_Education_x64\IDE\bin\Documents\f1_attitude',
    [ValidateSet('scan', 'sram', 'sram-noverify', 'flash')]
    [string]$Mode     = 'sram-noverify',
    [string]$Fs       = '',
    [string]$Cable    = 'Gowin USB Cable(GWU2X)',
    [int]$CableIndex  = -1
)

$cli = Join-Path $Gowin 'Programmer\bin\programmer_cli.exe'
if (-not (Test-Path $cli)) { throw "找不到 programmer_cli.exe：$cli" }
if ([string]::IsNullOrEmpty($Fs)) { $Fs = Join-Path $Project 'impl\pnr\f1_attitude.fs' }
if (-not (Test-Path $Fs)) { throw "找不到码流文件：$Fs" }

if ($Fs -match '[^\x00-\x7F]') {
    $dst = Join-Path $Project 'flash_tmp.fs'
    Copy-Item $Fs $dst -Force
    Write-Host ("已复制到英文路径：" + $dst) -ForegroundColor DarkGray
    $Fs = $dst
}

function Show($out) {
    ($out -split "`r?`n") |
        Where-Object { $_ -notmatch 'Programing:|Programming|Reading SRAM|Writing|Erasing' -and $_.Trim() -ne '' } |
        ForEach-Object { $_.TrimEnd() }
}

function Show-Tips {
    Write-Host ""
    Write-Host "==> 下载线打不开，请按顺序检查：" -ForegroundColor Yellow
    Write-Host "   ① 拔掉两根 USB 线，等 5 秒，只插 J1（下载口）→ 重新运行本命令" -ForegroundColor Yellow
    Write-Host "   ② 关掉可能占用串口/下载器的程序（串口助手、我的 read_serial.ps1、云源 IDE 的 Programmer）" -ForegroundColor Yellow
    Write-Host "   ③ 换一个电脑 USB 口（直插主板，不要用扩展坞/USB Hub）" -ForegroundColor Yellow
    Write-Host "   ④ 设备管理器里看有没有出现 'Gowin USB Cable'；没有就是线或口的问题" -ForegroundColor Yellow
}

# ---- 先找到可用的下载线编号 ----
$cand = if ($CableIndex -ge 0) { @($CableIndex) } else { @(0, 1, 2, 3) }
$good = -1
Write-Host "正在探测可用的下载线……" -ForegroundColor Cyan
foreach ($i in $cand) {
    $out = (& $cli --device GW1N-9C --run 0 --cable $Cable --cable-index $i 2>&1) -join "`n"
    if ($out -match 'ID Code is') {
        $good = $i
        Write-Host ("  ✅ 找到下载线：cable-index = $i（器件 ID 读取成功）") -ForegroundColor Green
        break
    } else {
        $reason = if ($out -match 'Cable failed to open') { 'Cable failed to open' }
                  elseif ($out -match 'No Gowin devices') { 'No Gowin devices found' }
                  else { '无响应' }
        Write-Host ("  ✗ cable-index $i ：$reason") -ForegroundColor DarkGray
    }
}

if ($good -lt 0) { Show-Tips; exit 50 }
if ($Mode -eq 'scan') { Write-Host "==> 下载链路正常（退出码 0）" -ForegroundColor Green; exit 0 }

$runMap = @{ 'sram' = 4; 'sram-noverify' = 2; 'flash' = 6 }
$run = $runMap[$Mode]
$word = switch ($Mode) { 'sram' { '烧到 SRAM（带校验）' } 'sram-noverify' { '烧到 SRAM（不校验，推荐）' } 'flash' { '写内部 Flash（掉电保存）' } }
Write-Host ("$word，cable-index = $good ……") -ForegroundColor Cyan

$code = 1
for ($attempt = 1; $attempt -le 2; $attempt++) {
    $out = & $cli --device GW1N-9C --run $run --fsFile $Fs --cable $Cable --cable-index $good 2>&1
    Show $out
    if ($LASTEXITCODE -eq 0) { $code = 0; break }
    $code = $LASTEXITCODE
    if ($attempt -eq 1) { Write-Host "第一次失败，重试一次……" -ForegroundColor Yellow; Start-Sleep -Seconds 2 }
}

if ($code -eq 0) {
    Write-Host "==> 成功（退出码 0）" -ForegroundColor Green
}
elseif ($code -eq 18) {
    Write-Host "==> 成功（退出码 18 = 仅 SRAM 回读校验失败）" -ForegroundColor Green
    Write-Host "    本板 GWU2X 的 SRAM 回读校验不可用：10/7 那个能正常跑的码流同样报 18/Status 0x0003F020。" -ForegroundColor DarkGray
    Write-Host "    => 配置本身已写进去了，功能请用串口遥测验证，不要看校验结果。" -ForegroundColor DarkGray
}
else {
    Write-Host ("==> 失败：退出码 " + $code + "（2/4=烧写失败）") -ForegroundColor Red
    Show-Tips
}