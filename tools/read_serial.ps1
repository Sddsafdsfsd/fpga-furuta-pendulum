<#
  read_serial.ps1 —— 读 FPGA 遥测 + 可选发送命令   （v4）

  用法：
    # 1) 纯观察（实时）
    powershell -NoProfile -ExecutionPolicy Bypass -File read_serial.ps1 -Seconds 30 -Live

    # 2) 边发命令边看数据（-Send：多条用 ; 分隔，最后一条按 -SendGapMs 重复发）
    powershell -NoProfile -ExecutionPolicy Bypass -File read_serial.ps1 -Seconds 20 -Live -Send "E1;P300"
    powershell -NoProfile -ExecutionPolicy Bypass -File read_serial.ps1 -Seconds 10 -Live -Send "P0;E0"

    # 3) 保存数据 + 自动质量分析（用于 AS5600 稳定性/线性度验证）
    powershell -NoProfile -ExecutionPolicy Bypass -File read_serial.ps1 -Seconds 120 -Live -Save "E:\deepseek工作区\f1_attitude\logs\testA.csv"

  遥测格式：A<4位> T<4位> E<符号4位> M<符号4位>

  v4 新增：
    -Save <csv路径>   把每帧 (时刻s,A,T,E,M) 写入 CSV，便于事后分析
    结束时自动给出【质量报告】：帧率、A 的噪声(静止抖动)、跳变次数(排除 0/4095 绕回)
#>
param(
    [int]$Seconds = 10,
    [string]$Port = '',
    [switch]$Live,
    [int]$Every = 10,
    [string]$Send = '',
    [int]$SendGapMs = 1000,
    [string]$Save = ''
)

if ([string]::IsNullOrEmpty($Port)) {
    $dev = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue |
           Where-Object { $_.Class -eq 'Ports' -and $_.FriendlyName -match 'CH340' } |
           Select-Object -First 1
    if (-not $dev) {
        Write-Host "没找到 CH340 串口。" -ForegroundColor Yellow
        Write-Host "检查: 线是否插在核心板 J2(串口口); 另一端是否插在电脑上。"
        exit 1
    }
    if ($dev.FriendlyName -match '\((COM\d+)\)') { $Port = $Matches[1] }
}

if ($Live) {
    Write-Host ("使用串口 {0}，实时采集 {1} 秒（Ctrl+C 可提前结束）……" -f $Port, $Seconds) -ForegroundColor Cyan
    Write-Host  "     A=角度/ADC  T=目标/磁场  E=编码器/STATUS  M=电机命令/AGC" -ForegroundColor DarkGray
} else {
    Write-Host ("使用串口 {0}，采集 {1} 秒……" -f $Port, $Seconds) -ForegroundColor Cyan
}
if ($Send -ne '') {
    Write-Host ("将依次发送命令：{0}（每条间隔 {1} ms，最后一条重复）" -f $Send, $SendGapMs) -ForegroundColor Yellow
}
if ($Save -ne '') {
    $saveDir = Split-Path -Parent $Save
    if ($saveDir -and -not (Test-Path $saveDir)) { New-Item -ItemType Directory -Force -Path $saveDir | Out-Null }
    Write-Host ("数据将保存到：{0}" -f $Save) -ForegroundColor Yellow
}

$sp = New-Object System.IO.Ports.SerialPort $Port,115200,'None',8,'One'
$sp.ReadTimeout = 2000
try {
    $sp.Open()
    $end = (Get-Date).AddSeconds($Seconds)
    $n = 0; $aMin = 99999; $aMax = -1; $eMin = 999999; $eMax = -999999; $shown = 0
    $t0 = Get-Date

    # 质量分析用的累加器
    $rows      = New-Object System.Collections.Generic.List[string]
    $prevA     = -1
    $prevWrap  = $false
    $jumps     = 0          # 非绕回的大跳变次数
    $maxJump   = 0
    $sumAbsD   = 0.0; $cntD = 0
    $aValues   = New-Object System.Collections.Generic.List[int]

    $cmdList = @()
    if ($Send -ne '') {
        $cmdList = @($Send -split ';' | ForEach-Object { $_.Trim() } | Where-Object { $_ -ne '' })
    }
    $cmdIdx   = 0
    $nextSend = Get-Date

    while ((Get-Date) -lt $end) {
        if ($cmdList.Count -gt 0 -and (Get-Date) -ge $nextSend) {
            if ($cmdIdx -lt $cmdList.Count) { $c = $cmdList[$cmdIdx] } else { $c = $cmdList[$cmdList.Count - 1] }
            $bytes = [System.Text.Encoding]::ASCII.GetBytes($c + "`r`n")
            $sp.Write($bytes, 0, $bytes.Length)
            Write-Host ("  >>> 发送命令: {0}" -f $c) -ForegroundColor Yellow
            if ($cmdIdx -lt $cmdList.Count) { $cmdIdx++ }
            $nextSend = (Get-Date).AddMilliseconds($SendGapMs)
        }

        try { $l = $sp.ReadLine().Trim() } catch { continue }
        if ($l -eq '') { continue }
        if (-not $Live -and $shown -lt 5) { Write-Output ("  " + $l); $shown++ }

        if ($l -match 'A(\d{4})\s+T(\d{4})\s+E([+-]\d{4})\s+M([+-]\d{4})') {
            $a = [int]$Matches[1]; $tg = [int]$Matches[2]; $e = [int]$Matches[3]; $m = [int]$Matches[4]
            if ($a -lt $aMin) { $aMin = $a }
            if ($a -gt $aMax) { $aMax = $a }
            if ($e -lt $eMin) { $eMin = $e }
            if ($e -gt $eMax) { $eMax = $e }
            $n++
            $sNow = [math]::Round(((Get-Date) - $t0).TotalSeconds, 3)

            # --- 质量统计 ---
            $aValues.Add($a)
            if ($prevA -ge 0) {
                $d = [math]::Abs($a - $prevA)
                # 绕回判定：0 与 4095 互为邻点
                if ($d -gt 2048) {
                    $d = 4096 - $d      # 折算成"实际转动量"
                    $prevWrap = $true
                }
                $sumAbsD += $d; $cntD++
                if ($d -gt $maxJump) { $maxJump = $d }
                if ($d -gt 300) { $jumps++ }   # 单次采样跳变 > 300 计数（约 26°）视为异常
            }
            $prevA = $a

            if ($Save -ne '') { $rows.Add(("{0},{1},{2},{3},{4}" -f $sNow, $a, $tg, $e, $m)) }

            if ($Live -and ($n % $Every -eq 0)) {
                $s = [int]((Get-Date) - $t0).TotalSeconds
                Write-Host ("  [{0,3}s] A={1,4}  T={2,4}  E={3,5}  M={4,5}" -f $s, $a, $tg, $e, $m)
            }
        }
    }

    if ($cmdList.Count -gt 0) {
        try {
            $stopBytes = [System.Text.Encoding]::ASCII.GetBytes("P0`r`nE0`r`n")
            $sp.Write($stopBytes, 0, $stopBytes.Length)
            Write-Host "  >>> 已发送停止命令 P0 / E0（安全）" -ForegroundColor Green
        } catch {}
    }

    if ($Save -ne '' -and $rows.Count -gt 0) {
        $hdr = "time_s,A,T,E,M"
        [System.IO.File]::WriteAllLines($Save, (@($hdr) + $rows), (New-Object System.Text.UTF8Encoding($false)))
        Write-Host ("  已保存 {0} 帧到 {1}" -f $rows.Count, $Save) -ForegroundColor Green
        $savedRows = $true
    }

    if ($n -gt 0) {
        Write-Host ""
        Write-Host ("收到 {0} 帧；A 范围 [{1}..{2}]；E 范围 [{3}..{4}] 跨度 {5}" -f $n, $aMin, $aMax, $eMin, $eMax, ($eMax - $eMin)) -ForegroundColor Green

        # ---- 质量报告 ----
        Write-Host ""
        Write-Host "===== 质量报告 =====" -ForegroundColor Cyan
        Write-Host ("  帧率            : {0:N1} 帧/秒（总 {1} 帧 / {2} 秒）" -f ($n / [math]::Max(1,$Seconds)), $n, $Seconds)
        Write-Host ("  A 取值范围      : {0} ~ {1}（跨度 {2}）" -f $aMin, $aMax, ($aMax - $aMin))
        if ($cntD -gt 0) {
            $meanD = $sumAbsD / $cntD
            Write-Host ("  相邻帧平均变化  : {0:N1} 计数/帧（静止时应接近 0）" -f $meanD)
            Write-Host ("  单帧最大跳变    : {0} 计数（约 {1:N1}°）" -f $maxJump, ($maxJump * 360.0 / 4096.0))
            Write-Host ("  异常跳变次数    : {0} 次（单帧 >300 计数，已排除 0/4095 绕回）" -f $jumps)
            if ($jumps -eq 0 -and $meanD -lt 30) {
                Write-Host "  => 读数非常稳定 ✓（无异常跳变）" -ForegroundColor Green
            } elseif ($jumps -lt ($n * 0.01)) {
                Write-Host "  => 读数基本稳定 ✓（偶发跳变 <1%，可接受）" -ForegroundColor Green
            } else {
                Write-Host "  => 读数抖动偏多 ✗（${jumps} 次）—— 可能是磁铁手持不稳、或需要加上拉电阻" -ForegroundColor Yellow
            }
        }
        if ($aMax -ge 4095 -and $aMin -le 5) {
            Write-Host "  => 覆盖 0~4095 全量程 ✓（转过了完整一圈）" -ForegroundColor Green
        }
    } else {
        Write-Host "没解析到遥测帧——确认烧的固件带遥测，且烧写后没断过电" -ForegroundColor Yellow
    }
} finally {
    # ★ Ctrl+C 会跳出上面的正常保存分支，数据会全部丢掉（v37/v38 就是这样丢的）。
    #   这里兜底再存一次：只要还没存过、且有数据，就一定落盘。
    if ($Save -ne '' -and -not $savedRows -and $rows -and $rows.Count -gt 0) {
        try {
            $hdr = "time_s,A,T,E,M"
            [System.IO.File]::WriteAllLines($Save, (@($hdr) + $rows), (New-Object System.Text.UTF8Encoding($false)))
            Write-Host ("  [中断] 已抢救 {0} 帧到 {1}" -f $rows.Count, $Save) -ForegroundColor Yellow
        } catch {}
    }
    if ($sp.IsOpen) { $sp.Close() }
}
